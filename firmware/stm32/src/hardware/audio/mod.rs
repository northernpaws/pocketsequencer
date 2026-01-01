mod codec;

use defmt::{error, info};
use derive_more::{Display, Error};

use embassy_stm32::hrtim::Master;
use grounded::uninit::GroundedArrayCell;

use embassy_embedded_hal::shared_bus::I2cDeviceError;
use embassy_embedded_hal::shared_bus::asynch::{self};
use embassy_executor::{InterruptExecutor, SpawnError};
use embassy_stm32::i2c::{self, I2c};
use embassy_stm32::interrupt::{InterruptExt, Priority};
use embassy_stm32::sai::MasterClockDivider;
use embassy_stm32::{interrupt, mode, peripherals, rcc, sai};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::Timer;

use catalina::engine::{
    audio::Frame,
    audio::oscillator::{self, Oscillator},
    audio::sample::U24,
    core::Hertz,
};

use nau88c22_rs::clock::CodecClockError;

use crate::hardware::CodecSAIResources;

const OUTPUT_CHANNEL_COUNT: usize = 2; // stereo

// Note that the block size needs to be big enough to where the DMA/SAI
// has enough to work with before the next cycle to generate more.
pub const BLOCK_LENGTH: usize = 128; // samples
pub const HALF_DMA_BUFFER_LENGTH: usize = (BLOCK_LENGTH) * OUTPUT_CHANNEL_COUNT; //  2 channels
pub const DMA_BUFFER_LENGTH: usize = HALF_DMA_BUFFER_LENGTH * 2; //  2 half-blocks

pub const SAMPLE_RATE: u32 = 48000;

// #[unsafe(link_section = ".sram1_bss")]
static AUDIO_TX_BUFFER: GroundedArrayCell<u32, { DMA_BUFFER_LENGTH }> = GroundedArrayCell::uninit();
// #[unsafe(link_section = ".sram1_bss")]
static AUDIO_RX_BUFFER: GroundedArrayCell<u32, { DMA_BUFFER_LENGTH }> = GroundedArrayCell::uninit();

pub type SAITransmitter<'a> = sai::Sai<'a, peripherals::SAI1, u32>;
pub type SAIReceiver<'a> = sai::Sai<'a, peripherals::SAI1, u32>;

static AUDIO_EXECUTOR: InterruptExecutor = InterruptExecutor::new();

#[interrupt]
unsafe fn SAI1() {
    unsafe { AUDIO_EXECUTOR.on_interrupt() }
}

pub struct Audio<'a> {
    codec: nau88c22_rs::Nau88c22<
        asynch::i2c::I2cDevice<'a, CriticalSectionRawMutex, I2c<'static, mode::Async, i2c::Master>>,
    >,
}

/// Errors the happen when initializing the audio component.
#[derive(Debug, Display, Error)]
pub enum InitError {
    /// Occurs if there was an issue communicating with one of the I2C devices.
    #[display("{_0:?}")]
    I2CError(#[error(not(source))] I2cDeviceError<embassy_stm32::i2c::Error>),

    /// Occurs if there was an error spawning one of the tasks.
    #[display("{_0:?}")]
    SpawnError(#[error(not(source))] SpawnError),

    /// Occurs if the codec master clock prescaler and PLL is not resolvable.
    CodecError(nau88c22_rs::InitError<I2cDeviceError<embassy_stm32::i2c::Error>>),
}

impl From<I2cDeviceError<embassy_stm32::i2c::Error>> for InitError {
    fn from(value: I2cDeviceError<embassy_stm32::i2c::Error>) -> Self {
        Self::I2CError(value)
    }
}

impl From<SpawnError> for InitError {
    fn from(value: SpawnError) -> Self {
        Self::SpawnError(value)
    }
}

impl From<nau88c22_rs::InitError<I2cDeviceError<embassy_stm32::i2c::Error>>> for InitError {
    fn from(value: nau88c22_rs::InitError<I2cDeviceError<embassy_stm32::i2c::Error>>) -> Self {
        Self::CodecError(value)
    }
}

impl<'a> Audio<'a> {
    pub async fn new(
        device: asynch::i2c::I2cDevice<
            'static,
            CriticalSectionRawMutex,
            I2c<'static, mode::Async, i2c::Master>,
        >,
        sai_resources: CodecSAIResources,
        // sai_transmitter: SAITransmitter<'static>,
        // mut sai_receiver: SAIReceiver<'static>,
    ) -> Result<Self, InitError> {
        // Calculate the SAI master clock divisor and derrived codec master clock divisor and PLL.
        let kernel_clock = rcc::frequency::<peripherals::SAI1>().0;
        let mclk_div: MasterClockDivider =
            mclk_div_from_u8((kernel_clock / (SAMPLE_RATE * 256)) as u8);
        let mclk_div_u8: u8 = mclk_div.into();
        let adjusted_mclk = kernel_clock / mclk_div_u8 as u32;
        info!("SAI1 master clock base: {}hz", kernel_clock);
        info!("SAI1 master clock divisor: {}", mclk_div);
        info!("SAI1 master clock adjusted: {}hz", adjusted_mclk);

        info!("Ideal SAI clock for sample rate: {}", SAMPLE_RATE * 256);

        let actual_sample_rate = adjusted_mclk as f32 / 256.0;
        info!(
            "SAI clock skew: {}% actual_sample_rate={}hz",
            (adjusted_mclk as f32 / (SAMPLE_RATE as f32 * 256.0)) as f32 * 100.0,
            actual_sample_rate
        );

        let mut codec = nau88c22_rs::Nau88c22::new(device);

        // After the transmitter has been configured, mclk goes active.
        // Now we can configure our codec to run off the mclk signal.

        info!("initializing audio codec...");
        codec::codec_init(&mut codec, adjusted_mclk, actual_sample_rate).await;

        // Codec wants delay from mclk start to sending frames
        Timer::after_millis(250).await;

        // Spawn the task that processes the codec data.
        info!("spawning SAI loop");

        // Use an interrupt executor to run the audio task at a higher priority.
        interrupt::SAI1.set_priority(Priority::P6);
        let spawner = AUDIO_EXECUTOR.start(interrupt::SAI1);
        spawner.spawn(device_loop(
            mclk_div.into(),
            sai_resources, /*sai_receiver*/
            actual_sample_rate,
        )?);

        Ok(Self { codec })
    }
}

#[embassy_executor::task]
async fn device_loop(
    mclk_div: MasterClockDivider,
    sai_resources: CodecSAIResources,
    sample_rate: f32,
    // sai_receiver: SAIReceiver<'static>,
) -> ! {
    // should never return
    let err = inner_device_loop(
        mclk_div,
        sai_resources,
        sample_rate, /* , sai_receiver*/
    )
    .await;
    panic!("{:?}", err);
}

#[derive(Debug)]
enum Never {}

async fn inner_device_loop(
    mclk_div: MasterClockDivider,
    mut sai_resources: CodecSAIResources,
    sample_rate: f32,
    // mut sai_receiver: SAIReceiver<'static>,
) -> Result<(), Never> {
    info!("initializing SAI buffers...");

    let audio_tx_buffer: &mut [u32] = unsafe {
        AUDIO_TX_BUFFER.initialize_all_copied(0);
        let (ptr, len) = AUDIO_TX_BUFFER.get_ptr_len();
        core::slice::from_raw_parts_mut(ptr, len)
    };

    let audio_rx_buffer: &mut [u32] = unsafe {
        AUDIO_RX_BUFFER.initialize_all_copied(0);
        let (ptr, len) = AUDIO_RX_BUFFER.get_ptr_len();
        core::slice::from_raw_parts_mut(ptr, len)
    };

    // Initialize the SAI receiver.
    // TODO: remove unwrap and use proper error type

    let (mut sai_transmitter, mut sai_receiver) = setup_sai(
        &mut sai_resources,
        audio_tx_buffer,
        audio_rx_buffer,
        mclk_div,
    );

    let mut osc = oscillator::RuntimeOscillator::new(
        oscillator::OscillatorType::Sine,
        // NOTE: We pass in the actual sampling rate calculated from the SAI clock
        //  difference, instead of hard-coding our desired 48kHz sample rate because
        //  any clock skew will shift the frequency otherwise.
        sample_rate as usize,
        Hertz::from_hertz(261.63), // middle C
    );

    info!("starting SAI receiver");
    sai_receiver.start().unwrap();

    info!("starting SAI loop");
    let mut buf = [0u32; HALF_DMA_BUFFER_LENGTH];
    const AMPLITUDE: f32 = 0.25;
    loop {
        // Chunk the sample buffer into left and right channel frames.
        for frame in buf.chunks_mut(OUTPUT_CHANNEL_COUNT) {
            // Output is 24-bit I2S data, so format
            // the oscillator sample to U24 sizes.
            let value: f32 = osc.sample(); // U24

            for sample in frame.iter_mut() {
                // Note that we use value.inner() instead of to_sample().
                //
                // to_sample() would convert to a u32 range, but the I2S
                // sample audio format is actually 24-bit so we want to
                // cap it at 24.
                // *sample = value.inner().scale_amp(0.25) as u32;
                *sample = (((value + 2.0) * (0xFFFFFF as f32 / 2.0)) * AMPLITUDE) as u32;
            }
        }

        // A write() must be called before read() to start the
        // master (transmitter) clock used by the receiver.
        match sai_transmitter.write(&buf).await {
            Ok(_) => {
                // info!("Write ok!");
            }
            Err(err) => {
                error!("Recreating SAI due to transmit error: {}", err);

                // In the event of an overrun error, re-create the SAI devices to clear the DMA status.
                drop(sai_transmitter);
                drop(sai_receiver);

                (sai_transmitter, sai_receiver) = setup_sai(
                    &mut sai_resources,
                    audio_tx_buffer,
                    audio_rx_buffer,
                    mclk_div,
                );
            }
        }

        /*match sai_receiver.read(&mut buf).await {
            Ok(_) => {
                // info!("Read ok!");
            }
            Err(err) => {
                error!("Recreating SAI due to receive error: {}", err);

                info!("SAI receive buffer size: {}", rx_size);
                info!("SAI block transfer size: {}", buf.len());

                // In the event of an overrun error, re-create the SAI devices to clear the DMA status.
                drop(sai_transmitter);
                drop(sai_receiver);

                (sai_transmitter, sai_receiver) = setup_sai(
                    &mut sai_resources,
                    audio_tx_buffer,
                    audio_rx_buffer,
                    mclk_div,
                );
            }
        }*/

        // info!("sai tick")

        // info!("sai receive: {}", buf[0]);
    }
}

fn mclk_div_from_u8(v: u8) -> MasterClockDivider {
    if v == 0 {
        return MasterClockDivider::DIV1;
    }

    assert!((1..=63).contains(&v));
    MasterClockDivider::from_bits(v)
}

fn setup_sai<'d>(
    sai_resources: &'d mut CodecSAIResources,
    transmit_buffer: &'d mut [u32],
    receive_buffer: &'d mut [u32],
    mclk_div: MasterClockDivider,
) -> (SAITransmitter<'d>, SAIReceiver<'d>) {
    // Derrive the required SAI clock divider based on
    // the configured sample rate for the audio engine.
    // let kernel_clock = rcc::frequency::<peripherals::SAI1>().0;
    // let mclk_div = mclk_div_from_u8((kernel_clock / (SAMPLE_RATE * 256)) as u8);

    // info!("SAI1 master clock: {}", kernel_clock);
    // info!("SAI1 master clock divider: {}", mclk_div);

    // The SAI config used for the A block configured in transmit mode.
    //
    // Configured for standard I2S.
    let mut tx_config = sai::Config::default();
    tx_config.mode = sai::Mode::Master;
    tx_config.tx_rx = sai::TxRx::Transmitter;
    tx_config.stereo_mono = sai::StereoMono::Stereo;
    tx_config.output_drive = sai::OutputDrive::OnStart; // disabled

    if mclk_div != MasterClockDivider::_RESERVED_0 {
        tx_config.master_clock_divider = mclk_div.into();
        tx_config.nodiv = false;
    } else {
        tx_config.master_clock_divider = MasterClockDivider::DIV1;
        tx_config.nodiv = true;
    }

    tx_config.fifo_threshold = sai::FifoThreshold::Empty;
    tx_config.companding = sai::Companding::None;
    tx_config.protocol = sai::Protocol::Free; // free for i2s
    tx_config.bit_order = sai::BitOrder::MsbFirst; // nac88c22 runs in MSB
    tx_config.clock_strobe = sai::ClockStrobe::Falling;
    tx_config.frame_sync_definition = sai::FrameSyncDefinition::ChannelIdentification;
    tx_config.slot_enable = 0xFFFF; // All configured slots active
    tx_config.slot_count = sai::word::U4(OUTPUT_CHANNEL_COUNT as u8); // The number of slots in the audio frame.
    tx_config.first_bit_offset = sai::word::U5(0);
    tx_config.frame_sync_polarity = sai::FrameSyncPolarity::ActiveLow;
    tx_config.frame_sync_offset = sai::FrameSyncOffset::BeforeFirstBit;
    tx_config.data_size = sai::DataSize::Data24;
    tx_config.frame_length = 64; // 64U * (nbslot / 2U)
    tx_config.frame_sync_active_level_length = sai::word::U7(32); // 32U * (nbslot / 2U)
    tx_config.slot_size = sai::SlotSize::Channel32; // sai::SlotSize::DataSize

    // The SAI config used for the B block configured in receive mode.
    let mut rx_config = tx_config.clone();
    rx_config.mode = sai::Mode::Slave; // slaved to the transmitter block
    rx_config.tx_rx = sai::TxRx::Receiver; // configure this block as a receiver
    rx_config.sync_input = sai::SyncInput::Internal; // passes sync to the second block
    rx_config.sync_output = false; // passes sync to the second block
    rx_config.clock_strobe = sai::ClockStrobe::Rising; // nac88c22 uses rising edge latching on bclk

    // Split the SAI periperal into it's two subblocks.
    let (sub_block_tx, sub_block_rx) = sai::split_subblocks(sai_resources.peri.reborrow());

    // Configure the first sub block as the transmitter and the main clock source.
    let sai_tx = sai::Sai::new_asynchronous_with_mclk(
        sub_block_tx,
        sai_resources.sck.reborrow(),
        sai_resources.dacdat.reborrow(),
        sai_resources.fs.reborrow(),
        sai_resources.mclk.reborrow(),
        sai_resources.tx_dma.reborrow(),
        transmit_buffer,
        tx_config,
    );

    // Configure the second sub block as a receiver,
    // syncronous to the mclk of the transmitter block.
    let sai_rx = sai::Sai::new_synchronous(
        sub_block_rx,
        sai_resources.adcdat.reborrow(),
        sai_resources.rx_dma.reborrow(),
        receive_buffer,
        rx_config,
    );

    (sai_tx, sai_rx)
}
