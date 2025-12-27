use core::f32::consts::PI;

use defmt::{error, info};
use derive_more::{Display, Error};
use embassy_embedded_hal::shared_bus::I2cDeviceError;
use embassy_embedded_hal::shared_bus::asynch::{self};
use embassy_executor::{SpawnError, Spawner};
use embassy_stm32::i2c::{self, I2c};
use embassy_stm32::sai::MasterClockDivider;
use embassy_stm32::{mode, peripherals, rcc, sai};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::Timer;
use grounded::uninit::GroundedArrayCell;

use crate::hardware::CodecSAIResources;

pub const BLOCK_LENGTH: usize = 32; // 32 samples
pub const HALF_DMA_BUFFER_LENGTH: usize = (BLOCK_LENGTH * 8) * 2; //  2 channels
pub const DMA_BUFFER_LENGTH: usize = HALF_DMA_BUFFER_LENGTH * 2; //  2 half-blocks
pub const SAMPLE_RATE: u32 = 48000;

static AUDIO_TX_BUFFER: GroundedArrayCell<u32, { DMA_BUFFER_LENGTH }> = GroundedArrayCell::uninit();
static AUDIO_RX_BUFFER: GroundedArrayCell<u32, { DMA_BUFFER_LENGTH }> = GroundedArrayCell::uninit();

pub type SAITransmitter<'a> = sai::Sai<'a, peripherals::SAI1, u32>;
pub type SAIReceiver<'a> = sai::Sai<'a, peripherals::SAI1, u32>;

pub struct Audio<'a, DELAY: embedded_hal_async::delay::DelayNs> {
    codec: nau88c22_rs::Nau88c22<
        asynch::i2c::I2cDevice<'a, CriticalSectionRawMutex, I2c<'static, mode::Async, i2c::Master>>,
    >,

    delay: DELAY,
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
impl<'a, DELAY: embedded_hal_async::delay::DelayNs> Audio<'a, DELAY> {
    pub async fn new(
        device: asynch::i2c::I2cDevice<
            'static,
            CriticalSectionRawMutex,
            I2c<'static, mode::Async, i2c::Master>,
        >,
        sai_resources: CodecSAIResources,
        // sai_transmitter: SAITransmitter<'static>,
        // mut sai_receiver: SAIReceiver<'static>,
        mut delay: DELAY,
        spawner: Spawner,
    ) -> Result<Self, InitError> {
        let mut codec = nau88c22_rs::Nau88c22::new(device);

        // Software reset the codec to a known state.
        info!("software resetting audio codec");
        codec.reset().await?;

        // Give time for the reset to finish.
        delay.delay_ms(200).await;

        let device_id = codec.read_deviceid().await?;
        info!("audio codec device ID: {}", device_id.id());

        codec
            .modify_clockcontrol1(|reg| {
                reg.with_clkm(false) // MCLK, pin#11 used as master clock
            })
            .await?;

        codec
            .modify_powermanagement1(|reg| {
                reg.with_dcbufen(true)
                    .with_aux1mxen(true)
                    .with_aux2mxen(true)
                    .with_pllen(false)
                    .with_micbiasen(true)
                    .with_abiasen(true)
                    .with_iobufen(true)
                    .with_refimp(0b11)
                    .with_dcbufen(false) // false for lower then 3.6v operation
            })
            .await?;

        // Datasheet sets to wait 250ms after setting IOBUFEN, DCBUFEN,
        // REFIMP and ABIASEN to charge output capacitors.
        Timer::after_millis(250).await;

        codec
            .modify_powermanagement2(|reg| {
                reg.with_rhpen(false)
                    .with_lphen(false)
                    .with_sleep(false)
                    .with_rbsten(true)
                    .with_lbsten(true)
                    .with_rpgaen(true)
                    .with_lpgaen(true)
                    .with_radcen(true)
                    .with_ladcen(true)
            })
            .await?;

        codec
            .modify_powermanagement3(|reg| {
                reg.with_auxout1en(true)
                    .with_auxout2en(true)
                    .with_lspken(false)
                    .with_rspken(false)
                    .with_rmixen(true)
                    .with_lmixen(true)
                    .with_rdacen(true)
                    .with_ldacen(true)
            })
            .await?;

        // TODO: configure PLL

        // Wait for the PLL to stabalize.
        Timer::after_millis(255).await;

        // Enable the PLL.
        codec
            .modify_powermanagement1(|reg| reg.with_pllen(true))
            .await?;

        // Enable the left aux in as the left ADC source.
        codec
            .modify_leftadcboost(|reg| {
                reg.with_lauxbstegain(0b101)
                    .with_lpgabst(false)
                    .with_lpgabstgaun(0)
            })
            .await?;

        // Enable the right aux in as the right ADC source.
        codec
            .modify_rightadcboost(|reg| {
                reg.with_rauxbstgain(0b101)
                    .with_rpgabst(false)
                    .with_rpgabstgain(0)
            })
            .await?;

        // Set the left main mix to use the left aux input.
        codec
            .modify_leftmixer(|reg| reg.with_lauxlmx(true).with_lauxmxgain(0b101))
            .await?;

        // Set the right main mix to use the right aux input.
        codec
            .modify_rightmixer(|reg| reg.with_rauxrmx(true).with_rauxmxgain(0b101))
            .await?;

        // Connect the left output mixer to the aux1 out.
        //
        // AUX1 can only connect to LMIX or RMIX.
        codec
            .modify_aux1mixer(|reg| {
                reg.with_auxiut1mt(false)
                    .with_rmixaux1(true) // mix in right mixer
                    .with_rdacaux1(true) // mix in right DAC output
            })
            .await?;

        // Connect the left output mixer to the aux2 out.
        //
        // AUX2 can only connect to LMIX but not RMIX.
        codec
            .modify_aux2mixer(|reg| {
                reg.with_auxout2mt(false)
                    .with_lmixaux2(true) // mix in left mixer
                    .with_ldacaux2(true) // mix in left dac output
            })
            .await?;

        // Spawn the task that processes the codec data.
        info!("spawning SAI loop");
        spawner.spawn(device_loop(sai_resources /*sai_receiver*/)?);

        Ok(Self { delay, codec })
    }
}

#[embassy_executor::task]
async fn device_loop(
    sai_resources: CodecSAIResources,
    // sai_receiver: SAIReceiver<'static>,
) -> ! {
    // should never return
    let err = inner_device_loop(sai_resources /* , sai_receiver*/).await;
    panic!("{:?}", err);
}

#[derive(Debug)]
enum Never {}

async fn inner_device_loop(
    mut sai_resources: CodecSAIResources,
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

    let mut buf = [0u32; HALF_DMA_BUFFER_LENGTH / 8];

    // Initialize the SAI receiver.
    // TODO: remove unwrap and use proper error type
    info!("starting SAI receiver");
    // sai_receiver.start().unwrap();

    info!("starting SAI loop");

    let mut sai_transmitter = setup_sai(&mut sai_resources, audio_tx_buffer, audio_rx_buffer);

    const AMPLITUDE: f32 = 0.10;

    let mut phase: f32 = 0.0;
    const FREQUENCY: f32 = 261.625565; // middle C
    let phase_inc = (2.0 * PI * FREQUENCY) / SAMPLE_RATE as f32;

    loop {
        for i in 0..buf.len() {
            let sine = libm::sinf(2.0 * PI * phase);
            buf[i] = ((sine * u32::MAX as f32) * AMPLITUDE) as u32;
            phase = phase + phase_inc;
            if phase >= 1.0 {
                phase = 0.0;
            }
        }

        // A write() must be called before read() to start the
        // master (transmitter) clock used by the receiver.
        match sai_transmitter.write(&buf).await {
            Ok(_) => {}
            Err(err) => {
                error!("Recreating SAI due to error: {}", err);

                // In the event of an overrun error, re-create the SAI devices to clear the DMA status.
                drop(sai_transmitter);

                sai_transmitter = setup_sai(&mut sai_resources, audio_tx_buffer, audio_rx_buffer);
            }
        }
        // sai_receiver.read(&mut buf).await.unwrap();

        // info!("sai receive: {}", buf[0]);
    }
}

fn mclk_div_from_u8(v: u8) -> MasterClockDivider {
    assert!((1..=63).contains(&v));
    MasterClockDivider::from_bits(v)
}

fn setup_sai<'d>(
    sai_resources: &'d mut CodecSAIResources,
    transmit_buffer: &'d mut [u32],
    receive_buffer: &'d mut [u32],
) -> (SAITransmitter<'d>) {
    // Derrive the required SAI clock divider based on
    // the configured sample rate for the audio engine.
    let kernel_clock = rcc::frequency::<peripherals::SAI1>().0;
    let mclk_div = mclk_div_from_u8((kernel_clock / (SAMPLE_RATE * 256)) as u8);

    info!("SAI1 master clock: {}", kernel_clock);
    info!("SAI1 master clock divider: {}", mclk_div);

    const OUTPUT_CHANNEL_COUNT: usize = 2;

    let mut tx_config = sai::Config::default();
    tx_config.slot_count = sai::word::U4(OUTPUT_CHANNEL_COUNT as u8);
    tx_config.slot_enable = 0xFFFF; // All slots
    tx_config.frame_sync_definition = sai::FrameSyncDefinition::StartOfFrame;
    tx_config.frame_sync_active_level_length = sai::word::U7(1);
    tx_config.bit_order = sai::BitOrder::MsbFirst;
    tx_config.frame_sync_offset = sai::FrameSyncOffset::OnFirstBit;
    tx_config.data_size = sai::DataSize::Data32;
    tx_config.frame_length = (OUTPUT_CHANNEL_COUNT * 32) as u8;
    tx_config.master_clock_divider = Some(mclk_div);
    // tx_config.sync_output = true; // passes sync to the second block

    // tx_config.mode = sai::Mode::Master;
    // tx_config.slot_count = sai::word::U4(2);
    // tx_config.tx_rx = sai::TxRx::Transmitter; // configure this block as a transmitter
    // tx_config.sync_output = true; // passes sync to the second block
    // tx_config.clock_strobe = sai::ClockStrobe::Falling;
    // tx_config.master_clock_divider = Some(mclk_div);
    // tx_config.stereo_mono = sai::StereoMono::Stereo;
    // tx_config.data_size = sai::DataSize::Data32;
    // tx_config.bit_order = sai::BitOrder::MsbFirst; // nau88c22 uses MSB
    // tx_config.frame_sync_polarity = sai::FrameSyncPolarity::ActiveHigh;
    // tx_config.frame_sync_offset = sai::FrameSyncOffset::OnFirstBit;
    // tx_config.frame_length = 64; // channels * data_size
    // tx_config.frame_sync_active_level_length = sai::word::U7(32);
    // tx_config.fifo_threshold = sai::FifoThreshold::Quarter;

    const RECEIVE_CHANNEL_COUNT: usize = 2;
    const SAMPLE_WIDTH_BIT: usize = 32;

    let mut rx_config = tx_config.clone();
    rx_config.mode = sai::Mode::Slave; // slaved to the transmitter block
    rx_config.tx_rx = sai::TxRx::Receiver; // configure this block as a receiver
    rx_config.slot_count = sai::word::U4(RECEIVE_CHANNEL_COUNT as u8);
    rx_config.slot_enable = 0xFFFF; // All slots
    rx_config.data_size = sai::DataSize::Data32;
    rx_config.frame_length = (RECEIVE_CHANNEL_COUNT * SAMPLE_WIDTH_BIT) as u8;
    rx_config.frame_sync_active_level_length = sai::word::U7(SAMPLE_WIDTH_BIT as u8);
    rx_config.bit_order = sai::BitOrder::MsbFirst;
    rx_config.mute_value = sai::MuteValue::LastValue;

    // rx_config.mode = sai::Mode::Slave; // slaved to the transmitter block
    // rx_config.tx_rx = sai::TxRx::Receiver; // configure this block as a receiver
    // rx_config.sync_input = sai::SyncInput::Internal; // receive sync from the first block
    // rx_config.clock_strobe = sai::ClockStrobe::Rising;
    // rx_config.sync_output = false;

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
    // let sai_rx = sai::Sai::new_synchronous(
    //     sub_block_rx,
    //     codec_sai.adcdat,
    //     codec_sai.rx_dma,
    //     rx_buf,
    //     rx_config,
    // );

    (sai_tx)
}
