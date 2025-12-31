use catalina::engine::audio::Frame;
use defmt::{error, info};
use derive_more::{Display, Error};

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
    audio::oscillator::{self, Oscillator},
    audio::sample::U24,
    core::Hertz,
};

use nau88c22_rs::registers::{
    AudioInterfaceDataFormat, GPIO1FunctionSelect, GPIO1PllDivisor, WordLength,
};

use crate::hardware::CodecSAIResources;

const OUTPUT_CHANNEL_COUNT: usize = 2; // stereo
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
    CodecClockError(CodecClockError),
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

impl From<CodecClockError> for InitError {
    fn from(value: CodecClockError) -> Self {
        Self::CodecClockError(value)
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
        codec_init(&mut codec, adjusted_mclk).await;

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
    loop {
        // Chunk the sample buffer into left and right channel frames.
        for frame in buf.chunks_mut(OUTPUT_CHANNEL_COUNT) {
            // Output is 24-bit I2S data, so format
            // the oscillator sample to U24 sizes.
            let value: U24 = osc.sample();

            for sample in frame.iter_mut() {
                // Note that we use value.inner() instead of to_sample().
                //
                // to_sample() would convert to a u32 range, but the I2S
                // sample audio format is actually 24-bit so we want to
                // cap it at 24.
                *sample = value.inner().scale_amp(0.25) as u32;
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
    tx_config.master_clock_divider = mclk_div.into();
    tx_config.nodiv = false;
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

/// https://www.nuvoton.com/export/resource-files/en-us--DS_NAU88C22_DataSheet_EN_Rev2.2.pdf
pub async fn codec_init(
    codec: &'_ mut nau88c22_rs::Nau88c22<
        asynch::i2c::I2cDevice<'_, CriticalSectionRawMutex, I2c<'static, mode::Async, i2c::Master>>,
    >,
    adjusted_mclk: u32,
) {
    // Software reset the codec to a known state.
    info!("software resetting audio codec");
    codec.reset().await.unwrap(); // register 0

    // Give time for the reset to finish.
    Timer::after_millis(200).await;

    let device_id = codec.read_deviceid().await.unwrap();
    info!("audio codec device ID: {}", device_id.id());

    // Register 1
    codec
        .modify_powermanagement1(|reg| {
            reg.with_dcbufen(false) // false for lower then 3.6v operation
                .with_aux1mxen(true) // Enable the aux 1&2 output mixers.
                .with_aux2mxen(true)
                .with_pllen(false) // Ensure the PLL is disabled.
                .with_micbiasen(true) // Enable the micbias buffer output.
                .with_abiasen(true) // Enable internal Analog Bias Buffer.
                .with_iobufen(true) // Internal Tie-off Buffer In Non-boost 1.0X Mode
                .with_refimp(0b11) // VREF Impedance Select - 80k breaks headphone output?
        })
        .await
        .unwrap();

    // Datasheet sets to wait 250ms after setting IOBUFEN, DCBUFEN,
    // REFIMP and ABIASEN to charge output capacitors.
    Timer::after_millis(250).await;

    // Register 2
    codec
        .modify_powermanagement2(|reg| {
            reg.with_rhpen(false) // Right headphone driver
                .with_lphen(false) // Left headphone driver
                .with_sleep(false) // Normal mode
                .with_rbsten(true) // Right channel ADC input enable
                .with_lbsten(true) // Left channel ADC input enable
                .with_rpgaen(true) // Right channel input PGA enable
                .with_lpgaen(true) // Left channel input PGA enable
                .with_radcen(true) // Right channel ADC enable
                .with_ladcen(true) // Left channel ADC enable
        })
        .await
        .unwrap();

    // Input routing
    {
        // Register 14.
        codec
            .modify_adccontrol(|reg| {
                reg.with_adcos(true) //128x oversampling
                    .with_hpf(0)
                    .with_hpfen(false) // disable high pass filter
            })
            .await
            .unwrap();

        // Register 44.
        codec
            .modify_inputcontrol(|reg| {
                reg.with_rmicnrpga(false)
                    .with_rmicnrpga(false) // disable default right differential mix in to PGA
                    .with_lmicnlpga(false)
                    .with_lmicnlpga(false) // disable default right differential mix in to PGA
            })
            .await
            .unwrap();

        // Mute left and right PGA (registers 45, 46)
        codec
            .modify_leftinputpgagain(|reg| reg.with_lpgamt(true))
            .await
            .unwrap();
        codec
            .modify_rightinputpgagain(|reg| reg.with_rpgamt(true))
            .await
            .unwrap();

        // Enable the left aux in as the left ADC source.
        codec
            .modify_leftadcboost(|reg| {
                reg.with_lpgabst(false) // Disable the left PGA input
                    .with_lauxbstgain(0b101) // Enable the left aux in at 0db
                    .with_lpgabstgain(0) // Left line-in to boost stage
            })
            .await
            .unwrap();

        // Enable the right aux in as the right ADC source.
        codec
            .modify_rightadcboost(|reg| {
                reg.with_rpgabst(false) // disable the PGA input
                    .with_rauxbstgain(0b101) // Enable the right aux in at 0db
                    .with_rpgabstgain(0) // Right line-in boost stage input
            })
            .await
            .unwrap();

        // Ensure ADC input volumes are at 0db.
        codec
            .modify_leftadcvolume(|reg| reg.with_ladcgain(0b11111111))
            .await
            .unwrap();
        codec
            .modify_rightadcvolume(|reg| reg.with_radcgain(0b11111111))
            .await
            .unwrap();
    }

    // Output routing and DAC setup
    {
        // Register 3 - output power
        codec
            .modify_powermanagement3(|reg| {
                reg.with_auxout1en(true) // Aux out 1 (pin#21) enable
                    .with_auxout2en(true) // Aux out 2 (pin#22) enable
                    .with_lspken(false) // Left speaker output driver enable
                    .with_rspken(false) // Right speaker output driver enable
                    .with_rmixen(true) // Right output main mixer enable
                    .with_lmixen(true) // Left output main mixer enable
                    .with_rdacen(true) // Right DAC enable
                    .with_ldacen(true) // Left DAC enable
            })
            .await
            .unwrap();

        // Register 10
        codec
            .modify_daccontrol(|reg| {
                reg.with_dacos(true) // 128x oversampling
            })
            .await
            .unwrap();

        // Register 50
        // Set the left main mix to use the left aux input.
        codec
            .modify_leftmixer(|reg| {
                reg.with_ldaclmx(true) // mix in the left dac output
            })
            .await
            .unwrap();

        // Register 51
        // Set the right main mix to use the right aux input.
        codec
            .modify_rightmixer(|reg| {
                reg.with_rdacrmx(true) // mix in the right dac output
            })
            .await
            .unwrap();

        // Set volumes to initial 0dB
        codec
            .modify_leftdacvolume(|reg| reg.with_ldacvu(false).with_ldacgain(0b11111111))
            .await
            .unwrap();
        codec
            .modify_rightdacvolume(|reg| reg.with_rdacvu(true).with_rdacgain(0b11111111))
            .await
            .unwrap();

        // TODO: remove after testing clock!!
        // Internal ADC -> DAC Loopback
        // Test routing ADC output to DAC input
        // codec
        //     .modify_companding(|reg| reg.with_addap(true))
        //     .await
        //     .unwrap();

        // Connect the left output mixer to the aux1 out.
        //
        // AUX1 can only connect to LMIX or RMIX.
        codec
            .modify_aux1mixer(|reg| {
                // reg.with_rdacaux1(true) // mix in right DAC output
                // reg.with_rdacaux1(true).with_radcaux1(true)
                reg.with_rdacaux1(true)
            })
            .await
            .unwrap();

        // Connect the left output mixer to the aux2 out.
        //
        // AUX2 can only connect to LMIX but not RMIX.
        codec
            .modify_aux2mixer(|reg| {
                // reg.with_ldacaux2(true) // mix in left dac output
                // reg.with_ldacaux2(true).with_ldacaux(true)
                reg.with_ldacaux2(true)
            })
            .await
            .unwrap();
    }

    // Audio format and clock
    {
        // Register 4
        //
        // Configure the audio format
        codec
            .modify_audiointerface(|reg| {
                reg.with_wlen(WordLength::Word24Bit) // 24-bit words
                    .with_aifmt(AudioInterfaceDataFormat::StandardI2S) // standard i2s
            })
            .await
            .unwrap();

        // Test by outputting the clock on GPIO1
        codec
            .modify_gpio(|reg| {
                reg.with_gpio1sel(GPIO1FunctionSelect::DividedPLLClock)
                    .with_gpio1pll(GPIO1PllDivisor::Divide1)
            })
            .await
            .unwrap();

        // Config with PLL
        {
            // Calculate the SAI master clock divisor and derrived codec master clock divisor and PLL.
            let (mclk_div, pllmclk, plln, pllk1, pllk2, pllk3) =
                nau88c22_calc_pll(adjusted_mclk as f32, SAMPLE_RATE as f32).unwrap();

            info!(
                "codec mclk_div={} pllmclk={} plln={} pllk1={} pllk2={} pllk3={}",
                mclk_div,
                pllmclk > 0,
                plln,
                pllk1,
                pllk2,
                pllk3
            );

            // Register 6
            //
            // Set MCLK (pin#11) as a master clock input, configure
            // the clock pins (BCLK and FS) in slave mode, and set
            // the desired MCLK divider.
            codec
                .modify_clockcontrol1(|reg| {
                    reg.with_clkm(true) // Internal PLL is used
                        // TODO: use mclk_div
                        .with_mclksel(nau88c22_rs::registers::MasterClockSourceScaling::Divide1) // divide PLL before MCLK
                        .with_clkioen(false) // fs and bclk are inputs
                })
                .await
                .unwrap();

            // Configure the PLL.
            codec
                .modify_plln(|reg| reg.with_pllmclk(pllmclk > 0).with_plln(plln))
                .await
                .unwrap();
            codec
                .modify_pllk1(|reg| reg.with_pllk(pllk1))
                .await
                .unwrap();
            codec
                .modify_pllk2(|reg| reg.with_pllk(pllk2))
                .await
                .unwrap();
            codec
                .modify_pllk3(|reg| reg.with_pllk(pllk3))
                .await
                .unwrap();

            // Enable the PLL.
            codec
                .modify_powermanagement1(|reg| reg.with_pllen(true))
                .await
                .unwrap();

            // Wait for the PLL to stabalize.
            Timer::after_millis(255).await;
        }
    }
}

const NAU_MCLKSEL: [f32; 8] = [1.0, 1.5, 2.0, 3.0, 4.0, 6.0, 8.0, 12.0];

#[derive(Debug, Display, Error)]
pub enum CodecClockError {
    SampleRateTooLow,
    SampleRateTooHigh,
    MCLKTooHigh,
    MCLKTooLow,
    UnresolvablePLL,
}

/// Caluclates a suitable set of clock control parameters for the NAU88C22
/// given the provided master block and sample rate inputs.
///
///  "IMCLK should not exceed 12.288MHz under any condition without enabling PLL49MOUT bit R72[2].
///   IMCLK is output from the Master Clock Prescaler. The prescaler reduces by an integer division
///   factor the input frequency input clock. The source of this input frequency clock is either
///   the external MCLK pin, or the output from the internal PLL Block."
///
///  "the optimum PLL oscillator frequency is in the range between 90MHz and
///   100MHz, and thus, it is best to keep f2 within this range."
///
/// Returns (MCLK_DIV, pllmclk, PLLN_N, pll_k1, pll_k2, pll_k3)
fn nau88c22_calc_pll(
    nau_mckl_in: f32,
    sample_rate_hz: f32,
) -> Result<(u8, u8, u8, u8, u16, u16), CodecClockError> {
    // 11.999kHz
    if sample_rate_hz < 11999.0 {
        error!("sample rate too low, {} < 11999.0", sample_rate_hz);
        return Err(CodecClockError::SampleRateTooLow);
    }

    // 48kHz
    if sample_rate_hz > 48001.0 {
        error!("sample rate too high, {} > 48001.0", sample_rate_hz);
        return Err(CodecClockError::SampleRateTooHigh);
    }

    // First, derrive the desired IMCLK frequency
    // which can be calculated as 256 * sample_Rate.
    //
    // IMCLK = desired Master Clock = (256)*(desired codec sample rate)
    //
    // 12_288_000hz for 48k
    let frq_imclk = 256.0 * sample_rate_hz;
    info!("NAC88C22 desired IMCLK rate (256*fs): {}hz", frq_imclk);

    // Check that the MCLK being provided to the codec is within the
    // workable PLL reference frequency range of 8MHz to 33MHz.
    if nau_mckl_in > 33_000_000.0 {
        error!("SAI mclk too high, {} > 33000000.0", nau_mckl_in);
        return Err(CodecClockError::MCLKTooHigh);
    }

    if nau_mckl_in < 9_000_000.0 {
        error!("SAI mclk too low, {} < 9000000.0", nau_mckl_in);
        return Err(CodecClockError::MCLKTooLow);
    }

    let mut mclkseldiv: u8 = 0;

    // f2 = (4)*(P)(IMCLK) or (2)*(P)(IMCLK) when PLL49MOUT bit R72[2] = 1
    // > where P is the Master Clock Prescale integer value
    //
    // optimal f2: 90MHz< f2 <100MHz
    let mut pll_f2: f32 = 0.0; // PLL Oscillator f2

    // Try to find a divisor that hits the optimal f2 frequency.
    for i in 0..8 {
        pll_f2 = 4.0 * NAU_MCLKSEL[i] * frq_imclk;
        if (pll_f2 >= 80000000.0) && (pll_f2 < 110000000.0) {
            mclkseldiv = i as u8;
            break;
        };
    }

    if mclkseldiv == 0 {
        for i in 0..8 {
            pll_f2 = 2.0 * NAU_MCLKSEL[i] * frq_imclk;
            if (pll_f2 >= 80000000.0) && (pll_f2 < 110000000.0) {
                mclkseldiv = i as u8;
                break;
            };
        }
    };

    info!(
        "NAU88C22 PLL oscillator f2={}hz (mclkseldiv={}={})",
        pll_f2, mclkseldiv, NAU_MCLKSEL[mclkseldiv as usize]
    );

    // There are two dividers between the PLL and the output pin.
    info!("NAU88C22 expected GPIO1 PLL output freq {}hz", pll_f2 / 4.0);

    info!(
        "NAC88C22 calculated achieved IMCLK rate: {}hz!",
        (pll_f2 / 4.0) / NAU_MCLKSEL[mclkseldiv as usize]
    );

    // f1 = (MCLK)/(D)
    //  D = PLL Prescale factor of 1, or 2
    //  MCLK = is the frequency at the MCLK pin
    let pll_f1: f32 = nau_mckl_in; // input MCLK frequency
    info!("NAC88C22 input mclk frequency f1={}hz", pll_f1);

    let mut nau_pres_mckl: u8 = 3;
    let mut nau_r1_ready: u8 = 0;

    let mut nau_int_pll: u32 = 0;

    // Fractional frequency multiplication factor for the PLL.
    //
    // R = f2/f1 = xy.abcdefgh
    // Fractional multipler R=f2/f1
    let mut nau_pll_r: f32 = pll_f2 / pll_f1;

    info!(
        "NAC88C22 fractional multiplier R=(f2/f1)=({}/{})={} (nau_pres_mckl={})",
        pll_f2, pll_f1, nau_pll_r, nau_pres_mckl
    );

    if (nau_pll_r > 6.0) && (nau_pll_r < 13.0) {
        nau_pres_mckl = 1;
    } else {
        nau_pll_r = pll_f2 / (pll_f1 * 2.0);
        nau_pres_mckl = 2;

        info!(
            "NAC88C22 scaled! fractional multiplier R=(f2/f1)={} (nau_pres_mckl={})",
            nau_pll_r, nau_pres_mckl
        );
    };

    // Truncated integer portion of the R value, and limited
    // to decimal value 6, 7, 8, 9, 10, 11, or 12.
    //
    // N = xy
    let mut integer_portion_n: u8 = 0;

    // Check that the fractional multiplier is within the supported PLL clock bounds,
    // and then calculate the derrived integer (H) and fractional (K) portions.
    if (nau_pll_r > 6.0) && (nau_pll_r < 13.0) {
        // Get the intergel part by flooring the fractional multiplier.
        integer_portion_n = libm::floorf(nau_pll_r) as u8; // N = xy

        // TODO check that N is 6, 7, 8, 9, 10, 11, or 12

        // Derrive the fractional portion K from the floored intger portion.
        let mut nau_pll: f32 = 0.0;
        nau_pll = nau_pll_r - (integer_portion_n as f32); // (0.abcdefgh)
        // Multiply by 2^24 to derrive the 24-bit binary fractional value.
        nau_pll *= 16777216.0; // K = (2^24)*(0.abcdefgh)
        // Cast to u32 to round to nearest whole integer.
        nau_int_pll = nau_pll as u32;

        nau_r1_ready = 1;
    };

    // Convert the fractional binary value into it's bit
    // segments to write to the corrosponding registers.
    let pll_k1: u8 = ((nau_int_pll >> 18) & 0x3F) as u8; // Highest order 6-bits of 24-bit fraction
    let pll_k2: u16 = ((nau_int_pll >> 9) & 0x1FF) as u16; // Middle 9-bits of 24-bit fraction
    let pll_k3: u16 = ((nau_int_pll >> 0) & 0x1FF) as u16; // Lowest order 9-bits of 24-bit fraction

    if nau_r1_ready != 1 {
        return Err(CodecClockError::UnresolvablePLL);
    }

    info!(
        "NAC88C22 integer portion N (Hex): 0x{:x}",
        integer_portion_n
    );
    info!("NAC88C22 fractional portion K (Hex): 0x{:x}", nau_int_pll);

    info!("Registers:");
    info!("R6[7:5](MCLKSEL)={}", mclkseldiv);
    info!(
        "R36[3:0](PLLN)={} R36[4](PLLMCLK)={}",
        integer_portion_n,
        (nau_pres_mckl - 1)
    );
    info!("R37(PLLK[23:18])={:06b}", pll_k1);
    info!("R38(PLLK[17:9])={:09b}", pll_k2);
    info!("R38(PLLK[8:0])={:09b}", pll_k3);

    Ok((
        mclkseldiv,
        (nau_pres_mckl - 1),
        integer_portion_n,
        pll_k1,
        pll_k2,
        pll_k3,
    ))
}
