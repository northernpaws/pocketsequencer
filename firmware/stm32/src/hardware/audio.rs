use defmt::{error, info};
use derive_more::{Display, Error};
use embassy_embedded_hal::shared_bus::I2cDeviceError;
use embassy_embedded_hal::shared_bus::asynch::{self};
use embassy_executor::{InterruptExecutor, SpawnError, Spawner};
use embassy_stm32::i2c::{self, I2c};
use embassy_stm32::interrupt::{InterruptExt, Priority};
use embassy_stm32::sai::MasterClockDivider;
use embassy_stm32::{interrupt, mode, peripherals, rcc, sai};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::Timer;
use grounded::uninit::GroundedArrayCell;

use catalina::engine::{
    audio::oscillator::{self, Oscillator},
    core::Hertz,
};

use crate::hardware::CodecSAIResources;

const OUTPUT_CHANNEL_COUNT: usize = 2; // stereo
pub const BLOCK_LENGTH: usize = 32; // samples
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
    AUDIO_EXECUTOR.on_interrupt()
}

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
                    .with_clkioen(false) // clock is in slave mode
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

        // Calculate the SAI master clock divisor and derrived codec master clock divisor and PLL.
        let kernel_clock = rcc::frequency::<peripherals::SAI1>().0;
        let mclk_div: MasterClockDivider =
            mclk_div_from_u8((kernel_clock / (SAMPLE_RATE * 256)) as u8);
        let mclk_div_u8: u8 = mclk_div.into();
        let adjusted_mclk = kernel_clock / mclk_div_u8 as u32;
        info!("SAI1 master clock base: {}hz", kernel_clock);
        info!("SAI1 master clock divisor: {}", mclk_div);
        info!("SAI1 master clock adjusted: {}hz", adjusted_mclk);

        let (mclk_div, pllmclk, plln, pllk1, pllk2, pllk3) =
            nau88c22_calc_pll(adjusted_mclk as f32, SAMPLE_RATE as f32)?;

        info!("Ideal SAI clock for sample rate: {}", SAMPLE_RATE * 256);
        info!(
            "SAI clock skew: {}%",
            (adjusted_mclk / (SAMPLE_RATE * 256)) as f32 * 100.0
        );

        info!(
            "codec mclk_div={} pllmclk={} plln={} pllk1={} pllk2={} pllk3={}",
            mclk_div,
            pllmclk > 0,
            plln,
            pllk1,
            pllk2,
            pllk3
        );

        // Configure the PLL.
        codec
            .modify_clockcontrol1(|reg| reg.with_mclksel(mclk_div))
            .await?;
        codec
            .modify_plln(|reg| reg.with_pllmclk(pllmclk > 0).with_plln(plln))
            .await?;
        codec.modify_pllk1(|reg| reg.with_pllk(pllk1)).await?;
        codec.modify_pllk2(|reg| reg.with_pllk(pllk2)).await?;
        codec.modify_pllk3(|reg| reg.with_pllk(pllk3)).await?;

        // Wait for the PLL to stabalize.
        Timer::after_millis(255).await;

        // Enable the PLL.
        codec
            .modify_powermanagement1(|reg| reg.with_pllen(true))
            .await?;

        // Configure the audio format
        codec
            .modify_audiointerface(|reg| {
                reg.with_wlen(0b11) // 32-bit words
                    .with_aifmt(0b10) // standard i2s
                    .with_mono(false) // stereo mode
            })
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
            .modify_leftmixer(|reg| {
                reg //.with_lauxlmx(true) // mix in the left audio input
                    //.with_lauxmxgain(0b101)
                    .with_ldaclmx(true) // mix in the left dac output
            })
            .await?;

        // Set the right main mix to use the right aux input.
        codec
            .modify_rightmixer(|reg| {
                reg //.with_rauxrmx(true) // mix in the right aux input
                    //.with_rauxmxgain(0b101)
                    .with_rdacrmx(true) // mix in the right dac output
            })
            .await?;

        // Connect the left output mixer to the aux1 out.
        //
        // AUX1 can only connect to LMIX or RMIX.
        codec
            .modify_aux1mixer(|reg| {
                reg.with_auxiut1mt(false)
                    //.with_rmixaux1(true) // mix in right mixer
                    .with_rdacaux1(true) // mix in right DAC output
            })
            .await?;

        // Connect the left output mixer to the aux2 out.
        //
        // AUX2 can only connect to LMIX but not RMIX.
        codec
            .modify_aux2mixer(|reg| {
                reg.with_auxout2mt(false)
                    //.with_lmixaux2(true) // mix in left mixer
                    .with_ldacaux2(true) // mix in left dac output
            })
            .await?;

        // Spawn the task that processes the codec data.
        info!("spawning SAI loop");

        // Use an interrupt executor to run the audio task at a higher priority.
        interrupt::SAI1.set_priority(Priority::P6);
        let spawner = AUDIO_EXECUTOR.start(interrupt::SAI1);
        spawner.spawn(device_loop(
            mclk_div.into(),
            sai_resources, /*sai_receiver*/
        )?);

        Ok(Self { delay, codec })
    }
}

#[embassy_executor::task]
async fn device_loop(
    mclk_div: MasterClockDivider,
    sai_resources: CodecSAIResources,
    // sai_receiver: SAIReceiver<'static>,
) -> ! {
    // should never return
    let err = inner_device_loop(mclk_div, sai_resources /* , sai_receiver*/).await;
    panic!("{:?}", err);
}

#[derive(Debug)]
enum Never {}

async fn inner_device_loop(
    mclk_div: MasterClockDivider,
    mut sai_resources: CodecSAIResources,
    // mut sai_receiver: SAIReceiver<'static>,
) -> Result<(), Never> {
    info!("initializing SAI buffers...");

    let audio_tx_buffer: &mut [u32] = unsafe {
        AUDIO_TX_BUFFER.initialize_all_copied(0);
        let (ptr, len) = AUDIO_TX_BUFFER.get_ptr_len();
        core::slice::from_raw_parts_mut(ptr, len)
    };

    let tx_size = audio_tx_buffer.len();

    let audio_rx_buffer: &mut [u32] = unsafe {
        AUDIO_RX_BUFFER.initialize_all_copied(0);
        let (ptr, len) = AUDIO_RX_BUFFER.get_ptr_len();
        core::slice::from_raw_parts_mut(ptr, len)
    };

    // Initialize the SAI receiver.
    // TODO: remove unwrap and use proper error type
    info!("starting SAI receiver");
    // sai_receiver.start().unwrap();

    info!("starting SAI loop");

    let mut sai_transmitter = setup_sai(
        &mut sai_resources,
        audio_tx_buffer,
        audio_rx_buffer,
        mclk_div,
    );

    let mut osc = oscillator::RuntimeOscillator::new(
        oscillator::OscillatorType::Sine,
        SAMPLE_RATE as usize,
        Hertz::from_hertz(261.63), // middle C
    );

    // Set SAI interrupt to high-priority.
    interrupt::SAI1.set_priority(Priority::P6);

    let mut buf = [0u32; HALF_DMA_BUFFER_LENGTH];
    loop {
        for frame in buf.chunks_mut(OUTPUT_CHANNEL_COUNT) {
            let value: u32 = osc.sample();
            for sample in frame.iter_mut() {
                *sample = value;
            }
        }

        // A write() must be called before read() to start the
        // master (transmitter) clock used by the receiver.
        match sai_transmitter.write(&buf).await {
            Ok(_) => {
                // info!("Write ok!");
            }
            Err(err) => {
                error!("Recreating SAI due to error: {}", err);

                info!("SAI transmit buffer size: {}", tx_size);
                info!("SAI block transfer size: {}", buf.len());

                // In the event of an overrun error, re-create the SAI devices to clear the DMA status.
                drop(sai_transmitter);

                sai_transmitter = setup_sai(
                    &mut sai_resources,
                    audio_tx_buffer,
                    audio_rx_buffer,
                    mclk_div,
                );
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
    mclk_div: MasterClockDivider,
) -> (SAITransmitter<'d>) {
    // Derrive the required SAI clock divider based on
    // the configured sample rate for the audio engine.
    // let kernel_clock = rcc::frequency::<peripherals::SAI1>().0;
    // let mclk_div = mclk_div_from_u8((kernel_clock / (SAMPLE_RATE * 256)) as u8);

    // info!("SAI1 master clock: {}", kernel_clock);
    // info!("SAI1 master clock divider: {}", mclk_div);

    let mut tx_config = sai::Config::default();
    tx_config.protocol = sai::Protocol::Free; // mode for I2S, PCM, TDM, etc.
    // The number of slots in the audio frame.
    tx_config.slot_count = sai::word::U4(OUTPUT_CHANNEL_COUNT as u8);
    // Enables the slots we're going to use.
    tx_config.slot_enable = 0b11; // First 2 slots.
    tx_config.frame_sync_definition = sai::FrameSyncDefinition::StartOfFrame;
    tx_config.frame_sync_active_level_length = sai::word::U7(32); // 1-bit cycle for i2s?
    tx_config.bit_order = sai::BitOrder::MsbFirst; // nac88c22 runs in MSB
    tx_config.frame_sync_offset = sai::FrameSyncOffset::OnFirstBit;
    tx_config.data_size = sai::DataSize::Data32;
    // The audio frame length expressed in number of SCK clock cycles.
    tx_config.frame_length = (OUTPUT_CHANNEL_COUNT * 32) as u8;
    tx_config.master_clock_divider = Some(mclk_div);
    tx_config.clock_strobe = sai::ClockStrobe::Rising; // nac88c22 uses rising edge latching on bclk
    tx_config.fifo_threshold = sai::FifoThreshold::Quarter;
    // tx_config.sync_output = true; // passes sync to the second block

    // const RECEIVE_CHANNEL_COUNT: usize = 2;
    // const SAMPLE_WIDTH_BIT: usize = 32;

    // let mut rx_config = tx_config.clone();
    // rx_config.mode = sai::Mode::Slave; // slaved to the transmitter block
    // rx_config.tx_rx = sai::TxRx::Receiver; // configure this block as a receiver
    // rx_config.slot_count = sai::word::U4(RECEIVE_CHANNEL_COUNT as u8);
    // rx_config.slot_enable = 0xFFFF; // All slots
    // rx_config.data_size = sai::DataSize::Data32;
    // rx_config.frame_length = (RECEIVE_CHANNEL_COUNT * SAMPLE_WIDTH_BIT) as u8;
    // rx_config.frame_sync_active_level_length = sai::word::U7(SAMPLE_WIDTH_BIT as u8);
    // rx_config.bit_order = sai::BitOrder::MsbFirst;
    // rx_config.mute_value = sai::MuteValue::LastValue;

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

const nau_mclksel: [f32; 8] = [1.0, 1.5, 2.0, 3.0, 4.0, 6.0, 8.0, 12.0];

#[derive(Debug, Display, Error)]
pub enum CodecClockError {
    SampleRateTooLow,
    SampleRateTooHigh,
    MCLKTooHigh,
    MCLKTooLow,
    UnresolvablePLL,
}

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
    let frq_imclk = 256.0 * sample_rate_hz;
    // let nau_mckl_in = master_clock_fq / pck_prescaller;

    // Check that the MCLK being provided to the codec is within a workable range.
    if nau_mckl_in > 33_000_000.0 {
        error!("SAI mclk too high, {} > 33000000.0", nau_mckl_in);
        return Err(CodecClockError::MCLKTooHigh);
    }
    if nau_mckl_in < 9_000_000.0 {
        error!("SAI mclk too low, {} < 9000000.0", nau_mckl_in);
        return Err(CodecClockError::MCLKTooLow);
    }

    let mut mclkseldiv: u8 = 0;

    let mut nau_pll_f2: f32 = 0.0;
    let mut nau_pll_f1: f32 = 0.0;

    for i in 0..8 {
        nau_pll_f2 = 4.0 * nau_mclksel[i] * frq_imclk;
        if (nau_pll_f2 >= 80000000.0) && (nau_pll_f2 < 110000000.0) {
            mclkseldiv = i as u8;
            break;
        };
    }

    if mclkseldiv == 0 {
        for i in 0..8 {
            nau_pll_f2 = 2.0 * nau_mclksel[i] * frq_imclk;
            if (nau_pll_f2 >= 80000000.0) && (nau_pll_f2 < 110000000.0) {
                mclkseldiv = i as u8;
                break;
            };
        }
    };

    let mut nau_pres_mckl: u8 = 3;
    let mut nau_r1_ready: u8 = 0;
    let mut nau_nxy: u8 = 0;
    let mut nau_pll: f32 = 0.0;
    let mut nau_int_pll: u32 = 0;

    nau_pll_f1 = nau_mckl_in;

    let mut nau_pll_r: f32 = nau_pll_f2 / nau_pll_f1;

    if (nau_pll_r > 6.0) && (nau_pll_r < 13.0) {
        nau_pres_mckl = 1;
    } else {
        nau_pll_r = nau_pll_f2 / (nau_pll_f1 * 2.0);
        nau_pres_mckl = 2;
    };

    if (nau_pll_r > 6.0) && (nau_pll_r < 13.0) {
        nau_nxy = libm::floorf(nau_pll_r) as u8;
        nau_pll = nau_pll_r - (nau_nxy as f32);
        nau_pll *= 16777216.0;
        nau_int_pll = nau_pll as u32;
        nau_r1_ready = 1;
    };

    let pll_k1: u8 = ((nau_int_pll >> 18) & 0x3F) as u8;
    let pll_k2: u16 = ((nau_int_pll >> 9) & 0x1FF) as u16;
    let pll_k3: u16 = ((nau_int_pll >> 0) & 0x1FF) as u16;

    if nau_r1_ready != 1 {
        return Err(CodecClockError::UnresolvablePLL);
    }

    // if nau_r1_ready == 1 {
    //     nau8812_clr_bit(POWER_MANAGMENT_1, (1 << PLLEN));
    //     nau8812_write(
    //         CLOCK_CONTROL_1,
    //         (1 << CLKM) | (mclkseldiv << MCLKSEL) | (0xD),
    //     );
    //     nau8812_write(PLLN_N, ((nau_pres_mckl - 1) << PLLMCLK) | (nau_nxy));
    //     nau8812_write(PLL_K_1, pll_k1);
    //     nau8812_write(PLL_K_2, pll_k2);
    //     nau8812_write(PLL_K_3, pll_k3);
    // } else {
    //     return 5;
    // }

    // nau8812_set_bit(POWER_MANAGMENT_1, (1 << PLLEN));

    Ok((
        mclkseldiv,
        (nau_pres_mckl - 1),
        nau_nxy,
        pll_k1,
        pll_k2,
        pll_k3,
    ))
}
