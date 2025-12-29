#![no_std]
#![no_main]

#[cfg(feature = "alloc")]
extern crate alloc;

use catalina::engine::audio::oscillator::Oscillator;
use catalina::engine::audio::sample::U24;
use catalina::engine::audio::{FromSample, Sample, oscillator};
use catalina::engine::core::Hertz;
use embassy_stm32::rcc::mux::Saisel;
use embassy_stm32::{mode, rcc::*};
// NOTE: We try to rely on heap allocations as little
// as possible for core elements of the system.
//
// Where possible, core systems should use heapless static
// allocation or allocation pools so their size is known
// and we avoid running into OOM problems.
#[cfg(feature = "alloc")]
use embedded_alloc::LlffHeap as Heap;
#[cfg(feature = "alloc")]
#[global_allocator]
static HEAP: Heap = Heap::empty();

// https://medium.com/@carlmkadie/how-rust-embassy-shine-on-embedded-devices-part-1-9f4911c92007
// https://medium.com/@carlmkadie/aad1adfccf72

use defmt::*;

use embassy_executor::Spawner;

use embassy_embedded_hal::shared_bus::asynch::{self};
use embassy_stm32::sai::MasterClockDivider;
use embassy_stm32::{Config, peripherals, rcc, sai};
use embassy_stm32::{
    i2c::{self, I2c},
    mode::Async,
    rcc::clocks,
    usart::{UartRx, UartTx},
};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::Timer;
use grounded::uninit::GroundedArrayCell;

use defmt::{error, info};
use derive_more::{Display, Error};
use static_cell::StaticCell;

use {
    // defmt_rtt as _,
    panic_probe as _,
};

use firmware::{
    hardware::{self, preamble::*},
    split_resources,
};

// Audio I2C bus.
//
// This communicates with the NAU88C22YG Audio Codec, FM SI4703-C19-GMR RX / SI4710-B30-GMR TX.
static I2C1_BUS: StaticCell<Mutex<CriticalSectionRawMutex, I2c<'static, Async, i2c::Master>>> =
    StaticCell::new();

const OUTPUT_CHANNEL_COUNT: usize = 2; // stereo
pub const BLOCK_LENGTH: usize = 32; // samples
pub const HALF_DMA_BUFFER_LENGTH: usize = (BLOCK_LENGTH) * OUTPUT_CHANNEL_COUNT; //  2 channels
pub const DMA_BUFFER_LENGTH: usize = HALF_DMA_BUFFER_LENGTH * 2; //  2 half-blocks

pub const SAMPLE_RATE: u32 = 48000;

// #[unsafe(link_section = ".sram1_bss")]
static AUDIO_TX_BUFFER: GroundedArrayCell<u32, { DMA_BUFFER_LENGTH }> = GroundedArrayCell::uninit();
// #[unsafe(link_section = ".sram1_bss")]
static AUDIO_RX_BUFFER: GroundedArrayCell<u32, { DMA_BUFFER_LENGTH }> = GroundedArrayCell::uninit();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("program start!");
    // If it returns, something went wrong.
    if let Err(err) = inner_main(spawner).await {
        defmt::panic!("{}", err);
    }
}

static DEBUG_SERIAL_TX: StaticCell<UartTx<'static, Async>> = StaticCell::new();
static DEBUG_SERIAL_RX: StaticCell<UartRx<'static, Async>> = StaticCell::new();

// Inner-main allows us us to returns errors via Result instead of relying on panics.
#[expect(
    clippy::future_not_send,
    reason = "Safe in single-threaded, bare-metal embedded context"
)]
#[expect(clippy::items_after_statements, reason = "Keeps related code together")]
async fn inner_main(spawner: Spawner) -> Result<(), ()> {
    // TODO: add error type
    info!("initializing clocks and PLL");
    let p = hardware::init();

    info!("getting handles to hardware peripherals");
    let mut r = split_resources!(p);

    // Get a handle to the STM6601 power manager and enable PS_HOLD as early as
    // possible to make sure the STM6601 won't power the system regulator off.
    //
    // This MUST be configured as early as possible to ensure that the
    // PS_HOLD pin is held high by the MCU to keep the power enabled.
    //
    // NOTE: PLL errors before this point will cause the power controller to shut
    // off the paniced MCU after 5 seconds, leaving little time for debugging.
    info!("Configuring stm6601...");
    let mut stm6601 = hardware::get_stm6601(r.power);
    stm6601.power_enable().unwrap(); // TODO: change to result error

    info!("Power good confirmed!");

    // Initialize the debug UART connection to the BMP
    // header and configure it for use with defmt.
    //
    // Baud: 115200
    // DataBits8
    // ParityNone
    // Stop1
    let mut debug_uart = hardware::get_uart_debug(r.uart_debug);
    debug_uart.blocking_write(b"debug serial ready").unwrap();
    let (debug_tx_raw, debug_rx_raw) = debug_uart.split();
    let debug_tx = DEBUG_SERIAL_TX.init(debug_tx_raw);
    let _debug_rx = DEBUG_SERIAL_RX.init(debug_rx_raw);
    defmt_serial::defmt_serial(debug_tx);

    let clocks = clocks(&p.RCC);
    println!("System clock speed: {}", clocks.sys);
    println!("APB1  clock speed: {}", clocks.pclk1);
    println!("APB1 Timer clock speed: {}", clocks.pclk1_tim);
    println!("APB2  clock speed: {}", clocks.pclk2);
    println!("APB2 Timer clock speed: {}", clocks.pclk2_tim);

    // Calculate the SAI master clock divisor and derrived codec master clock divisor and PLL.
    let kernel_clock = rcc::frequency::<peripherals::SAI1>().0;
    let mclk_div: MasterClockDivider = mclk_div_from_u8((kernel_clock / (SAMPLE_RATE * 256)) as u8);
    let mclk_div_u8: u8 = mclk_div.into();
    let adjusted_mclk = kernel_clock / mclk_div_u8 as u32;
    info!("SAI1 master clock base: {}hz", kernel_clock);
    info!("SAI1 master clock divisor: {}", mclk_div);
    info!("SAI1 master clock adjusted: {}hz", adjusted_mclk);

    info!("Ideal SAI clock for sample rate: {}", SAMPLE_RATE * 256);
    info!(
        "SAI clock skew: {}%",
        (adjusted_mclk / (SAMPLE_RATE * 256)) as f32 * 100.0
    );

    // Initialize the bus for the I2C1 peripheral.
    //
    // This communicates with the NAU88C22YG Audio Codec,
    // FM SI4703-C19-GMR RX / SI4710-B30-GMR TX.
    info!("initializing audio i2c1 bus");
    let i2c1_bus = I2C1_BUS.init(Mutex::new(hardware::get_i2c1(r.i2c1)));

    info!("initializing audio device");
    let device = asynch::i2c::I2cDevice::new(i2c1_bus);

    let mut codec = nau88c22_rs::Nau88c22::new(device);

    // Software reset the codec to a known state.
    info!("software resetting audio codec");
    codec.reset().await.unwrap();

    // Give time for the reset to finish.
    Timer::after_millis(200).await;

    let device_id = codec.read_deviceid().await.unwrap();
    info!("audio codec device ID: {}", device_id.id());

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

    let rx_size = audio_rx_buffer.len();

    // Initialize the SAI receiver.
    // TODO: remove unwrap and use proper error type

    // The SAI config used for the A block configured in transmit mode.
    let mut tx_config = sai::Config::default();
    tx_config.slot_count = sai::word::U4(OUTPUT_CHANNEL_COUNT as u8); // The number of slots in the audio frame.
    tx_config.slot_enable = 0b11; // First 2 slots.
    tx_config.frame_sync_definition = sai::FrameSyncDefinition::StartOfFrame;
    tx_config.frame_sync_active_level_length = sai::word::U7(32); // 1-bit cycle for i2s?
    tx_config.bit_order = sai::BitOrder::MsbFirst; // nac88c22 runs in MSB
    tx_config.frame_sync_offset = sai::FrameSyncOffset::OnFirstBit;
    tx_config.data_size = sai::DataSize::Data24;
    tx_config.frame_length = 64; // The audio frame length expressed in number of SCK clock cycles. // (channels * 32) for 24 and 32 bit
    tx_config.master_clock_divider = mclk_div.into();
    tx_config.clock_strobe = sai::ClockStrobe::Rising; // nac88c22 uses rising edge latching on bclk
    tx_config.fifo_threshold = sai::FifoThreshold::Quarter;
    tx_config.sync_output = true; // passes sync to the second block

    // The SAI config used for the B block configured in receive mode.
    let mut rx_config = tx_config.clone();
    rx_config.mode = sai::Mode::Slave; // slaved to the transmitter block
    rx_config.tx_rx = sai::TxRx::Receiver; // configure this block as a receiver
    rx_config.sync_input = sai::SyncInput::Internal; // passes sync to the second block
    rx_config.sync_output = false; // passes sync to the second block
    rx_config.clock_strobe = sai::ClockStrobe::Rising; // nac88c22 uses rising edge latching on bclk

    let mut osc = oscillator::RuntimeOscillator::new(
        oscillator::OscillatorType::Sine,
        SAMPLE_RATE as usize,
        Hertz::from_hertz(261.63), // middle C
    );

    // Split the SAI periperal into it's two subblock peripherals.
    let (sub_block_tx, sub_block_rx) = sai::split_subblocks(r.codec.peri.reborrow());

    // Configure the first sub block as the transmitter and the main clock source.
    let mut sai_tx = sai::Sai::new_asynchronous_with_mclk(
        sub_block_tx,
        r.codec.sck.reborrow(),
        r.codec.dacdat.reborrow(),
        r.codec.fs.reborrow(),
        r.codec.mclk.reborrow(),
        r.codec.tx_dma.reborrow(),
        audio_tx_buffer,
        tx_config,
    );

    // After the transmitter has been configured, mclk goes active.
    // Now we can configure our codec to run off the mclk signal.

    info!("initializing audio codec...");
    codec_init(&mut codec, adjusted_mclk).await;

    // Codec wants delay from mclk start to sending frames
    Timer::after_millis(250).await;

    // Configure the second sub block as a receiver,
    // syncronous to the mclk of the transmitter block.
    let mut sai_rx = sai::Sai::new_synchronous(
        sub_block_rx,
        r.codec.adcdat.reborrow(),
        r.codec.rx_dma.reborrow(),
        audio_rx_buffer,
        rx_config,
    );

    info!("starting SAI receiver");
    sai_rx.start().unwrap();

    info!("starting SAI loop");
    let mut buf = [0u32; HALF_DMA_BUFFER_LENGTH];
    loop {
        for frame in buf.chunks_mut(OUTPUT_CHANNEL_COUNT) {
            let value: U24 = osc.sample();
            for sample in frame.iter_mut() {
                *sample = value.to_sample();
            }
        }

        // A write() must be called before read() to start the
        // master (transmitter) clock used by the receiver.
        sai_tx.write(&buf).await.unwrap();
        sai_rx.read(&mut buf).await.unwrap();
    }

    Ok(())
}

/// https://www.nuvoton.com/export/resource-files/en-us--DS_NAU88C22_DataSheet_EN_Rev2.2.pdf
pub async fn codec_init(
    codec: &'_ mut nau88c22_rs::Nau88c22<
        asynch::i2c::I2cDevice<'_, CriticalSectionRawMutex, I2c<'static, mode::Async, i2c::Master>>,
    >,
    adjusted_mclk: u32,
) {
    codec
        .modify_clockcontrol1(|reg| {
            reg.with_clkm(false) // MCLK, pin#11 used as master clock
                .with_clkioen(false) // clock is in slave mode
        })
        .await
        .unwrap();

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
        .await
        .unwrap();

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
        .await
        .unwrap();

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
        .await
        .unwrap();

    // Configure the audio format
    codec
        .modify_audiointerface(|reg| {
            reg.with_wlen(0b10) // 24-bit words
                .with_aifmt(0b10) // standard i2s
                .with_mono(false) // stereo mode
        })
        .await
        .unwrap();

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

    // Configure the PLL.
    codec
        .modify_clockcontrol1(|reg| reg.with_mclksel(mclk_div))
        .await
        .unwrap();
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

    // Wait for the PLL to stabalize.
    Timer::after_millis(255).await;

    // Enable the PLL.
    codec
        .modify_powermanagement1(|reg| reg.with_pllen(true))
        .await
        .unwrap();

    // Enable the left aux in as the left ADC source.
    codec
        .modify_leftadcboost(|reg| {
            reg.with_lauxbstegain(0b101)
                .with_lpgabst(false)
                .with_lpgabstgaun(0)
        })
        .await
        .unwrap();

    // Enable the right aux in as the right ADC source.
    codec
        .modify_rightadcboost(|reg| {
            reg.with_rauxbstgain(0b101)
                .with_rpgabst(false)
                .with_rpgabstgain(0)
        })
        .await
        .unwrap();

    // DAC setup
    {
        // Disable DAC mutes.
        codec
            .modify_daccontrol(|reg| reg.with_automt(false).with_softmt(false))
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
    }

    // Output mixer setup
    {
        // Set the left main mix to use the left aux input.
        codec
            .modify_leftmixer(|reg| {
                reg //.with_lauxlmx(true) // mix in the left audio input
                    //.with_lauxmxgain(0b101)
                    .with_ldaclmx(true) // mix in the left dac output
            })
            .await
            .unwrap();

        // Set the right main mix to use the right aux input.
        codec
            .modify_rightmixer(|reg| {
                reg //.with_rauxrmx(true) // mix in the right aux input
                    //.with_rauxmxgain(0b101)
                    .with_rdacrmx(true) // mix in the right dac output
            })
            .await
            .unwrap();
    }

    // AUX out mixer setup
    {
        // Connect the left output mixer to the aux1 out.
        //
        // AUX1 can only connect to LMIX or RMIX.
        codec
            .modify_aux1mixer(|reg| {
                reg.with_auxiut1mt(false)
                    // TODO: shouldn't need both..
                    .with_rmixaux1(true) // mix in right mixer
                    .with_rdacaux1(true) // mix in right DAC output
            })
            .await
            .unwrap();

        // Connect the left output mixer to the aux2 out.
        //
        // AUX2 can only connect to LMIX but not RMIX.
        codec
            .modify_aux2mixer(|reg| {
                reg.with_auxout2mt(false)
                    // TODO: shouldn't need both..
                    .with_lmixaux2(true) // mix in left mixer
                    .with_ldacaux2(true) // mix in left dac output
            })
            .await
            .unwrap();
    }
}

fn mclk_div_from_u8(v: u8) -> MasterClockDivider {
    defmt::assert!((1..=63).contains(&v));
    MasterClockDivider::from_bits(v)
}

pub type SAITransmitter<'a> = sai::Sai<'a, peripherals::SAI1, u32>;
pub type SAIReceiver<'a> = sai::Sai<'a, peripherals::SAI1, u32>;

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

    let mut pll_f2: f32 = 0.0; // PLL Oscillator f2
    let mut pll_f1: f32 = nau_mckl_in; // input MCLK frequency

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

    let mut nau_pres_mckl: u8 = 3;
    let mut nau_r1_ready: u8 = 0;
    let mut integer_portion_n: u8 = 0; // Integer portion N
    let mut nau_pll: f32 = 0.0;
    let mut nau_int_pll: u32 = 0;

    // Fractional multipler R=f2/f1
    let mut nau_pll_r: f32 = pll_f2 / pll_f1;

    if (nau_pll_r > 6.0) && (nau_pll_r < 13.0) {
        nau_pres_mckl = 1;
    } else {
        nau_pll_r = pll_f2 / (pll_f1 * 2.0);
        nau_pres_mckl = 2;
    };

    // Check that the fractional multiplier is within the supported PLL clock bounds,
    // and then calculate the derrived integer (H) and fractional (K) portions.
    if (nau_pll_r > 6.0) && (nau_pll_r < 13.0) {
        // Get the intergel part by flooring the fractional multiplier.
        integer_portion_n = libm::floorf(nau_pll_r) as u8;

        // Derrive the fractional portion from the floored intger portion.
        nau_pll = nau_pll_r - (integer_portion_n as f32);
        // Multiply by 2^24 to derrive the 24-bit binary fractional value.
        nau_pll *= 16777216.0; // 2^24
        // Cast to u32 to round to nearest whole number.
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

    info!("NAC88C22 desired IMCLK rate (256*fs): {}hz", frq_imclk);
    info!("NAC88C22 fractional multiplier R=(f2/f1): {}", nau_pll_r);
    info!("NAC88C22 PLL oscillator (f2): {}hz", pll_f2);
    info!(
        "NAC88C22 integer portion N (Hex): 0x{:x}",
        integer_portion_n
    );
    info!("NAC88C22 fractional portion K (Hex): 0x{:x}", nau_int_pll);

    Ok((
        mclkseldiv,
        (nau_pres_mckl - 1),
        integer_portion_n,
        pll_k1,
        pll_k2,
        pll_k3,
    ))
}
