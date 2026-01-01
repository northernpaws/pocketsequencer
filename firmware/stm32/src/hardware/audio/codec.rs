use defmt::{error, info};
use derive_more::{Display, Error};

use embassy_embedded_hal::shared_bus::asynch;
use embassy_stm32::{
    i2c::{self, I2c},
    mode,
};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

use embassy_time::Timer;
use nau88c22_rs::registers::{
    AudioInterfaceDataFormat, GPIO1FunctionSelect, GPIO1PllDivisor, WordLength,
};

/// https://www.nuvoton.com/export/resource-files/en-us--DS_NAU88C22_DataSheet_EN_Rev2.2.pdf
pub async fn codec_init(
    codec: &'_ mut nau88c22_rs::Nau88c22<
        asynch::i2c::I2cDevice<'_, CriticalSectionRawMutex, I2c<'static, mode::Async, i2c::Master>>,
    >,
    adjusted_mclk: u32,
    sample_rate: f32,
) {
    // Software reset the codec to a known state.
    info!("software resetting audio codec");
    codec.reset().await.unwrap(); // register 0

    // Give time for the reset to finish.
    loop {
        Timer::after_millis(25).await;

        // Check that the device has finished reset and is responsive.
        let device_id = codec.read_deviceid().await.unwrap();
        info!("audio codec device ID: {}", device_id.id());
        if device_id.id() == 26 {
            break;
        }
    }

    // First enable the aux mixers,internal tie-off, and slow-charge impedance.
    codec
        .modify_powermanagement1(|reg| {
            reg.with_dcbufen(false) // false for lower then 3.6v operation
                .with_aux1mxen(true) // Enable the aux 1&2 output mixers.
                .with_aux2mxen(true)
                .with_pllen(false) // Ensure the PLL is disabled.
                .with_micbiasen(false) // Enable the micbias buffer output.
                .with_abiasen(true) // Enable internal Analog Bias Buffer.
                .with_iobufen(true) // Internal Tie-off Buffer In Non-boost 1.0X Mode
                .with_refimp(0b01) // VREF Impedance Select - 80k
        })
        .await
        .unwrap();

    // Wait 250ms for the cap to charge to avoid popping/clicking on the outputs.
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
        // codec
        //     .modify_adccontrol(|reg| {
        //         reg.with_adcos(true) //128x oversampling
        //             .with_hpf(0)
        //             .with_hpfen(false) // disable high pass filter
        //     })
        //     .await
        //     .unwrap();

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
        // codec
        //     .modify_daccontrol(|reg| {
        //         reg.with_dacos(true) // 128x oversampling
        //     })
        //     .await
        //     .unwrap();

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
            // TODO: should the sample rate here be the desired sampling rate, or the one derrived from the SAI divisor?
            let (mclk_div, pllmclk, plln, pllk1, pllk2, pllk3) =
                nau88c22_calc_pll(adjusted_mclk as f32, sample_rate).unwrap();

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
pub fn nau88c22_calc_pll(
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
