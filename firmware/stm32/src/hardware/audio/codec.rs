use defmt::{error, info};
use derive_more::{Display, Error};

use embassy_embedded_hal::shared_bus::{I2cDeviceError, asynch};
use embassy_stm32::{
    i2c::{self, I2c},
    mode,
};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

use nau88c22_rs::{
    AudioConfig, Aux1OutputConfig, Aux2OutputConfig, ClockConfig, DACChannelConfig, DACConfig,
    InitError, InitializationConfig,
};

/// https://www.nuvoton.com/export/resource-files/en-us--DS_NAU88C22_DataSheet_EN_Rev2.2.pdf
pub async fn codec_init(
    codec: &'_ mut nau88c22_rs::Nau88c22<
        asynch::i2c::I2cDevice<'_, CriticalSectionRawMutex, I2c<'static, mode::Async, i2c::Master>>,
    >,
    adjusted_mclk: u32,
    sample_rate: f32,
) -> Result<(), InitError<I2cDeviceError<i2c::Error>>> {
    codec
        .initialize(
            InitializationConfig {
                audio: AudioConfig {
                    aux1_output: Some(Aux1OutputConfig {
                        muted: false,
                        attenuation_6_0db: false,
                        left_mixer_input: false,
                        left_dac_input: false,
                        right_adc_mixer_input: false,
                        right_mixer_input: false,
                        right_dac_input: true,
                    }),
                    aux2_output: Some(Aux2OutputConfig {
                        muted: false,
                        aux1_interconnect_input: false,
                        left_adc_mix_input: false,
                        left_mixer_input: false,
                        left_dac_input: true,
                    }),
                    enable_micbias: false,
                    enable_headphone_right: false,
                    enable_headphone_left: false,
                    input_mixer_right: None,
                    input_mixer_left: None,
                    output_mixer_right: None,
                    output_mixer_left: None,
                    adc: None,
                    dac: Some(DACConfig {
                        oversample_128: false,
                        dac_left: Some(DACChannelConfig {
                            gain: Default::default(),
                        }),
                        dac_right: Some(DACChannelConfig {
                            gain: Default::default(),
                        }),
                    }),
                    format: Default::default(),
                },

                // SAI1 clock is close enough to sample rate
                // to work without needing the codec's PLL.
                clock: Some(ClockConfig {
                    mclk: adjusted_mclk as f32,
                    sample_rate,
                }),
            },
            embassy_time::Delay,
        )
        .await
    /*
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
    }*/
}
