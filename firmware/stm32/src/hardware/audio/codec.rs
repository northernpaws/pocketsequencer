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
                    enable_speaker_right: false,
                    enable_speaker_left: false,
                    adc: None,
                    dac: Some(DACConfig {
                        oversample_128: false,
                        dac_left: Some(DACChannelConfig { gain: 0xFF }),
                        dac_right: Some(DACChannelConfig { gain: 0xFF }),
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
}
