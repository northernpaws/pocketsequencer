use defmt::info;
use embassy_executor::{SpawnError, Spawner};

use firmware::hardware::{self, audio::AudioParameters};
use nau88c22_rs::{
    AudioConfig, Aux1OutputConfig, Aux2OutputConfig, ClockConfig, DACChannelConfig, DACConfig,
    InitializationConfig,
};

pub fn spawn(
    spawner: Spawner,
    codec: hardware::AudioCodec,
    command_receiver: super::AudioCommandReceiver,
    params: AudioParameters,
) -> Result<(), SpawnError> {
    spawner.spawn(task(codec, command_receiver, params)?);

    Ok(())
}

#[embassy_executor::task]
pub async fn task(
    codec: hardware::AudioCodec,
    command_receiver: super::AudioCommandReceiver,
    params: AudioParameters,
) -> ! {
    // should never return
    let err = inner_task(codec, command_receiver, params).await;
    panic!("audio codec task exited unexpectedly: {:?}", err);
}

#[derive(Debug)]
enum Never {}

async fn inner_task(
    mut codec: hardware::AudioCodec,
    command_receiver: super::AudioCommandReceiver,
    params: AudioParameters,
) -> Result<(), Never> {
    let mut routing_rable: super::AudioRoutingTable = Default::default();

    // Perform the initial codec initialization.
    info!("initializing codec");
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
                    mclk: params.adjusted_mclk as f32,
                    sample_rate: params.actual_sample_rate,
                }),
            },
            embassy_time::Delay,
        )
        .await
        .unwrap();

    loop {
        let command = command_receiver.receive().await;

        match command {
            super::AudioCommand::UpdateRoutingTable(audio_routing_table) => {
                routing_rable = audio_routing_table;

                // TODO: put in retry loop instead
                routing_rable.apply(&mut codec).await.unwrap();
            }
        }
    }
}
