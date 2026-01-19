use defmt::info;
use embassy_executor::{SpawnError, Spawner};

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use firmware::hardware::{self, audio::AudioParameters};
use nau88c22_rs::{
    ADCConfig, AudioConfig, AudioFormat, Aux1OutputConfig, Aux2OutputConfig, ClockConfig,
    DACChannelConfig, DACConfig, InitializationConfig, LeftOutputMixerConfig,
    RightOutputMixerConfig,
    registers::{AudioInterfaceDataFormat, WordLength},
};

pub fn spawn(
    spawner: Spawner,
    codec: hardware::AudioCodec,
    command_receiver: super::AudioCommandReceiver,
    params: AudioParameters,
    audio_ready: &'static Signal<CriticalSectionRawMutex, bool>,
) -> Result<(), SpawnError> {
    spawner.spawn(task(codec, command_receiver, params, audio_ready)?);

    Ok(())
}

#[embassy_executor::task]
pub async fn task(
    codec: hardware::AudioCodec,
    command_receiver: super::AudioCommandReceiver,
    params: AudioParameters,
    audio_ready: &'static Signal<CriticalSectionRawMutex, bool>,
) -> ! {
    // should never return
    let err = inner_task(codec, command_receiver, params, audio_ready).await;
    panic!("audio codec task exited unexpectedly: {:?}", err);
}

#[derive(Debug)]
enum Never {}

async fn inner_task(
    mut codec: hardware::AudioCodec,
    command_receiver: super::AudioCommandReceiver,
    params: AudioParameters,
    audio_ready: &'static Signal<CriticalSectionRawMutex, bool>,
) -> Result<(), Never> {
    let mut routing_rable: super::AudioRoutingTable = Default::default();

    // Perform the initial codec initialization.
    //
    // This configures the audio paths and routing
    // to go to all the places that we expect and
    // make assumptions about when changing the
    // audio routing settings.
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
                        right_mixer_input: true,
                        right_dac_input: false,
                    }),
                    aux2_output: Some(Aux2OutputConfig {
                        muted: false,
                        aux1_interconnect_input: false,
                        left_adc_mix_input: false,
                        left_mixer_input: true,
                        left_dac_input: false,
                    }),
                    enable_micbias: false,
                    enable_headphone_right: false,
                    enable_headphone_left: false,
                    input_mixer_right: None,
                    input_mixer_left: None,
                    output_mixer_right: Some(RightOutputMixerConfig {
                        aux_gain: Default::default(),
                        aux_input: false,
                        adc_input_gain: Default::default(),
                        adc_input: false,
                        dac_input: true,
                    }),
                    output_mixer_left: Some(LeftOutputMixerConfig {
                        aux_gain: Default::default(),
                        aux_input: false,
                        adc_input_gain: Default::default(),
                        adc_input: false,
                        dac_input: true,
                    }),
                    enable_speaker_right: false,
                    enable_speaker_left: false,
                    adc: Some(ADCConfig {
                        oversample_128: false,
                        adc_left: Some(nau88c22_rs::ADCChannelConfig {
                            gain: Default::default(),
                        }),
                        adc_right: Some(nau88c22_rs::ADCChannelConfig {
                            gain: Default::default(),
                        }),
                    }),
                    dac: Some(DACConfig {
                        oversample_128: true,
                        dac_left: Some(DACChannelConfig { gain: 0xFF - 20 }),
                        dac_right: Some(DACChannelConfig { gain: 0xFF - 20 }), // 1 = 0.5dB attenuation
                    }),
                    format: AudioFormat {
                        word_length: WordLength::Word32Bit,
                        data_format: AudioInterfaceDataFormat::StandardI2S,
                    },
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

    codec
        .modify_eq1highcutoff(|reg| reg.with_eqm(false))
        .await
        .unwrap();

    // Start with in softmute, and then transition out of it to help reduce pops.
    codec
        .modify_daccontrol(|reg| reg.with_automt(false).with_softmt(true))
        .await
        .unwrap();

    // DAC -> DAC loopback.
    // codec
    //     .modify_companding(|reg| reg.with_addap(true))
    //     .await
    //     .unwrap();

    // Wait for the audio task to be ready and have sent some
    // initial frames, and then move the DAC out of softmute.
    loop {
        if audio_ready.wait().await {
            codec
                .modify_daccontrol(|reg| reg.with_softmt(false))
                .await
                .unwrap();

            break;
        }
    }

    codec
        .modify_daccontrol(|reg| reg.with_ldacpl(false).with_rdacpl(false))
        .await
        .unwrap();

    codec
        .modify_dacdither(|reg| reg.with_analog_dither(0b00000).with_mod_dither(0b00000))
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
