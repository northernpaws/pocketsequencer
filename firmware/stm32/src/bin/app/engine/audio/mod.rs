use embassy_executor::{SpawnError, Spawner};
use firmware::hardware::{self, CodecSAIResources, audio::Audio};

use engine::AudioEngine;

pub mod codec;
pub mod engine;
pub mod instrument;
pub mod render;

/// Start the audio subsystem.
pub fn start(
    spawner: Spawner,
    codec: hardware::AudioCodec,
    audio: Audio,
    r: CodecSAIResources,
) -> Result<(), SpawnError> {
    let engine = AudioEngine::new();

    // Starts the codec management task.
    codec::start(spawner, codec, audio.params())?;

    // Starts the audio rendering task.
    render::spawn_task(audio, r, engine)?;

    Ok(())
}
