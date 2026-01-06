use embassy_executor::SpawnError;
use firmware::hardware::{CodecSAIResources, audio::Audio};

pub mod engine;
pub mod instrument;
pub mod task;

use engine::AudioEngine;

/// Start the audio subsystem.
pub fn start(audio: Audio<'static>, r: CodecSAIResources) -> Result<(), SpawnError> {
    let engine = AudioEngine::new();

    task::start_audio(audio, r, engine)
}
