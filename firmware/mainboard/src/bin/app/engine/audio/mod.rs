use embassy_executor::{SpawnError, Spawner};
use firmware::hardware::{self, CodecSAIResources, audio::Audio};

use engine::AudioEngine;

use crate::engine::audio::codec::AudioCommandSender;

pub mod codec;
pub mod engine;
pub mod render;

/// Start the audio subsystem.
pub fn start(
    spawner: Spawner,
    codec: hardware::AudioCodec,
    audio: Audio,
    r: CodecSAIResources,
) -> Result<AudioManager, SpawnError> {
    let engine = AudioEngine::new();

    // Starts the codec management task.
    let command_sender = codec::start(spawner, codec, audio.params())?;

    // Starts the audio rendering task that manages the
    // ringbuffer and DMA transfer to the SAI peripheral.
    render::spawn_task(audio, r, engine)?;

    Ok(AudioManager::new(command_sender))
}

/// Wraps the control channels for the audio tasks to provide a unified interface.
pub struct AudioManager {
    command_sender: codec::AudioCommandSender,
}

impl AudioManager {
    pub fn new(command_sender: codec::AudioCommandSender) -> Self {
        Self { command_sender }
    }
}
