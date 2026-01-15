use catalina::engine::audio::frame;
use embassy_executor::{SpawnError, Spawner};
use embassy_sync::{
    blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex},
    zerocopy_channel,
};
use firmware::hardware::{self, CodecSAIResources, audio::Audio};

use engine::AudioEngine;
use static_cell::StaticCell;

pub mod codec;
pub mod engine;
pub mod render;

pub type SampleBlock = heapless::Vec<frame::Stereo<f32>, { render::HALF_DMA_BUFFER_LENGTH }>;
pub type SystemAudioSender =
    zerocopy_channel::Sender<'static, CriticalSectionRawMutex, SampleBlock>;
pub type SystemAudioReceiver =
    zerocopy_channel::Receiver<'static, CriticalSectionRawMutex, SampleBlock>;

static SYSTEM_SAMPLE_BLOCKS: StaticCell<[SampleBlock; 2]> = StaticCell::new();
static SYSTEM_CHANNEL: StaticCell<
    zerocopy_channel::Channel<'_, CriticalSectionRawMutex, SampleBlock>,
> = StaticCell::new();

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

    // Allocate two alternating buffers for the sample transfers.
    let system_sample_blocks =
        SYSTEM_SAMPLE_BLOCKS.init([heapless::Vec::new(), heapless::Vec::new()]);

    // Establish a zero-copy channel for transferring received audio samples between tasks.

    let system_channel = SYSTEM_CHANNEL.init(zerocopy_channel::Channel::new(system_sample_blocks));
    let (system_sender, system_receiver) = system_channel.split();

    // Starts the audio rendering task that manages the
    // ringbuffer and DMA transfer to the SAI peripheral.
    render::spawn_task(audio, r, engine, system_receiver)?;

    Ok(AudioManager::new(command_sender, system_sender))
}

/// Wraps the control channels for the audio tasks to provide a unified interface.
pub struct AudioManager {
    command_sender: codec::AudioCommandSender,
    pub system_audio: SystemAudioSender,
}

impl AudioManager {
    pub fn new(command_sender: codec::AudioCommandSender, system_audio: SystemAudioSender) -> Self {
        Self {
            command_sender,
            system_audio,
        }
    }
}
