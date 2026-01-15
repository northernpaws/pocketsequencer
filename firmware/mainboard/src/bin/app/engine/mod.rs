//! The engine components consists of module that manage the audio
//! and sequencer engine, such as the audio codec and storage.

use defmt::info;
use embassy_executor::{SpawnError, Spawner};

use firmware::hardware::{
    AudioCodec, CodecSAIResources, audio::Audio, internal_storage::InternalStorage,
    sd_card::SdFilesystem,
};

use crate::engine::{audio::AudioManager, drive::Drive, midi::MIDIManager};

pub mod audio;
pub mod drive;
pub mod midi;
pub mod usb;

/// Spawns all the engine tasks and constructs the required wrappers to operate them.
pub async fn start_engine(
    spawner: Spawner,
    sd_card: SdFilesystem<'static>,
    internal_storage: InternalStorage,
    audio: Audio,
    audio_codec: AudioCodec,
    sai_resources: CodecSAIResources,
    usb_driver: embassy_stm32::usb::Driver<'static, embassy_stm32::peripherals::USB_OTG_HS>,
) -> Result<Engine, SpawnError> {
    // Start the tasks for managing the SD card and internal storage.
    //
    // Returns a wrapper around the drive channels to make creating
    // interfaces and dispatching drive operations easier.
    info!("engine: starting drive..");
    let drv = drive::start(sd_card, internal_storage, spawner).await?;

    // Start the MIDI processing task.
    //
    // This collects and processes MIDI events from USB, UART, etc.
    //
    // Returns a convenince wrapper around the MIDI channels for
    // managing the MIDI routing table.
    info!("engine: starting midi..");
    let midi_manager = midi::start(spawner)?;

    // Start the USB handler tasks.
    //
    // These handle responding to the USB class connections.
    info!("engine: starting usb..");
    usb::start(
        spawner,
        usb_driver,
        midi_manager.make_endpoint(midi::MIDISource::Usb),
    );

    // Start the tasks for managing the audio interface.
    //
    // Initialize the audio towards the end, some parts of
    // the filesystem and USB setup will cause the ringbuffer
    // to overrun otherwise.
    info!("engine: starting audio..");
    let audio_manager = audio::start(spawner, audio_codec, audio, sai_resources)?;

    Ok(Engine::new(drv, midi_manager, audio_manager))
}

/// Wraps the components of the engine into a convinent struct.
pub struct Engine {
    /// Wraps the channels required for communicating with the drive task.
    pub drive: Drive<'static>,
    /// Wraps the channels required for communication with the MIDI tasks.
    pub midi: MIDIManager,
    /// Wraps the channels required for communication with the audio tasks.
    pub audio: AudioManager,
}

impl Engine {
    /// Constructs a new engine.
    fn new(drive: Drive<'static>, midi: MIDIManager, audio: AudioManager) -> Self {
        Self { drive, midi, audio }
    }
}
