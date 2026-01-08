//! The engine components consists of module that manage the audio
//! and sequencer engine, such as the audio codec and storage.

use defmt::info;
use embassy_executor::Spawner;
use embassy_sync::{channel::Channel, pubsub::PubSubChannel};
use firmware::hardware::{
    CodecSAIResources, audio::Audio, internal_storage::InternalStorage, sd_card::SdFilesystem,
};

use crate::engine::midi::{MIDIDestinations, MIDIEndpoint, MIDISources};

pub mod audio;
pub mod drive;
pub mod midi;
pub mod usb;

/// Spawns all the engine tasks and constructs the required wrappers to operate them.
pub async fn start_engine(
    spawner: Spawner,
    sd_card: SdFilesystem<'static>,
    internal_storage: InternalStorage,
    audio: Audio<'static>,
    sai_resources: CodecSAIResources,
    usb_driver: embassy_stm32::usb::Driver<'static, embassy_stm32::peripherals::USB_OTG_HS>,
) -> (drive::Drive<'static>, midi::MIDIManager) {
    /// Channel for sending commands to the filesystem task.
    static DRIVE_COMMANDS: drive::CommandChannel = Channel::new();
    /// PubSub channel for receiving results from the filesystem task.
    static DRIVE_RESULTS: drive::CommandResultChannel = PubSubChannel::new();

    // Start the tasks for managing the SD card and internal storage.
    //
    // Returns a wrapper around the drive channels to make creating
    // interfaces and dispatching drive operations easier.
    info!("engine: starting drive..");
    let drv = drive::start(
        sd_card,
        internal_storage,
        spawner,
        &DRIVE_COMMANDS,
        &DRIVE_RESULTS,
    )
    .await
    .unwrap();

    // Start the tasks for managing the audio interface.
    info!("engine: starting audio..");
    audio::start(audio, sai_resources).unwrap();

    /// Channel for MIDI routing table updates.
    static MIDI_ROUTING_CHANNEL: midi::MIDIRoutingChannel = Channel::new();

    // Channels for sending MIDI messages to destinations.
    static MIDI_DEST_SERIAL: midi::MIDIDestinationChannel = Channel::new();
    static MIDI_DEST_USB: midi::MIDIDestinationChannel = Channel::new();

    // Channels for receiving MIDI messages from sources.
    static MIDI_SRC_SERIAL: midi::MIDISourceChannel = Channel::new();
    static MIDI_SRC_USB: midi::MIDISourceChannel = Channel::new();

    // Start the MIDI processing task.
    //
    // This collects and processes MIDI events from USB, UART, etc.
    //
    // Returns a convenince wrapper around the MIDI channels for
    // managing the MIDI routing table.
    info!("engine: starting midi..");
    let midi_manager = midi::start(
        spawner,
        // Channel for receiving external MIDI events, i.e. from USB.
        MIDISources::new(MIDI_SRC_USB.receiver(), MIDI_SRC_SERIAL.receiver()),
        // Channels for routing MIDI messages to their destinations.
        MIDIDestinations::new(MIDI_DEST_USB.sender(), MIDI_DEST_SERIAL.sender()),
        // Channel for updating the MIDI routing table.
        &MIDI_ROUTING_CHANNEL,
    )
    .unwrap();

    // Start the USB handler tasks.
    //
    // These handle responding to the USB class connections.
    info!("engine: starting usb..");
    usb::start(
        spawner,
        usb_driver,
        MIDIEndpoint::new(MIDI_DEST_USB.receiver(), MIDI_SRC_USB.sender()),
    );

    (drv, midi_manager)
}
