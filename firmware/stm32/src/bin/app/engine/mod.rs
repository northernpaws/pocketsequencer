//! The engine components consists of module that manage the audio
//! and sequencer engine, such as the audio codec and storage.

use defmt::info;
use embassy_executor::Spawner;
use embassy_sync::{channel::Channel, pubsub::PubSubChannel};
use firmware::hardware::{
    CodecSAIResources, audio::Audio, internal_storage::InternalStorage, sd_card::SdFilesystem,
};

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
    // Start the tasks for managing the SD card and internal storage.
    //
    // Returns a wrapper around the drive channels to make creating
    // interfaces and dispatching drive operations easier.
    info!("engine: starting drive..");
    let drv = drive::start(sd_card, internal_storage, spawner)
        .await
        .unwrap();

    // Start the tasks for managing the audio interface.
    info!("engine: starting audio..");
    audio::start(audio, sai_resources).unwrap();

    // Start the MIDI processing task.
    //
    // This collects and processes MIDI events from USB, UART, etc.
    //
    // Returns a convenince wrapper around the MIDI channels for
    // managing the MIDI routing table.
    info!("engine: starting midi..");
    let midi_manager = midi::start(spawner).unwrap();

    // Start the USB handler tasks.
    //
    // These handle responding to the USB class connections.
    info!("engine: starting usb..");
    usb::start(
        spawner,
        usb_driver,
        midi_manager.make_endpoint(midi::MIDISource::Usb),
    );

    (drv, midi_manager)
}
