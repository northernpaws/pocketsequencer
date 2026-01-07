use alloc::boxed::Box;
use embassy_executor::{SpawnError, Spawner};
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    channel::{self, Channel, Receiver, Sender},
};
use midly::{
    MidiMessage,
    live::{MtcQuarterFrameMessage, SystemRealtime},
    num::{u4, u7, u14},
};

mod tasks;

pub const MIDI_EVENT_BUFFER_COUNT: usize = 4;

/// Channel for MIDI events received from a serial port or the USB peripheral.
pub type MIDIRxChannel = Channel<CriticalSectionRawMutex, MIDIEvent, MIDI_EVENT_BUFFER_COUNT>;
/// Channel for MIDI events transmitted to a serial port or USB peripheral.
pub type MIDITxChannel = Channel<CriticalSectionRawMutex, MIDIEvent, MIDI_EVENT_BUFFER_COUNT>;

pub type MIDIEventReceiver<'a> =
    Receiver<'a, CriticalSectionRawMutex, MIDIEvent, MIDI_EVENT_BUFFER_COUNT>;
pub type MIDIEventSender<'a> =
    Sender<'a, CriticalSectionRawMutex, MIDIEvent, MIDI_EVENT_BUFFER_COUNT>;

/// A "system common event", as defined by the MIDI spec.
///
/// Modified from the midly library so that arrays use a heapless
/// vec instead to avoid lifetime issues with channels. The packet
/// size of 64 is based on the USB MIDI class packet size.
#[derive(Clone, PartialEq, Eq, Debug, Hash)]
pub enum SystemCommon {
    /// A system-exclusive event.
    ///
    /// System Exclusive events start with a `0xF0` byte and finish with a `0xF7` byte, but this
    /// slice does not include either: it only includes data bytes in the `0x00..=0x7F` range.
    SysEx(Box<[u7]>),

    /// A MIDI Time Code Quarter Frame message, carrying a tag type and a 4-bit tag value.
    MidiTimeCodeQuarterFrame(MtcQuarterFrameMessage, u4),

    /// The number of MIDI beats (6 x MIDI clocks) that have elapsed since the start of the
    /// sequence.
    SongPosition(u14),

    /// Select a given song index.
    SongSelect(u7),

    /// Request the device to tune itself.
    TuneRequest,

    /// An undefined System Common message, with arbitrary data bytes.
    Undefined(u8, Box<[u7]>),
}

#[derive(Clone, PartialEq, Eq, Debug, Hash)]
pub enum MIDIMessage {
    /// A MIDI message associated with a channel, carrying musical data.
    ///
    /// Status byte in the range `0x80 ..= 0xEF`.
    Midi {
        /// The MIDI channel that this message is associated with.
        channel: u4,
        /// The MIDI message type and associated data.
        message: MidiMessage,
    },

    /// A System Common message, as defined by the MIDI spec, including System Exclusive events.
    ///
    /// Status byte in the range `0xF0 ..= 0xF7`.
    Common(SystemCommon),

    /// A one-byte System Realtime message.
    ///
    /// Status byte in the range `0xF8 ..= 0xFF`.
    Realtime(SystemRealtime),
}

/// Identifies the source of a MIDI message, primarily
/// used for routing MIDI messages between endpoints.
#[derive(Clone, PartialEq, Eq, Debug, Hash, defmt::Format)]
pub enum MIDISource {
    /// Identifies a MIDI message coming
    /// from the USB driver.
    Usb,

    /// Identifiers a MIDI messages coming
    /// from the hardware MIDI/serial port.
    Serial,
}

/// A MIDI event raised by a MIDI interface/endpoint.
///
/// Contains the relevant MIDI message, along with some
/// identifying information for MIDI routing.
#[derive(Clone, PartialEq, Eq, Debug, Hash)]
pub struct MIDIEvent {
    /// Identifies the source of the MIDI event.
    ///
    /// Primarily used for routing.
    pub source: MIDISource,

    /// The MIDI message.
    pub message: MIDIMessage,
}

/// Defines the table used for routing incoming and outgoing MIDI messages.
#[derive(Clone, Debug, PartialEq, Eq, defmt::Format)]
pub struct MIDIRoutingTable {}

/// Alias type for a channel used to update the MIDI routing table.
pub type MIDIRoutingChannel = channel::Channel<CriticalSectionRawMutex, MIDIRoutingTable, 2>;
pub type MIDIRoutingReceiver<'a> =
    channel::Receiver<'a, CriticalSectionRawMutex, MIDIRoutingTable, 2>;

/// Starts the MIDI processing components.
pub fn start(
    spawner: Spawner,
    // Receives external MIDI events.
    midi_event_rx: MIDIEventReceiver<'static>,
    // Sends external MIDI events.
    midi_event_tx: MIDIEventSender<'static>,

    // Channel used to update the routing table.
    routing_channel: &'static MIDIRoutingChannel,
) -> Result<MIDIManager, SpawnError> {
    let routing_receiver = routing_channel.receiver();

    // Start the tasks for MIDI handling.
    tasks::start_midi(spawner, midi_event_rx, midi_event_tx, routing_receiver)?;

    Ok(MIDIManager { routing_channel })
}

/// Wraps the channels for communicating with the MIDI tasks for convenience.
pub struct MIDIManager {
    routing_channel: &'static MIDIRoutingChannel,
}

impl MIDIManager {
    /// Manually send a MIDI message into the MIDI task.
    pub async fn send_message(&mut self, message: MIDIMessage) {
        todo!()
    }

    /// Updates the active MIDI routing table with a new table.
    pub async fn update_routing_table(&mut self, table: MIDIRoutingTable) {
        self.routing_channel.sender().send(table).await
    }
}
