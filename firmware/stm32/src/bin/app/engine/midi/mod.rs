use alloc::{boxed::Box, vec::Vec};
use embassy_executor::{SpawnError, Spawner};
use embassy_futures::select::select;
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

/// Channel for MIDI messages transmitted to a serial port or USB peripheral.
pub type MIDIDestinationChannel =
    Channel<CriticalSectionRawMutex, MIDIMessage, MIDI_EVENT_BUFFER_COUNT>;
pub type MIDIDestinationReceiver<'a> =
    Receiver<'a, CriticalSectionRawMutex, MIDIMessage, MIDI_EVENT_BUFFER_COUNT>;
pub type MIDIDestinationSender<'a> =
    Sender<'a, CriticalSectionRawMutex, MIDIMessage, MIDI_EVENT_BUFFER_COUNT>;

/// Channel for MIDI events received from a serial port or USB peripheral.
pub type MIDISourceChannel = Channel<CriticalSectionRawMutex, MIDIMessage, MIDI_EVENT_BUFFER_COUNT>;
pub type MIDISourceReceiver<'a> =
    Receiver<'a, CriticalSectionRawMutex, MIDIMessage, MIDI_EVENT_BUFFER_COUNT>;
pub type MIDISourceSender<'a> =
    Sender<'a, CriticalSectionRawMutex, MIDIMessage, MIDI_EVENT_BUFFER_COUNT>;

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

/// Filter for the MIDI routing table.
#[derive(Clone, Debug, PartialEq, Eq, defmt::Format)]
pub enum MIDIMessageFilter {
    /// Filter MIDI messages based on their channel.
    Channel(u8),
}

/// Specifies the destination for a routed MIDI message.
#[derive(Clone, Debug, PartialEq, Eq, defmt::Format)]
pub enum MIDIDestination {
    /// Writes the routed MIDI messages to the USB interface.
    Usb { channel: u8 },
    /// Writes the routed MIDI messages to the serial/hardware MIDI interface.
    Serial { channel: u8 },
}

/// A route in the MIDI routing table.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct MIDIRoute {
    /// Filters to apply to the MIDI route.
    filters: Vec<MIDIMessageFilter>,
    /// Specifies the destination for the route.
    destination: MIDIDestination,
}

/// Defines the table used for routing incoming and outgoing MIDI messages.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct MIDIRoutingTable {
    routes: Vec<MIDIRoute>,
}

/// Used to initialize a default routing table.
impl Default for MIDIRoutingTable {
    fn default() -> Self {
        Self {
            routes: Default::default(),
        }
    }
}

/// Alias type for a channel used to update the MIDI routing table.
pub type MIDIRoutingChannel = channel::Channel<CriticalSectionRawMutex, MIDIRoutingTable, 2>;
pub type MIDIRoutingReceiver<'a> =
    channel::Receiver<'a, CriticalSectionRawMutex, MIDIRoutingTable, 2>;

/// Container struct holding the channels for routing
/// MIDI messages to various destinations.
pub struct MIDIDestinations {
    /// Channel to send MIDI messages to the USB interface.
    usb: MIDIDestinationSender<'static>,
    /// Channel to send MIDI messages to the hardware/serial interface.
    serial: MIDIDestinationSender<'static>,
}

impl MIDIDestinations {
    pub fn new(
        usb: MIDIDestinationSender<'static>,
        serial: MIDIDestinationSender<'static>,
    ) -> Self {
        Self { usb, serial }
    }

    /// Routes a messages to the appropriate destination channel.
    pub async fn route_message(&mut self, message: MIDIMessage, destination: MIDIDestination) {
        match destination {
            MIDIDestination::Usb { channel } => self.usb.send(message).await,
            MIDIDestination::Serial { channel } => self.serial.send(message).await,
        }
    }
}

/// Container for holding the channels used to receive MIDI messages from various sources.
pub struct MIDISources {
    usb: MIDISourceReceiver<'static>,
    serial: MIDISourceReceiver<'static>,
}

impl MIDISources {
    pub fn new(usb: MIDISourceReceiver<'static>, serial: MIDISourceReceiver<'static>) -> Self {
        Self { usb, serial }
    }

    /// Wait for the next message from any of the channels,
    /// and tag it with the source it came from for routing.
    pub async fn next(&mut self) -> MIDIEvent {
        match select(self.usb.receive(), self.serial.receive()).await {
            embassy_futures::select::Either::First(message) => MIDIEvent {
                source: MIDISource::Usb,
                message,
            },
            embassy_futures::select::Either::Second(message) => MIDIEvent {
                source: MIDISource::Serial,
                message,
            },
        }
    }
}

/// Container for the two channels used by MIDI endpoints to manage their messages.
pub struct MIDIEndpoint {
    /// Channel receiver used by the endpoint to
    /// receive MIDI messages it should transmit.
    sink: MIDIDestinationReceiver<'static>,

    /// Channel sender used by the endpoint to send messages
    /// it received that the MIDI subsystem should handle.
    source: MIDISourceSender<'static>,
}

impl MIDIEndpoint {
    pub fn new(sink: MIDIDestinationReceiver<'static>, source: MIDISourceSender<'static>) -> Self {
        Self { sink, source }
    }

    /// Sends a MIDI message from the endpoint to the MIDI processing task.
    pub async fn send_message(&self, message: MIDIMessage) {
        self.source.send(message).await
    }

    /// Receives the next MIDI message for the endpoint to sink.
    pub async fn receive_message(&self) -> MIDIMessage {
        self.sink.receive().await
    }

    /// Clears both the message channels of any backlog.
    pub fn clear(&self) {
        self.source.clear();
        self.sink.clear();
    }
}

/// High-level container struct for initializing the various channel
/// senders and receivers needed by the MIDI task and MIDI endpoints.
struct MIDIChannels {
    /// Channel for MIDI routing table updates.
    routing: &'static MIDIRoutingChannel,

    // Channels for sending MIDI messages to destinations.
    dest_serial: &'static MIDIDestinationChannel,
    dest_usb: &'static MIDIDestinationChannel,

    // Channels for receiving MIDI messages from sources.
    src_serial: &'static MIDISourceChannel,
    src_usb: &'static MIDISourceChannel,
}

impl MIDIChannels {
    fn new() -> Self {
        // Initiaizing the channels here as statics isn't exactly the most
        // trasparent thing to the caller, but it significantly cleans up
        // the messy-ness of constructing them all.

        /// Channel for MIDI routing table updates.
        static MIDI_ROUTING_CHANNEL: MIDIRoutingChannel = Channel::new();

        // Channels for sending MIDI messages to destinations.
        static MIDI_DEST_SERIAL: MIDIDestinationChannel = Channel::new();
        static MIDI_DEST_USB: MIDIDestinationChannel = Channel::new();

        // Channels for receiving MIDI messages from sources.
        static MIDI_SRC_SERIAL: MIDISourceChannel = Channel::new();
        static MIDI_SRC_USB: MIDISourceChannel = Channel::new();

        Self {
            routing: &MIDI_ROUTING_CHANNEL,
            dest_serial: &MIDI_DEST_SERIAL,
            dest_usb: &MIDI_DEST_USB,
            src_serial: &MIDI_SRC_SERIAL,
            src_usb: &MIDI_SRC_USB,
        }
    }

    /// Construct a set of channel accessors for allowing an endpoint access to the MIDI task.
    fn make_endpoint(&self, source: MIDISource) -> MIDIEndpoint {
        match source {
            MIDISource::Usb => MIDIEndpoint {
                sink: self.dest_usb.receiver(),
                source: self.src_usb.sender(),
            },
            MIDISource::Serial => todo!(),
        }
    }

    /// Makes a set of MIDI sources used by the MIDI task to source messages from endpoints.
    fn make_sources(&self) -> MIDISources {
        MIDISources {
            usb: self.src_usb.receiver(),
            serial: self.src_serial.receiver(),
        }
    }

    /// Makes a set of MIDI destinations used by the MIDI task to route messages to endpoints.
    fn make_destinations(&self) -> MIDIDestinations {
        MIDIDestinations {
            usb: self.dest_usb.sender(),
            serial: self.dest_serial.sender(),
        }
    }

    fn make_table_updater(&self) -> MIDIRoutingReceiver<'static> {
        self.routing.receiver()
    }
}

/// Starts the MIDI processing components.
pub fn start(spawner: Spawner) -> Result<MIDIManager, SpawnError> {
    // Container that initializations the static channels used by the MIDI task.
    let midi_channels = MIDIChannels::new();

    // Spawn the tasks for MIDI handling.
    tasks::spawn(
        spawner,
        midi_channels.make_table_updater(),
        midi_channels.make_sources(),
        midi_channels.make_destinations(),
    )?;

    Ok(MIDIManager {
        channels: midi_channels,
    })
}

/// Wraps the channels for communicating with the MIDI tasks for convenience.
pub struct MIDIManager {
    channels: MIDIChannels,
}

impl MIDIManager {
    /// Manually send a MIDI message into the MIDI task.
    pub async fn send_message(&mut self, message: MIDIMessage) {
        todo!()
    }

    /// Updates the active MIDI routing table with a new table.
    pub async fn update_routing_table(&mut self, table: MIDIRoutingTable) {
        self.channels.routing.send(table).await
    }

    /// Construct a set of channel accessors for allowing an endpoint access to the MIDI task.
    pub fn make_endpoint(&self, source: MIDISource) -> MIDIEndpoint {
        self.channels.make_endpoint(source)
    }
}
