use alloc::boxed::Box;
use embassy_executor::Spawner;
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    channel::{Receiver, Sender},
};
use midly::{
    MidiMessage,
    live::{MtcQuarterFrameMessage, SystemRealtime},
    num::{u4, u7, u14},
};

mod tasks;

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
pub enum MIDIEvent {
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

pub type MIDIEventReceiver<'a> = Receiver<'a, CriticalSectionRawMutex, MIDIEvent, 4>;
pub type MIDIEventSender<'a> = Sender<'a, CriticalSectionRawMutex, MIDIEvent, 4>;

/// Starts the MIDI processing components.
pub fn start(
    spawner: Spawner,
    // Receives external MIDI events.
    midi_event_rx: MIDIEventReceiver<'static>,
    // Sends external MIDI events.
    midi_event_tx: MIDIEventSender<'static>,
) {
    // Start the tasks for MIDI handling.
    tasks::start_midi(spawner, midi_event_rx, midi_event_tx).unwrap();
}
