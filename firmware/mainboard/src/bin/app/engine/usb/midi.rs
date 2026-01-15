use alloc::vec::Vec;
use defmt::info;
use midly::{MidiMessage, live::LiveEvent, num::u7};

use crate::engine::midi::{MIDIMessage, SystemCommon};

/// Converts a midly event into a MIDI message that's compatible with our channel format.
///
/// We can't use midly's type directly because it relies on lifetimes we can't fulfill ovr pubsub channels.
pub fn decode_midi_message(event: LiveEvent<'_>) -> MIDIMessage {
    match event {
        LiveEvent::Midi { channel, message } => MIDIMessage::Midi { channel, message },
        LiveEvent::Common(system_common) => match system_common {
            midly::live::SystemCommon::SysEx(u7s) => {
                MIDIMessage::Common(SystemCommon::SysEx(u7s.to_vec().into_boxed_slice()))
            }
            midly::live::SystemCommon::MidiTimeCodeQuarterFrame(mtc_quarter_frame_message, u4) => {
                MIDIMessage::Common(SystemCommon::MidiTimeCodeQuarterFrame(
                    mtc_quarter_frame_message,
                    u4,
                ))
            }
            midly::live::SystemCommon::SongPosition(u14) => {
                MIDIMessage::Common(SystemCommon::SongPosition(u14))
            }
            midly::live::SystemCommon::SongSelect(a) => {
                MIDIMessage::Common(SystemCommon::SongSelect(a))
            }
            midly::live::SystemCommon::TuneRequest => {
                MIDIMessage::Common(SystemCommon::TuneRequest)
            }
            midly::live::SystemCommon::Undefined(n, u7s) => {
                MIDIMessage::Common(SystemCommon::Undefined(n, u7s.to_vec().into_boxed_slice()))
            }
        },
        LiveEvent::Realtime(system_realtime) => MIDIMessage::Realtime(system_realtime),
    }
}

/// Converts a MIDI message into a Midly event that can be written as a MIDI packet.
pub fn encode_midi_message<'a>(
    message: MIDIMessage,
) -> Result<Vec<u8>, <Vec<u8> as midly::io::Write>::Error> {
    let mut data: Vec<u7> = Vec::new();
    let event: LiveEvent<'_> = match message {
        MIDIMessage::Midi { channel, message } => LiveEvent::Midi { channel, message },
        MIDIMessage::Common(system_common) => LiveEvent::Common(match system_common {
            SystemCommon::SysEx(u7s) => {
                data.resize(u7s.len(), 0.into());
                data.copy_from_slice(&u7s);
                midly::live::SystemCommon::SysEx(data.as_slice())
            }
            SystemCommon::MidiTimeCodeQuarterFrame(mtc_quarter_frame_message, u4) => {
                midly::live::SystemCommon::MidiTimeCodeQuarterFrame(mtc_quarter_frame_message, u4)
            }
            SystemCommon::SongPosition(u14) => midly::live::SystemCommon::SongPosition(u14),
            SystemCommon::SongSelect(a) => midly::live::SystemCommon::SongSelect(a),
            SystemCommon::TuneRequest => midly::live::SystemCommon::TuneRequest,
            SystemCommon::Undefined(a, u7s) => {
                data.resize(u7s.len(), 0.into());
                data.copy_from_slice(&u7s);
                midly::live::SystemCommon::Undefined(a, data.as_slice())
            }
        }),
        MIDIMessage::Realtime(system_realtime) => todo!(),
    };

    let mut buf: Vec<u8> = Vec::new();
    event.write(&mut buf)?;

    Ok(buf)
}

/// Logs a decoded MIDI event.
pub fn log_event(event: LiveEvent<'_>) {
    match event {
        LiveEvent::Midi { channel, message } => match message {
            MidiMessage::NoteOn { key, vel: _ } => info!(
                "MIDI: note on {} on channel {}",
                key.as_int(),
                channel.as_int()
            ),
            MidiMessage::NoteOff { key, vel: _ } => info!(
                "MIDI: note off {} on channel {}",
                key.as_int(),
                channel.as_int()
            ),
            MidiMessage::Aftertouch { key, vel: _ } => info!(
                "MIDI: aftertouch {} on channel {}",
                key.as_int(),
                channel.as_int()
            ),
            MidiMessage::Controller { controller, value } => info!(
                "MIDI: controller {}={} on channel {}",
                controller.as_int(),
                value.as_int(),
                channel.as_int()
            ),
            MidiMessage::ProgramChange { program } => info!(
                "MIDI: program {} on channel {}",
                program.as_int(),
                channel.as_int()
            ),
            MidiMessage::ChannelAftertouch { vel: _ } => {
                info!("MIDI: aftertouch on channel {}", channel.as_int())
            }
            MidiMessage::PitchBend { bend } => info!(
                "MIDI: pitch bend {} on channel {}",
                bend.0.as_int(),
                channel.as_int()
            ),
        },
        LiveEvent::Common(system_common) => match system_common {
            midly::live::SystemCommon::SysEx(_) => info!("MIDI: SYS: SysEx"),
            midly::live::SystemCommon::MidiTimeCodeQuarterFrame(_, _u4) => {
                info!("MIDI: SYS: quarter frame")
            }
            midly::live::SystemCommon::SongPosition(u14) => {
                info!("MIDI: SYS: song position: {}", u14.as_int())
            }
            midly::live::SystemCommon::SongSelect(a) => {
                info!("MIDI: SYS: song select: {}", a.as_int())
            }
            midly::live::SystemCommon::TuneRequest => info!("MIDI: SYS: tune request"),
            midly::live::SystemCommon::Undefined(_, _) => info!("MIDI: SYS: undefined"),
        },
        LiveEvent::Realtime(system_realtime) => match system_realtime {
            midly::live::SystemRealtime::TimingClock => {
                info!("MIDI: REALTIME: timing clock")
            }
            midly::live::SystemRealtime::Start => info!("MIDI: REALTIME: start"),
            midly::live::SystemRealtime::Continue => info!("MIDI: REALTIME: continue"),
            midly::live::SystemRealtime::Stop => info!("MIDI: REALTIME: stop"),
            midly::live::SystemRealtime::ActiveSensing => {
                info!("MIDI: REALTIME: active sensing")
            }
            midly::live::SystemRealtime::Reset => info!("MIDI: REALTIME: reset"),
            midly::live::SystemRealtime::Undefined(_) => {
                info!("MIDI: REALTIME: undefined message")
            }
        },
    }
}
