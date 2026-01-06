use defmt::{info, trace};
use embassy_executor::{SpawnError, Spawner};

use crate::engine::midi::MIDIEventSender;

use super::MIDIEventReceiver;

pub fn start_midi(
    spawner: Spawner,
    midi_event_rx: MIDIEventReceiver<'static>,
    midi_event_tx: MIDIEventSender<'static>,
) -> Result<(), SpawnError> {
    info!("spawning MIDI task...");
    spawner.spawn(midi_task(midi_event_rx, midi_event_tx)?);

    Ok(())
}

#[embassy_executor::task]
pub async fn midi_task(
    midi_event_rx: MIDIEventReceiver<'static>,
    midi_event_tx: MIDIEventSender<'static>,
) -> ! {
    // should never return
    let err = inner_midi_task(midi_event_rx, midi_event_tx).await;
    panic!("{:?}", err);
}

#[derive(Debug)]
enum Never {}

async fn inner_midi_task(
    midi_event_rx: MIDIEventReceiver<'_>,
    midi_event_tx: MIDIEventSender<'_>,
) -> Result<(), Never> {
    loop {
        // Wait for the next MIDI event.
        let event = midi_event_rx.receive().await;
        trace!("midi: received event ");
    }
}
