use defmt::{info, trace};
use embassy_executor::{SpawnError, Spawner};
use embassy_futures::select::{Either, select};

use crate::engine::midi::{MIDIEventSender, MIDIRoutingChannel, MIDIRoutingReceiver};

use super::MIDIEventReceiver;

pub fn start_midi(
    spawner: Spawner,
    midi_event_rx: MIDIEventReceiver<'static>,
    midi_event_tx: MIDIEventSender<'static>,
    routing_receiver: MIDIRoutingReceiver<'static>,
) -> Result<(), SpawnError> {
    info!("spawning MIDI task...");
    spawner.spawn(midi_task(midi_event_rx, midi_event_tx, routing_receiver)?);

    Ok(())
}

#[embassy_executor::task]
pub async fn midi_task(
    midi_event_rx: MIDIEventReceiver<'static>,
    midi_event_tx: MIDIEventSender<'static>,
    routing_receiver: MIDIRoutingReceiver<'static>,
) -> ! {
    // should never return
    let err = inner_midi_task(midi_event_rx, midi_event_tx, routing_receiver).await;
    panic!("midi task exited unexpectedly: {:?}", err);
}

#[derive(Debug)]
enum Never {}

async fn inner_midi_task(
    midi_event_rx: MIDIEventReceiver<'_>,
    midi_event_tx: MIDIEventSender<'_>,

    // Receives routing table updates.
    routing_receiver: MIDIRoutingReceiver<'static>,
) -> Result<(), Never> {
    loop {
        // Wait for the next MIDI event.
        let event = midi_event_rx.receive();
        let table = routing_receiver.receive();

        match select(event, table).await {
            Either::First(event) => {
                trace!("midi: received event from {}", event.source);
            }
            Either::Second(table) => {
                trace!("midi: received routing table update");
            }
        }
    }
}
