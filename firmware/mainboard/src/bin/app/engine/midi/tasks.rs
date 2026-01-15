use defmt::{info, trace};
use embassy_executor::{SpawnError, Spawner};
use embassy_futures::select::{Either, select};

use crate::engine::midi::{
    MIDIDestination, MIDIDestinations, MIDIRoutingReceiver, MIDIRoutingTable, MIDISources,
};

/// Spawns the MIDI handling task.
pub fn spawn(
    spawner: Spawner,

    routing_receiver: MIDIRoutingReceiver<'static>,
    // Table of channel receivers for receiving messages from MIDI endpoints.
    sources: MIDISources,
    // Table of channel senders to send MIDI messages over endpoints.
    destinations: MIDIDestinations,
) -> Result<(), SpawnError> {
    info!("spawning MIDI task...");
    spawner.spawn(midi_task(routing_receiver, sources, destinations)?);

    Ok(())
}

#[embassy_executor::task]
pub async fn midi_task(
    routing_receiver: MIDIRoutingReceiver<'static>,
    sources: MIDISources,
    destinations: MIDIDestinations,
) -> ! {
    // should never return
    let err = inner_midi_task(routing_receiver, sources, destinations).await;
    panic!("midi task exited unexpectedly: {:?}", err);
}

#[derive(Debug)]
enum Never {}

async fn inner_midi_task(
    // Receives routing table updates.
    routing_receiver: MIDIRoutingReceiver<'static>,

    // Receives messages from their sources.
    mut sources: MIDISources,

    // Routes messages to their appropriate destination interface.
    mut destinations: MIDIDestinations,
) -> Result<(), Never> {
    // Holds the currently active MIDI routing table.
    let mut table = MIDIRoutingTable::default();

    loop {
        // Wait for the next MIDI event.
        match select(sources.next(), routing_receiver.receive()).await {
            Either::First(event) => {
                trace!("midi: received event from {}", event.source);

                // TODO: route the MIDI message

                // Determine the destination from the routing table.
                let destination: MIDIDestination = MIDIDestination::Usb { channel: 0 };

                // Route the message to it's destination.
                destinations.route_message(event.message, destination).await;
            }
            Either::Second(new_table) => {
                trace!("midi: received routing table update");

                // Apply the new MIDI routing table.
                table = new_table;
            }
        }
    }
}
