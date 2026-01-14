use alloc::string::ToString;
use defmt::info;
use embassy_executor::{SpawnError, Spawner};

mod riff;

use crate::engine::Engine;

/// Starts the main program for running the device.
pub async fn start(spawner: Spawner, mut engine: Engine) -> Result<(), SpawnError> {
    // Create a drive interface handle.
    let mut drive = engine.drive.create_interface().await.unwrap();

    // Attempt to read a startup sound.
    //
    // This reads the sound into the SDRAM on the heap.
    info!("searching for startup.wav");
    if let Ok(start_sound) = drive.read_file("startup.wav".to_string()).await {
        info!("found startup.wav, decoding {} bytes", start_sound.len());

        // Attempt to decode the WAVE header.
        match riff::read_header(start_sound[0..44].try_into().unwrap()).await {
            Ok(_) => {
                info!("decoded valid WAV file");
            }
            Err(err) => {
                info!("WAV file appears invalid: {}", err);
            }
        }

        // TODO: play the startup sound
    } else {
        info!("didn't find startup.wav, skipping");
    }

    Ok(())
}
