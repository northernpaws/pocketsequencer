use alloc::string::ToString;
use defmt::info;
use embassy_executor::{SpawnError, Spawner};
use wavv::{Data, Wav};

use crate::engine::Engine;

/// Starts the main program for running the device.
pub async fn start(spawner: Spawner, mut engine: Engine) -> Result<(), SpawnError> {
    // Create a drive interface handle.
    let mut drive = engine.drive.create_interface().await.unwrap();

    // Attempt to read a startup sound.
    //
    // This reads the sound into the SDRAM on the heap.
    let path = "startup.wav".to_string();
    info!("searching for {}", path);
    if let Ok(start_sound) = drive.read_file(path).await {
        info!("found startup.wav, decoding {} bytes", start_sound.len());

        match Wav::from_bytes(&start_sound) {
            Ok(wav) => match wav.data {
                Data::BitDepth8(samples) => {
                    info!("decoded valid 8 bit WAV file");
                }
                Data::BitDepth16(samples) => {
                    info!("decoded valid 16 bit WAV file");
                }
                Data::BitDepth24(samples) => {
                    info!("decoded valid 24 bit WAV file");
                }
            },
            Err(err) => {
                info!("WAV file appears invalid: {}", defmt::Debug2Format(&err));
            }
        }

        // TODO: play the startup sound
    } else {
        info!("didn't find startup.wav, skipping");
    }

    Ok(())
}
