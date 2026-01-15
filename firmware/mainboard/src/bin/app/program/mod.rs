use alloc::string::ToString;
use catalina::engine::audio::Sample;
use defmt::info;
use embassy_executor::{SpawnError, Spawner};
use wavv::{Data, Wav};

use crate::engine::{Engine, audio::render::HALF_DMA_BUFFER_LENGTH};

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
            Ok(wav) => {
                info!("  channels={}", wav.fmt.num_channels);
                info!("  bit_depth={}bit", wav.fmt.bit_depth);
                info!("  sample_rate={}hz", wav.fmt.sample_rate);

                match wav.data {
                    Data::BitDepth8(samples) => {
                        info!("decoded valid 8 bit WAV file");
                    }
                    Data::BitDepth16(samples) => {
                        info!("decoded valid 16 bit WAV file");

                        // Chunk the audio file based on the buffer size of the transfer channel.
                        for chunk in samples.chunks(HALF_DMA_BUFFER_LENGTH) {
                            // Wait for the next free audio transfer buffer.
                            let buffer = engine.audio.system_audio.send().await;

                            // Clear the previous items.
                            buffer.clear();

                            // Break the samples chunk into alternating channels,
                            // and write the to the transfer buffer.
                            // TODO: could probably be a memcpy instead
                            for frame in chunk.chunks(2) {
                                let l_sample_int: i16 = frame[0];
                                let l_sample_float: f32 = l_sample_int as f32;
                                let l_sample: f32 = l_sample_float / 32_768.0f32;

                                let r_sample_int: i16 = frame[1];
                                let r_sample_float: f32 = r_sample_int as f32;
                                let r_sample: f32 = r_sample_float / 32_768.0f32;

                                // .to_sample() handles the conversion from the source
                                // sample type to the destination sample type, including
                                // scalining based on the ranges.
                                buffer.push([l_sample, r_sample]).unwrap();
                            }

                            // Signal that the buffer has been filled and can be played.
                            engine.audio.system_audio.send_done();

                            // info!(
                            //     "sent {} samples to system audio channel",
                            //     HALF_DMA_BUFFER_LENGTH
                            // );
                        }
                    }
                    Data::BitDepth24(samples) => {
                        info!("decoded valid 24 bit WAV file");
                    }
                }
            }
            Err(err) => {
                info!("WAV file appears invalid: {}", defmt::Debug2Format(&err));
            }
        }

        // Zero out both buffers.
        let buffer = engine.audio.system_audio.send().await;
        buffer.clear();
        for _i in 0..buffer.capacity() {
            buffer.push([f32::EQUILIBRIUM, f32::EQUILIBRIUM]).unwrap();
        }
        engine.audio.system_audio.send_done();

        let buffer = engine.audio.system_audio.send().await;
        buffer.clear();
        for _i in 0..buffer.capacity() {
            buffer.push([f32::EQUILIBRIUM, f32::EQUILIBRIUM]).unwrap();
        }
        engine.audio.system_audio.send_done();

        info!("startup sound finished");
    } else {
        info!("didn't find startup.wav, skipping");
    }

    Ok(())
}
