use crate::engine::audio::engine::AudioEngine;
use crate::hardware::audio::Audio;

use cortex_m_rt::interrupt;
use defmt::{error, info};
use embassy_executor::{InterruptExecutor, SpawnError};
use embassy_stm32::interrupt::InterruptExt;
use embassy_stm32::interrupt::{self, Priority};
use firmware::hardware::CodecSAIResources;
use grounded::uninit::GroundedArrayCell;

use catalina::engine::{
    audio::oscillator::{self},
    core::Hertz,
};

// Note that the block size needs to be big enough to where the DMA/SAI
// has enough to work with before the next cycle to generate more.
pub const BLOCK_LENGTH: usize = 128; // samples
const OUTPUT_CHANNEL_COUNT: usize = 2; // stereo
pub const HALF_DMA_BUFFER_LENGTH: usize = (BLOCK_LENGTH) * OUTPUT_CHANNEL_COUNT; //  2 channels
pub const DMA_BUFFER_LENGTH: usize = HALF_DMA_BUFFER_LENGTH * 2; //  2 half-blocks

// Stuttering or drops in the framerate of the display or input isn't as
// imeddiatly noticable as stuttering or delay in audio, so we use an
// interrupt executor so we can run it at a higher priority then other
// tasks, making sure that audio is as consistent as possible.
static AUDIO_EXECUTOR: InterruptExecutor = InterruptExecutor::new();

// NOTE: Use of a GroundedArrayCell here is very important! If we used something
//  else like a StaticCell, the program would attempt to allocate the entire
//  buffer on the stack to initialize the static memory.
static AUDIO_TX_BUFFER: GroundedArrayCell<u32, { DMA_BUFFER_LENGTH }> = GroundedArrayCell::uninit();
static AUDIO_RX_BUFFER: GroundedArrayCell<u32, { DMA_BUFFER_LENGTH }> = GroundedArrayCell::uninit();

static FRAME_BUFFER: GroundedArrayCell<u32, { HALF_DMA_BUFFER_LENGTH }> =
    GroundedArrayCell::uninit();

// Use the SAI1 interrupt for the audio executor.
//
// Because we're using DMA with SAI1, we don't actually
// need to rely on the SAI1 interrupt for anything, making
// it a good candidate for the audio executor.
#[interrupt]
unsafe fn SAI1() {
    unsafe { AUDIO_EXECUTOR.on_interrupt() }
}

/// Spawns the task for audio playback using a high-priority interrupt based executor.
///
/// We opt for using an interrupt executor over the standard Embassy task executor
/// so we can run the audio at a higher priority. If the audio task gets starved it
/// is very audibly noticable vs starving something like input or display.
pub fn spawn_task(
    audio: Audio,
    r: CodecSAIResources,
    engine: AudioEngine,
) -> Result<(), SpawnError> {
    // Use an interrupt executor to run the audio task at a higher priority.
    interrupt::SAI1.set_priority(Priority::P6);
    let spawner = AUDIO_EXECUTOR.start(interrupt::SAI1);
    spawner.spawn(audio_task(audio, r, engine)?);

    Ok(())
}

#[embassy_executor::task]
pub async fn audio_task(audio: Audio, r: CodecSAIResources, engine: AudioEngine) -> ! {
    // should never return
    let err = inner_audio_task(audio, r, engine).await;
    panic!("audio task exited unexpectedly: {:?}", err);
}

#[derive(Debug)]
enum Never {}

async fn inner_audio_task(
    mut audio: Audio,
    mut r: CodecSAIResources,
    mut engine: AudioEngine,
) -> Result<(), Never> {
    info!("starting audio task");

    info!("initializing audio buffers...");

    // Zero-out the static buffer for transmitting audio to the SAI.
    let audio_tx_buffer: &mut [u32] = unsafe {
        AUDIO_TX_BUFFER.initialize_all_copied(0);
        let (ptr, len) = AUDIO_TX_BUFFER.get_ptr_len();
        core::slice::from_raw_parts_mut(ptr, len)
    };

    // Zero-out the static buffer for receiving audio to the SAI.
    let audio_rx_buffer: &mut [u32] = unsafe {
        AUDIO_RX_BUFFER.initialize_all_copied(0);
        let (ptr, len) = AUDIO_RX_BUFFER.get_ptr_len();
        core::slice::from_raw_parts_mut(ptr, len)
    };

    // Initialize the SAI transmitter and receiver blocks.
    //
    // This also initializes the codec if it's the first call to `create_sai`.
    let (mut sai_transmitter, mut sai_receiver) = audio
        .create_sai(&mut r, audio_tx_buffer, audio_rx_buffer)
        .await;

    // Dummy audio source for testing
    let mut osc = oscillator::RuntimeOscillator::new(
        oscillator::OscillatorType::Sine,
        // NOTE: We pass in the actual sampling rate calculated from the SAI clock
        //  difference, instead of hard-coding our desired 48kHz sample rate because
        //  any clock skew will shift the frequency otherwise.
        audio.get_sample_rate() as usize,
        Hertz::from_hertz(261.63), // middle C
    );

    // Zero-out the frame block buffer for incremental writes to the transmit ringbuffer.
    let buf: &mut [u32] = unsafe {
        FRAME_BUFFER.initialize_all_copied(0);
        let (ptr, len) = FRAME_BUFFER.get_ptr_len();
        core::slice::from_raw_parts_mut(ptr, len)
    };

    const AMPLITUDE: f32 = 0.25;
    loop {
        // Loop over each frame in the buffer and render
        // the samples for the left and right channels.
        for frame in buf.chunks_mut(OUTPUT_CHANNEL_COUNT) {
            // Output is 24-bit I2S data, so format
            // the oscillator sample to U24 sizes.
            // let value: f32 = osc.sample(); // U24

            // for sample in frame.iter_mut() {
            //     // Note that we use value.inner() instead of to_sample().
            //     //
            //     // to_sample() would convert to a u32 range, but the I2S
            //     // sample audio format is actually 24-bit so we want to
            //     // cap it at 24.
            //     // *sample = value.inner().scale_amp(0.25) as u32;
            //     *sample = (((value + 2.0) * (0xFFFFFF as f32 / 2.0)) * AMPLITUDE) as u32;
            // }

            let value = engine.render();
            for (i, sample) in frame.iter_mut().enumerate() {
                // Convert the sample to u24 encoded as u32 frames.
                *sample =
                    (((value[i % value.len()] + 2.0) * (0xFFFFFF as f32 / 2.0)) * AMPLITUDE) as u32;
            }
        }

        // Write the rendered buffer to the larger ring buffer for DMA.
        //
        // Writing to the SAI device blocks until the write is complete,
        // at which point we known that we need to render and write more
        // while the DMA finishes transferring the written block.
        //
        // If an overrun error occurs, that means that we're not writing
        // fast enough in this loop to keep up with DMA.
        // A write() must be called before read() to start the
        // master (transmitter) clock used by the receiver.
        match sai_transmitter.write(&buf).await {
            Ok(_) => {}
            Err(err) => {
                error!("Audio transmit error: {}", err);
            }
        }
    }
}
