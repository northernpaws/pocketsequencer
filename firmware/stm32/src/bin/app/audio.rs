use crate::hardware::audio::Audio;
use catalina::engine::audio::oscillator;
use catalina::engine::audio::oscillator::Oscillator;
use catalina::engine::core::Hertz;
use cortex_m_rt::interrupt;
use defmt::info;
use embassy_executor::{InterruptExecutor, SpawnError};
use embassy_stm32::interrupt::InterruptExt;
use embassy_stm32::interrupt::{self, Priority};
use grounded::uninit::GroundedArrayCell;

// Note that the block size needs to be big enough to where the DMA/SAI
// has enough to work with before the next cycle to generate more.
pub const BLOCK_LENGTH: usize = 128; // samples
const OUTPUT_CHANNEL_COUNT: usize = 2; // stereo
pub const HALF_DMA_BUFFER_LENGTH: usize = (BLOCK_LENGTH) * OUTPUT_CHANNEL_COUNT; //  2 channels
pub const DMA_BUFFER_LENGTH: usize = HALF_DMA_BUFFER_LENGTH * 2; //  2 half-blocks

static AUDIO_EXECUTOR: InterruptExecutor = InterruptExecutor::new();

// NOTE: Use of a GroundedArrayCell here is very important! If we used something
//  else like a StaticCell, the program would attempt to allocate the entire
//  buffer on the stack to initialize the static memory.
static AUDIO_TX_BUFFER: GroundedArrayCell<u32, { DMA_BUFFER_LENGTH }> = GroundedArrayCell::uninit();
static AUDIO_RX_BUFFER: GroundedArrayCell<u32, { DMA_BUFFER_LENGTH }> = GroundedArrayCell::uninit();

#[interrupt]
unsafe fn SAI1() {
    unsafe { AUDIO_EXECUTOR.on_interrupt() }
}

/// Spawns the task for audio playback using a high-priority interrupt based executor.
///
/// We opt for using an interrupt executor over the standard Embassy task executor
/// so we can run the audio at a higher priority. If the audio task gets starved it
/// is very audibly noticable vs starving something like input or display.
pub fn start_audio(audio: Audio<'static>) -> Result<(), SpawnError> {
    // Use an interrupt executor to run the audio task at a higher priority.
    interrupt::SAI1.set_priority(Priority::P6);
    let spawner = AUDIO_EXECUTOR.start(interrupt::SAI1);
    spawner.spawn(audio_task(audio)?);

    Ok(())
}

#[embassy_executor::task]
pub async fn audio_task(audio: Audio<'static>) -> ! {
    // should never return
    let err = inner_audio_task(audio).await;
    panic!("{:?}", err);
}

#[derive(Debug)]
enum Never {}

async fn inner_audio_task(audio: Audio<'static>) -> Result<(), Never> {
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

    // Dummy audio source for testing
    let mut osc = oscillator::RuntimeOscillator::new(
        oscillator::OscillatorType::Sine,
        // NOTE: We pass in the actual sampling rate calculated from the SAI clock
        //  difference, instead of hard-coding our desired 48kHz sample rate because
        //  any clock skew will shift the frequency otherwise.
        audio.get_sample_rate() as usize,
        Hertz::from_hertz(261.63), // middle C
    );

    let mut buf: [u32; HALF_DMA_BUFFER_LENGTH] = [0u32; HALF_DMA_BUFFER_LENGTH];
    const AMPLITUDE: f32 = 0.25;
    loop {
        // Loop over each frame in the buffer and render
        // the samples for the left and right channels.
        for frame in buf.chunks_mut(OUTPUT_CHANNEL_COUNT) {
            // Output is 24-bit I2S data, so format
            // the oscillator sample to U24 sizes.
            let value: f32 = osc.sample(); // U24

            for sample in frame.iter_mut() {
                // Note that we use value.inner() instead of to_sample().
                //
                // to_sample() would convert to a u32 range, but the I2S
                // sample audio format is actually 24-bit so we want to
                // cap it at 24.
                // *sample = value.inner().scale_amp(0.25) as u32;
                *sample = (((value + 2.0) * (0xFFFFFF as f32 / 2.0)) * AMPLITUDE) as u32;
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
    }
}
