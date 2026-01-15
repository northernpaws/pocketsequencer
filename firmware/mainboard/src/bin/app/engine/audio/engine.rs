use alloc::vec::Vec;
use catalina::engine::audio::frame;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, zerocopy_channel};
use static_cell::StaticCell;

// The data type that is exchanged via the zero-copy channel (a sample vector).
pub type SampleBlock = heapless::Vec<frame::Stereo<u32>, 128>; // USB_MAX_SAMPLE_COUNT

pub struct AudioEngine {}

impl AudioEngine {
    pub fn new() -> Self {
        Self {}
    }

    pub fn render(&mut self) -> frame::Stereo<f32> {
        [0.0, 0.0]
    }
}
