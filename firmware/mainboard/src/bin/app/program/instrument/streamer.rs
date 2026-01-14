use catalina::engine::{
    audio::{AudioSource, Stereo, signal::Signal},
    instrument::Instrument,
};
use embassy_sync::{
    blocking_mutex::raw::{NoopRawMutex, ThreadModeRawMutex},
    pipe::Pipe,
};

/// A type of sampler that streams audio directly from the SD card.
///
/// This offers limited options for sample manipulation, but doesn't
/// require storage in a memory buffer and is faster for long samples,
/// such as backing tracks.
pub struct Streamer {
    path: heapless::String<255>,
    playing: bool,
    pipe: Pipe<ThreadModeRawMutex, 256>,
}

impl Streamer {
    pub fn new(path: heapless::String<255>) -> Self {
        Self {
            path,
            playing: false,
            pipe: Pipe::new(),
        }
    }

    pub fn set_sample(path: heapless::String<255>) -> Result<(), ()> {
        Ok(())
    }
}

// TODO: not sure signal is actually useful on the instrument
// interace, since most rendering is done in blocks..
impl Signal for Streamer {
    type Frame = Stereo<f32>;

    fn next(&mut self) -> Self::Frame {
        todo!()
    }
}

impl AudioSource for Streamer {
    type Frame = Stereo<f32>;

    fn render(&mut self, buffer: &'_ mut [Self::Frame]) {}
}

impl Instrument for Streamer {
    fn init(&mut self) {
        // no-op
    }

    fn note_on(
        &mut self,
        note: catalina::engine::music::note::Note,
        velocity: u8,
    ) -> Result<(), catalina::engine::instrument::NoteError> {
        self.playing = true;
        Ok(()) // no-op
    }

    fn note_off(&mut self, note: catalina::engine::music::note::Note) {
        self.playing = false;
        // no-op
    }
}
