use catalina::engine::audio::frame;

pub struct AudioEngine {}

impl AudioEngine {
    pub fn new() -> Self {
        Self {}
    }

    pub fn render(&mut self) -> frame::Stereo<f32> {
        [0.0, 0.0]
    }
}
