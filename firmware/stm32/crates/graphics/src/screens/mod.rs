use embedded_graphics_core::draw_target::DrawTarget;

pub trait Screen {
    fn draw<D: DrawTarget>(&mut self, target: D);
}

pub struct HomeScreen {}

impl Screen for HomeScreen {
    fn draw<D: DrawTarget>(&mut self, target: D) {
        let display_area = target.bounding_box();
    }
}
