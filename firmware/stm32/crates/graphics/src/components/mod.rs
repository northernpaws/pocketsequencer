pub trait Component {
    fn draw<D: DrawTarget>(&mut self, target: D);
}
