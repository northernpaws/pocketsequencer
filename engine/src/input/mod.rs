use crate::prelude::*;

/// Encapsulates a specific input, such as a button.
pub trait Input {}

pub struct InputEvent<I: Input> {
    pub input: I,
    pub pressed: bool,
}

/// An input context receives input events, typically
/// tied to a specific UI screen or context.
pub trait Context<I: Input> {
    /// Called when the context is active
    /// and there's an input events.
    fn on_input(&mut self, input: I);
}

/// Driver provides the layer between
/// the input system and the hardware.
pub trait Driver<I: Input> {
    /// Poll for new input event(s).
    fn poll(&mut self) -> impl Future<Output = Result<InputEvent<I>, ()>> + Send;
}

pub struct Manager<I: Input, D: Driver<I>> {
    i: PhantomData<I>,

    driver: D,
}

impl<I: Input, D: Driver<I>> Manager<I, D> {
    pub fn new(driver: D) -> Self {
        Self {
            i: PhantomData::default(),
            driver,
        }
    }
}
