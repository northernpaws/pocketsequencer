#![cfg_attr(not(feature = "std"), no_std)]

#[cfg(all(not(feature = "std"), feature = "alloc"))]
extern crate alloc;

// MUST be the first module listed
mod fmt;

pub mod prelude;

pub mod input;

use prelude::*;

pub struct Engine <
    I: input::Input,
    InputDriver: input::Driver<I>
>{
    i: PhantomData<I>,
    input_manager: input::Manager<I, InputDriver>
}

impl<
    I: input::Input,
    InputDriver: input::Driver<I>
> Engine<I, InputDriver> {
    pub fn new(
        input_driver: InputDriver
    ) -> Self {
        Self {
            i: PhantomData::default(),
            input_manager: input::Manager::new(input_driver)
        }
    }

    pub async fn start (&mut self) {
        info!("Starting engine...");
    }
}
