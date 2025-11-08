use embedded_hal::digital::{InputPin, OutputPin};

use embassy_stm32::exti::ExtiInput;

use defmt::trace;

/// ST IC for power button management.
pub struct Stm6601<
    'a,
    // INT: InputPin, // Signals when power button is pressed.
    HOLD: OutputPin, // Hold HIGH to keep power enabled
    PBOUT: InputPin, // Current power button state
> {
    /// Signals when power button is pressed.
    pin_int: ExtiInput<'a>,
    /// Hold HIGH to keep power enabled
    pin_hold: HOLD,
    /// Current power button state
    pin_pbout: PBOUT,
}

impl<
    'a,
    HOLD: OutputPin, // Hold HIGH to keep power enabled
    PBOUT: InputPin, // Current power button state
> Stm6601<'a, HOLD, PBOUT>
{
    pub fn new(
        // Signals when power button is pressed.
        pin_int: ExtiInput<'a>,
        // Hold HIGH to keep power enabled
        pin_hold: HOLD,
        // Current power button state
        pin_pbout: PBOUT,
    ) -> Self {
        Self {
            pin_int,
            pin_hold,
            pin_pbout,
        }
    }

    /// Sets the STM6601 PS_HOLD pin to LOW which disables the 3v3 regulator.
    pub fn power_disable(&mut self) -> Result<(), HOLD::Error> {
        trace!("disabling device power");
        self.pin_hold.set_low()
    }

    /// Sets the PS_HOLD pin HIGH to keep the power controlled enabled.
    pub fn power_enable(&mut self) -> Result<(), HOLD::Error> {
        trace!("enabling device power hold");
        self.pin_hold.set_high()
    }

    /// Reads the current power button state.
    pub fn button_pressed(&mut self) -> Result<bool, PBOUT::Error> {
        self.pin_pbout.is_high()
    }

    /// Reads the current power button state.
    pub fn button_released(&mut self) -> Result<bool, PBOUT::Error> {
        self.pin_pbout.is_low()
    }

    pub async fn wait_for_button_press(&mut self) {
        self.pin_int.wait_for_falling_edge().await
    }
}
