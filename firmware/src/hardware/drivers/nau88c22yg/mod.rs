pub mod register;

use defmt::{trace, warn};

use embassy_stm32::exti::ExtiInput;

use embedded_hal_async::i2c::SevenBitAddress;

const ADDRESS: SevenBitAddress = 0b0011010;

pub struct Nau88c22yg<'a, I2C: embedded_hal_async::i2c::I2c> {
    /// I2C device on the bus.
    device: I2C,
    address: SevenBitAddress,
}

impl<'a, I2C: embedded_hal_async::i2c::I2c> Nau88c22yg<'a, I2C> {
    /// Constructs a new TCA8418 driver.
    pub fn new (
        device: I2C,
    ) -> Self {
        Self{
            device,
            address: ADDRESS,
        }
    }

}