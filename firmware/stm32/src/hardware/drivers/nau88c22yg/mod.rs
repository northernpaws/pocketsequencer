// pub mod register;

use embedded_hal_async::i2c::SevenBitAddress;

pub const ADDRESS: SevenBitAddress = 0b0011010;

pub struct Nau88c22yg<I2C: embedded_hal_async::i2c::I2c> {
    /// I2C device on the bus.
    device: I2C,
    address: SevenBitAddress,
}

impl<I2C: embedded_hal_async::i2c::I2c> Nau88c22yg<I2C> {
    /// Constructs a new TCA8418 driver.
    pub fn new(device: I2C) -> Self {
        Self {
            device,
            address: ADDRESS,
        }
    }
}
