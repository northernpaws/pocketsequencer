pub mod register;

use embassy_stm32::{
    exti::ExtiInput, Config
};

const ADDRESS: u8 = 0b0110100;

/// Holds all the buttons on the device mapped
/// to the TCA8418RTWR keypad decoder.
/// 
/// ROW4-ROW7 is configured as MENU3-MENU0
/// 
/// ROW0-ROW3 is configured as the matrix rows
/// COL3 - COL3 are configured as columns 3-9 (inverted)
pub enum Button {
    // Standalone buttons
    Menu0,
    Menu1,
    Menu2,
    Menu3,

    // Matrix buttons
    Trig1,
    Trig2,
    Trig3,
    Trig4,
    Trig5,
    Trig6,
    Trig7,
    Trig8,
    Trig9,
    Trig10,
    Trig11,
    Trig12,
    Trig13,
    Trig14,
    Trig15,
    Trig16,
}

pub struct Tca8418<'a, I2C: embedded_hal_async::i2c::I2c> {
    // Exti-bound input pin for the keypad interrupt signal.
    int: ExtiInput<'a>,

    /// I2C device on the bus.
    device: I2C,

    read_buf: u8,
}

impl<'a, I2C: embedded_hal_async::i2c::I2c> Tca8418<'a, I2C> {
    /// Constructs a new TCA8418 driver.
    pub fn new (
        int: ExtiInput<'a>,
        device: I2C,
    ) -> Self {
        Self{
            int,
            device,
            read_buf: 0
        }
    }

    /// Initializes the TCA8418 with the
    /// register configurations for the device.
    ///
    /// ROW4-ROW7 is configured as MENU3-MENU0 with pull-ups.
    /// 
    /// ROW0-ROW3 is configured as the matrix rows.
    /// COL3 - COL3 are configured as columns 3-9 (inverted).
    pub fn init (&mut self) {
        
    }

    /// Reads the value of the specified register address.
    pub async fn read_register(&mut self, address: register::Address) -> Result<u8, I2C::Error> {
        self.device.read(address, &self.read_buf).await?;
        Ok(self.read_buf)
    }
}

