pub mod register;

use embassy_stm32::exti::ExtiInput;

use embedded_hal_async::i2c::SevenBitAddress;

pub const FUSB302BMPX_ADDRESS: SevenBitAddress = 0b0100010;

pub struct Fusb302b<'a, I2C: embedded_hal_async::i2c::I2c> {
    // Exti-bound input pin for the keypad interrupt signal.
    _int: ExtiInput<'a>,

    /// I2C device on the bus.
    ///
    /// FUSB302B supports Fast Mode
    /// Plus traffic up to 1 MHz SCL.
    /// Minimum clock speed is 400 kHz.
    device: I2C,

    address: SevenBitAddress,

    write_buf: [u8; 2],
    write_read_buf: [u8; 1],
    read_buf: [u8; 1],
}

impl<'a, I2C: embedded_hal_async::i2c::I2c> Fusb302b<'a, I2C> {
    /// Constructs a new TCA8418 driver.
    pub fn new(int: ExtiInput<'a>, device: I2C) -> Self {
        Self {
            _int: int,
            device,
            address: FUSB302BMPX_ADDRESS,

            write_buf: [0u8; 2],
            write_read_buf: [0u8; 1],
            read_buf: [0u8; 1],
        }
    }

    /// Reads the value of a register.
    pub async fn read_register<REG: register::Register>(&mut self) -> Result<REG, I2C::Error> {
        // Writes the register address to read, and
        // then reads the responding register values.
        self.write_read_buf[0] = REG::ADDRESS;
        self.device
            .write_read(self.address, self.write_buf.as_ref(), &mut self.read_buf)
            .await?;

        Ok(REG::from_storage(self.read_buf[0]))
    }

    /// Writes a new value for a register.
    pub async fn write_register<REG: register::Register>(
        &mut self,
        reg: REG,
    ) -> Result<(), I2C::Error> {
        self.write_buf[0] = REG::ADDRESS;
        self.write_buf[1] = reg.into_storage();
        self.device
            .write(self.address, self.write_buf.as_ref())
            .await
    }
}
