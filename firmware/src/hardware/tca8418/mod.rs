pub mod register;

use defmt::trace;

use embassy_stm32::exti::ExtiInput;

use embedded_hal_async::i2c::SevenBitAddress;

const DEFAULT_ADDRESS: SevenBitAddress = 0b0110100;

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
    _int: ExtiInput<'a>,

    /// I2C device on the bus.
    device: I2C,

    write_buf: [u8; 2],
    write_read_buf: [u8; 1],
    read_buf: [u8; 1],

    address: SevenBitAddress,
}

impl<'a, I2C: embedded_hal_async::i2c::I2c> Tca8418<'a, I2C> {
    /// Constructs a new TCA8418 driver.
    pub fn new (
        int: ExtiInput<'a>,
        device: I2C,
    ) -> Self {
        Self{
            _int: int,
            device,
            address: DEFAULT_ADDRESS,
            write_buf: [0u8; 2],
            write_read_buf: [0u8; 1],
            read_buf: [0u8; 1]
        }
    }

    /// Initializes the TCA8418 with the
    /// register configurations for the device.
    ///
    /// ROW4-ROW7 is configured as MENU3-MENU0 with pull-ups.
    /// 
    /// ROW0-ROW3 is configured as the matrix rows.
    /// COL0 - COL3 are configured as columns 3-9 (inverted).
    pub async fn init (&mut self) -> Result<(), I2C::Error> {
        trace!("configuring TCA8418");

        { // Enable interrupts for the pins in use.
            let gpio_int_en_1 = register::GPIOInterruptEnable1::new(
                // Enable interrupts for the menu buttons.
                register::GPIOInterruptEnable::Enabled, // ROW7
                register::GPIOInterruptEnable::Enabled, // ROW6
                register::GPIOInterruptEnable::Enabled, // ROW5
                register::GPIOInterruptEnable::Enabled, // ROW4
                register::GPIOInterruptEnable::Disabled,
                register::GPIOInterruptEnable::Disabled,
                register::GPIOInterruptEnable::Disabled,
                register::GPIOInterruptEnable::Disabled,
            );

            trace!("enabling interrupts for menu buttons");
            self.write_register(gpio_int_en_1).await?;
        }

        { // Enable FIFO for the MENU GPIO buttons.
            let gpi_event_mode_1 = register::GPIEventMode1::new(
                register::GPIEventMode::FIFOEnabled, // ROW7
                register::GPIEventMode::FIFOEnabled, // ROW6
                register::GPIEventMode::FIFOEnabled, // ROW5
                register::GPIEventMode::FIFOEnabled, // ROW4
                register::GPIEventMode::FIFODisabled,
                register::GPIEventMode::FIFODisabled,
                register::GPIEventMode::FIFODisabled,
                register::GPIEventMode::FIFODisabled,
            );

            trace!("enabling FIFO for menu buttons");
            self.write_register(gpi_event_mode_1).await?;
        }

        { // Configure the keypad matrix.
            let kp_gpio_1 = register::KeypadGPIO1::new(
                register::GPIOMode::GPIOMode,
                register::GPIOMode::GPIOMode,
                register::GPIOMode::GPIOMode,
                register::GPIOMode::GPIOMode,
                // Configure the rows for the keypad matrix.
                register::GPIOMode::KeyScanMode, // ROW3
                register::GPIOMode::KeyScanMode, // ROW2
                register::GPIOMode::KeyScanMode, // ROW1
                register::GPIOMode::KeyScanMode, // ROW0
            );

            let kp_gpio_2 = register::KeypadGPIO2::new(
                register::GPIOMode::GPIOMode,
                register::GPIOMode::GPIOMode,
                register::GPIOMode::GPIOMode,
                register::GPIOMode::GPIOMode,
                // Configure the columns for the keypad matrix.
                register::GPIOMode::KeyScanMode, // COL3
                register::GPIOMode::KeyScanMode, // COL2
                register::GPIOMode::KeyScanMode, // COL1
                register::GPIOMode::KeyScanMode, // COL0
            );

            trace!("enabling keyscan for keypad matrix");
            self.write_register(kp_gpio_1).await?;
            self.write_register(kp_gpio_2).await?;
        }

        Ok(())
    }

    /// Reads the value of a register.
    pub async fn read_register<REG: register::Register>(&mut self) -> Result<REG, I2C::Error> {
        // Writes the register address to read, and
        // then reads the responding register values.
        self.write_read_buf[0] = REG::ADDRESS;
        self.device.write_read(self.address, self.write_buf.as_ref(), &mut self.read_buf).await?;

        Ok(REG::exact_from(self.read_buf[0]))
    }

    /// Writes a new value for a register.
    pub async fn write_register<REG: register::Register>(&mut self, reg: REG) -> Result<(), I2C::Error> {
        self.write_buf[0] = REG::ADDRESS;
        self.write_buf[1] = reg.into();
        self.device.write(self.address, self.write_buf.as_ref()).await
    }
}
