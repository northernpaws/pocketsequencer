pub mod register;

use defmt::{trace, warn};

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
    Unknown,

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

        let configuration_register = register::Configuration::new(
            true, // auto increment for read-write operations
            false, // GPI evens tracked when keypad locked
            false, // overflowed data is lost
            true, // IC will pulse INT on new interrupts
            false, // don't assert INT on overflow
            false, // don't assert interrupt after keypad unlock
            true, // assert INT for GPI events
            true, // assert INT for keypad events
        );
        self.write_register(configuration_register).await?;

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

            // "Once the TCA8418 has had the keypad array configured, it will enter idle mode when no keys are being pressed.
            // All columns configured as part of the keypad array will be driven low and all rows configured as part of the
            // keypad array will be set to inputs, with pull-up resistors enabled. During idle mode, the internal oscillator is turned
            // off so that power consumption is low as the device awaits a key press."
            //
            // TODO: think rows and columns are configured in reverse, will need to swap in the software config.
        }

        Ok(())
    }

    /// Wait for the next interrupt and process it.
    pub async fn process(&mut self) -> Result<(), I2C::Error> {
        // INT is active-low.
        self._int.wait_for_falling_edge().await;
        
        // When the interrupt is triggered, we need to read
        // the interrupt status table to see what it was.
        let mut int_status: register::InterruptStatus = self.read_register().await?;

        // If there are logged events for the FIFO stack, then we need to read it.
        if int_status.key_event_interrupt_status().get() || int_status.gpi_interrupt_status().get() {
            // Process the key events in a loop to ensure we're catching any events
            // that came in while we where still processing the FIFO stack.
            loop {
                // Check the event counter fields to see how many key pressed are stored in the FIFO stack.
                let mut event_register: register::KeyLockAndEventCounter = self.read_register().await?;
                let key_events_count = event_register.key_event_count().get().value();
                trace!("found {} stored keypressed", defmt::Display2Format(&key_events_count));

                // Stop processing the FIFO stack if there are
                // no keypresses left in the status register.
                if key_events_count == 0 {
                    break;
                }

                for _n in 0..key_events_count {
                    // The KEY_EVENT_A (0x04) is the head of the FIFO stack.
                    //
                    // Each read of KEY_EVENT_A will deincrement the key event count
                    // by one, and move all the data in the stack down by one.
                    let mut key_event: register::KeyEventA = self.read_register().await?;

                    // Read the key index.
                    //
                    // A value of 0 to 80 indicate which key has been
                    // pressed or released in a keypad matrix.
                    // 
                    // Values of 97 to 114 are for GPI events.
                    //
                    // A value of 0 indicates that there are no more events in the FIFO stack.
                    let key_index = key_event.key_index().get().value();
                    if key_index == 0 {
                        // Warn because hitting this unessessarily will cause a per-frame performance hit.
                        warn!("somehow we read more key events then there where in the FIFO stack?");
                        break;
                    }

                    let key_pressed = key_event.key_pressed().get();

                    trace!("key {} pressed: {}", defmt::Display2Format(&key_index), key_pressed);

                    // Match the key event table index to it's corresponding matrix cell or GPI input.
                    // https://www.ti.com/lit/ds/symlink/tca8418.pdf?ts=1754456323928 (p.g. 15)
                    let button: Button = match key_index {
                        4 => Button::Trig1,
                        3 => Button::Trig2,
                        2 => Button::Trig3,
                        1 => Button::Trig4,
                        14 => Button::Trig5,
                        13 => Button::Trig6,
                        12 => Button::Trig7,
                        11 => Button::Trig8,
                        24 => Button::Trig9,
                        23 => Button::Trig10,
                        22 => Button::Trig11,
                        21 => Button::Trig12,
                        34 => Button::Trig13,
                        33 => Button::Trig14,
                        32 => Button::Trig15,
                        31 => Button::Trig16,

                        // GPIs are represented with decimal value of 97 and run through decimal value of 114.
                        // R0-R7 are represented by 97-104 and C0-C9 are represented by 105-114.
                        //
                        // R0 R1 R2 R3  R4  R5  R6  R7
                        // 97 98 99 100 101 102 103 104
                        101 => Button::Menu3,
                        102 => Button::Menu2,
                        103 => Button::Menu1,
                        104 => Button::Menu0,

                        _ => Button::Unknown,  
                    };

                    // TODO: emit some sort of button event
                } 
            }
        }
        
        self._int.wait_for_rising_edge().await;

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
