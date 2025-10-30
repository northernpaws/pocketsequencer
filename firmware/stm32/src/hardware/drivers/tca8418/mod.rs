pub mod register;

use defmt::{trace, warn, error, info};

use embassy_stm32::exti::ExtiInput;

use embassy_time::Timer;
use embedded_hal_async::i2c::SevenBitAddress;

use grounded::uninit::GroundedArrayCell;

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

// see: https://github.com/embassy-rs/embassy/blob/abcb6e607c4f13bf99c406fbb92480c32ebd0d4a/docs/pages/faq.adoc#stm32-bdma-only-working-out-of-some-ram-regions
// Defined in memory.x
#[unsafe(link_section = ".ram_d3")]
static mut RAM_D3_BUF: GroundedArrayCell<u8, 4> = GroundedArrayCell::uninit();

pub struct Tca8418<'a, I2C: embedded_hal_async::i2c::I2c> {
    // Exti-bound input pin for the keypad interrupt signal.
    _int: ExtiInput<'a>,

    /// I2C device on the bus.
    device: I2C,

    write_buf: &'a mut [u8], //[u8; 2],
    write_read_buf: &'a mut [u8], // [u8; 1],
    read_buf: &'a mut [u8], // [u8; 1],

    address: SevenBitAddress,
}

impl<'a, I2C: embedded_hal_async::i2c::I2c> Tca8418<'a, I2C> {
    /// Constructs a new TCA8418 driver.
    pub fn new (
        int: ExtiInput<'a>,
        device: I2C,
    ) -> Self {
        // The I2C4 peripherial on STM32H7 requires the BDMA, and in
        // turn the BDMA can only access data in the D3 section of RAM.
        let (write_buf, write_read_buf, read_buf) = unsafe {
            let ram = &mut *core::ptr::addr_of_mut!(RAM_D3_BUF);
            ram.initialize_all_copied(0);
            (
                ram.get_subslice_mut_unchecked(0, 2), // 0..1
                ram.get_subslice_mut_unchecked(2, 1), // 2
                ram.get_subslice_mut_unchecked(3, 1), // 3
            )
        };
        
        Self{
            _int: int,
            device,
            address: DEFAULT_ADDRESS,
            write_buf,
            write_read_buf,
            read_buf
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

        // Test that we're able to read back the value to make sure we're able to talk to the device.
        let initial_config: register::Configuration = self.read_register().await?;
        trace!("initial configuration: {:08b}", initial_config.raw());

        let configuration_register = register::Configuration::new(
            false, // auto increment for read-write operations
            false, // GPI evens tracked when keypad locked
            true, // overflow data shifts with last event pushing first event out
            true, //  processor interrupt is deasserted for 50 μs and reassert with pending interrupts
            true, // assert INT on overflow
            false, // don't assert interrupt after keypad unlock
            false, // assert INT for GPI events // TODO: change
            true, // assert INT for keypad events
        );
        self.write_register(configuration_register).await?;
        
        // Test that we're able to read back the value to make sure we're able to talk to the device.
        let actual_config: register::Configuration = self.read_register().await?;
        assert!(configuration_register == actual_config, "Failed to set configuration register!");

        self.write_register_byte(register::GPIO_INTERRUPT_ENABLE1_ADDRESS, 0x00).await?;
        self.write_register_byte(register::GPIO_INTERRUPT_ENABLE2_ADDRESS, 0x00).await?;
        self.write_register_byte(register::GPIO_INTERRUPT_ENABLE3_ADDRESS, 0x00).await?;

        // self.write_register_byte(register::DEBOUNCE_DISABLE1_ADDRESS, 0b00001111).await?;
        // self.write_register_byte(register::DEBOUNCE_DISABLE2_ADDRESS, 0b00001111).await?;
        // self.write_register_byte(register::DEBOUNCE_DISABLE3_ADDRESS, 0x00000000).await?;

        // Enable FIFO for the MENU GPIO buttons.
        self.write_register_byte(register::GPI_EVENT_MODE1_ADDRESS, 0b11110000).await?;

        // Enable rows 0-3
        self.write_register_byte(register::KEYPAD_GPIO1_ADDRESS, 0b00001111).await?;
        // Enable colums 0-3
        self.write_register_byte(register::KEYPAD_GPIO2_ADDRESS, 0b00001111).await?;
        self.write_register_byte(register::KEYPAD_GPIO3_ADDRESS, 0b00000000).await?;

        // "Once the TCA8418 has had the keypad array configured, it will enter idle mode when no keys are being pressed.
        // All columns configured as part of the keypad array will be driven low and all rows configured as part of the
        // keypad array will be set to inputs, with pull-up resistors enabled. During idle mode, the internal oscillator is turned
        // off so that power consumption is low as the device awaits a key press."
        
        Ok(())
    }

    pub async fn process_keypad(&mut self) -> Result<(), I2C::Error> {
        loop {
            // The KEY_EVENT_A (0x04) is the head of the FIFO stack.
            //
            // Each read of KEY_EVENT_A will deincrement the key event count
            // by one, and move all the data in the stack down by one.
            let mut key_event: register::KeyEventA = self.read_register().await?;
            trace!("processing key event {:08b}", key_event.raw());

            // A 0 in the key event A register indicates there are no more key events.
            if key_event.raw() == 0_u8 {
                trace!("no more events in FIFO!");
                break Ok(());
            }

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
                break Ok(());
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

            Timer::after_millis(100).await;
        }
    }

    /// Wait for the next interrupt and process it.
    pub async fn process(&mut self) -> Result<(), I2C::Error> {
        trace!("waiting for keypress interrupt");

        // INT is active-low.
        //
        // We need to handle the interrupt to clear the FIFO
        // if it was triggered before we started listening.
        if self._int.is_low() {
            trace!("interrupt line already low, processing...");
        } else {
            self._int.wait_for_falling_edge().await;
        }

        trace!("received interrupt!");
        
        // When the interrupt is triggered, we need to read
        // the interrupt status table to see what it was.
        let mut int_status: register::InterruptStatus = self.read_register().await?;

        // If there are logged events for the FIFO stack, then we need to read it.
        if int_status.key_event_interrupt_status().get() || int_status.gpi_interrupt_status().get() {
            self.process_keypad().await?;
        }
        
        // Overflow is triggered if the key event FIFO
        // is full and a new keypress event occurs.
        if int_status.overflow_interrupt_status().get() {
            error!("Keypad input event overflow!")
        }

        // Clear all interrupt bits.
        //
        // If one was missed, then the INT line
        // will be reasserted from INT_CFG(CFG[4])=1.
        self.clear_all_interrupts().await?;

        Ok(())
    }

    /// Clears all interrupt flags.
    /// 
    /// Processes a little faster then manually writing
    /// with register::InterruptStatus since it skips
    /// processing the register struct.
    /// 
    /// If INT_CFG(CFG[4])=1 and there are still pending keypressed
    /// in the FIFO, then clearing the interrupt will deassert INT
    /// for 50 μs and update the interrupt table and re-assert.
    pub async fn clear_all_interrupts (&mut self) -> Result<(), I2C::Error> {
        self.write_register_byte(register::INT_STAT_ADDRESS, 0xff).await?;
        Ok(())
    }

/// Reads the value of a register.
pub async fn read_register<REG: register::Register>(&mut self) -> Result<REG, I2C::Error> {
    let write_buf = &mut self.write_read_buf[..1];

    // Writes the register address to read, and
    // then reads the responding register values.
    write_buf[0] = REG::ADDRESS;

    // For some reason if we don't access the buffers like this, then bit 0 is always
    // 1. Appears to be some kind of memory issue with the mapping to RAM D3.
    let read_buf = &mut self.read_buf[..1];

    self.device.write_read(self.address, write_buf, read_buf).await?;

    trace!("read register 0x{:X}={:08b}", write_buf[0], read_buf[0]);

    let result = REG::exact_from(read_buf[0]);
    Ok(result)
}

    /// Writes a new value for a register.
    pub async fn write_register_byte(&mut self, address: u8, value: u8) -> Result<(), I2C::Error> {
        let write_buf = &mut self.write_buf[..2];
        write_buf[0] = address;
        write_buf[1] = value;
        trace!("write register 0x{:X}={:08b}", address, value);
        self.device.write(self.address, write_buf).await
    }

    /// Writes a new value for a register.
    pub async fn write_register<REG: register::Register>(&mut self, reg: REG) -> Result<(), I2C::Error> {
        let write_buf = &mut self.write_buf[..2];
        write_buf[0] = REG::ADDRESS;
        write_buf[1] = reg.into();
        trace!("write register 0x{:X}={:08b}", write_buf[0], write_buf[1]);
        self.device.write(self.address, write_buf).await
    }
}
