pub mod register;

use defmt::{error, trace};

use embedded_hal_async::i2c::SevenBitAddress;

use grounded::uninit::GroundedArrayCell;

use crate::hardware::drivers::tca8418::register::KeyEventA;

pub const DEFAULT_ADDRESS: SevenBitAddress = 0b0110100;

/// Struct containing the key event data from the TAC8418 FIFO.
pub struct KeyEvent {
    /// The index of the key that was pressed.
    ///
    /// See: https://www.ti.com/lit/ds/symlink/tca8418.pdf?ts=1761759640741 (P.g. 15).
    pub key: u8,

    /// Whether the key was pressed or released.
    pub pressed: bool,
}

pub enum Gpio {
    Row7,
    Row6,
    Row5,
    Row4,
    Row3,
    Row2,
    Row1,
    Row0,
    Column9,
    Column8,
    Column7,
    Column6,
    Column5,
    Column4,
    Column3,
    Column2,
    Column1,
    Column0,
}

// see: https://github.com/embassy-rs/embassy/blob/abcb6e607c4f13bf99c406fbb92480c32ebd0d4a/docs/pages/faq.adoc#stm32-bdma-only-working-out-of-some-ram-regions
// Defined in memory.x
#[unsafe(link_section = ".ram_d3")]
static mut RAM_D3_BUF: GroundedArrayCell<u8, 4> = GroundedArrayCell::uninit();

pub struct Tca8418<'a, I2C: embedded_hal_async::i2c::I2c> {
    /// I2C bus device handle to use for the TAC8418.
    device: I2C,

    write_buf: &'a mut [u8],      //[u8; 2],
    write_read_buf: &'a mut [u8], // [u8; 1],
    read_buf: &'a mut [u8],       // [u8; 1],

    /// I2C device address.
    address: SevenBitAddress,
}

impl<'a, I2C: embedded_hal_async::i2c::I2c> Tca8418<'a, I2C> {
    /// Constructs a new TCA8418 driver.
    pub fn new(device: I2C) -> Self {
        // The I2C4 peripherial on STM32H7 requires the BDMA, and in
        // turn the BDMA can only access data in the D3 section of RAM.
        //
        // TODO: Move this outside the driver struct for portability.
        //  Instead, pass the buffer references into the constructor.
        let (write_buf, write_read_buf, read_buf) = unsafe {
            let ram = &mut *core::ptr::addr_of_mut!(RAM_D3_BUF);
            ram.initialize_all_copied(0);
            (
                ram.get_subslice_mut_unchecked(0, 2), // 0..1
                ram.get_subslice_mut_unchecked(2, 1), // 2
                ram.get_subslice_mut_unchecked(3, 1), // 3
            )
        };

        Self {
            device,
            address: DEFAULT_ADDRESS,
            write_buf,
            write_read_buf,
            read_buf,
        }
    }

    /// Initializes the TCA8418 with the
    /// register configurations for the device.
    pub async fn init(&mut self) -> Result<(), I2C::Error> {
        // "Once the TCA8418 has had the keypad array configured, it will enter idle mode when no keys are being pressed.
        // All columns configured as part of the keypad array will be driven low and all rows configured as part of the
        // keypad array will be set to inputs, with pull-up resistors enabled. During idle mode, the internal oscillator is turned
        // off so that power consumption is low as the device awaits a key press."

        Ok(())
    }

    /// Read the status of a GPIO pin from the corrosponding register.
    pub async fn read_gpio_status(&mut self, gpio: Gpio) -> Result<bool, I2C::Error> {
        match gpio {
            Gpio::Row7
            | Gpio::Row6
            | Gpio::Row5
            | Gpio::Row4
            | Gpio::Row3
            | Gpio::Row2
            | Gpio::Row1
            | Gpio::Row0 => {
                let mut register = self.read_register::<register::GPIODataStatus1>().await?;
                match gpio {
                    Gpio::Row7 => Ok(register.row_7().get()),
                    Gpio::Row6 => Ok(register.row_6().get()),
                    Gpio::Row5 => Ok(register.row_5().get()),
                    Gpio::Row4 => Ok(register.row_4().get()),
                    Gpio::Row3 => Ok(register.row_3().get()),
                    Gpio::Row2 => Ok(register.row_2().get()),
                    Gpio::Row1 => Ok(register.row_1().get()),
                    Gpio::Row0 => Ok(register.row_0().get()),
                    _ => todo!(),
                }
            }
            Gpio::Column7
            | Gpio::Column6
            | Gpio::Column5
            | Gpio::Column4
            | Gpio::Column3
            | Gpio::Column2
            | Gpio::Column1
            | Gpio::Column0 => {
                let mut register = self.read_register::<register::GPIODataStatus2>().await?;
                match gpio {
                    Gpio::Column7 => Ok(register.column_7().get()),
                    Gpio::Column6 => Ok(register.column_6().get()),
                    Gpio::Column5 => Ok(register.column_5().get()),
                    Gpio::Column4 => Ok(register.column_4().get()),
                    Gpio::Column3 => Ok(register.column_3().get()),
                    Gpio::Column2 => Ok(register.column_2().get()),
                    Gpio::Column1 => Ok(register.column_1().get()),
                    Gpio::Column0 => Ok(register.column_0().get()),
                    _ => todo!(),
                }
            }
            Gpio::Column9 | Gpio::Column8 => {
                let mut register = self.read_register::<register::GPIODataStatus3>().await?;
                match gpio {
                    Gpio::Column9 => Ok(register.column_9().get()),
                    Gpio::Column8 => Ok(register.column_8().get()),
                    _ => todo!(),
                }
            }
        }
    }

    /// Reads the head of the FIFO and returns the key event, or `None` if the FIFO was empty.
    pub async fn poll_fifo(&mut self) -> Result<Option<KeyEventA>, I2C::Error> {
        // The KEY_EVENT_A (0x04) is the head of the FIFO stack.
        //
        // Each read of KEY_EVENT_A will deincrement the key event count
        // by one, and move all the data in the stack down by one.
        let key_event: register::KeyEventA = self.read_register().await?;
        trace!("processing key event {:08b}", key_event.raw());

        // A 0 in the key event A register indicates there are no more key events.
        if key_event.raw() == 0_u8 {
            return Ok(None);
        }

        Ok(Some(key_event))
    }

    /// Convenience method for getting the current interrupt status.
    pub async fn get_interrupt_status(&mut self) -> Result<register::InterruptStatus, I2C::Error> {
        // When the interrupt is triggered, we need to read
        // the interrupt status table to see what it was.
        let mut int_status: register::InterruptStatus = self.read_register().await?;

        // Overflow is triggered if the key event FIFO
        // is full and a new keypress event occurs.
        //
        // Normally this is an error condition that indicates an
        // issue with your input loop, so we log it as an error.
        if int_status.overflow_interrupt_status().get() {
            error!("Keypad input event overflow!")
        }

        Ok(int_status)
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
    pub async fn clear_all_interrupts(&mut self) -> Result<(), I2C::Error> {
        self.write_register_byte(register::INT_STAT_ADDRESS, 0xff)
            .await?;
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

        self.device
            .write_read(self.address, write_buf, read_buf)
            .await?;

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
    pub async fn write_register<REG: register::Register>(
        &mut self,
        reg: REG,
    ) -> Result<(), I2C::Error> {
        let write_buf = &mut self.write_buf[..2];
        write_buf[0] = REG::ADDRESS;
        write_buf[1] = reg.into();
        trace!("write register 0x{:X}={:08b}", write_buf[0], write_buf[1]);
        self.device.write(self.address, write_buf).await
    }
}
