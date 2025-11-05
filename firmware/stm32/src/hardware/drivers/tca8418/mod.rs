pub mod register;

use defmt::{error, info, trace, warn};

use embassy_stm32::exti::ExtiInput;

use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    channel::{Channel, Receiver, Sender},
};
use embassy_time::Timer;
use embedded_hal_async::i2c::SevenBitAddress;

use grounded::uninit::GroundedArrayCell;

const DEFAULT_ADDRESS: SevenBitAddress = 0b0110100;

/// Struct containing the key event data from the TAC8418 FIFO.
pub struct KeyEvent {
    /// The index of the key that was pressed.
    ///
    /// See: https://www.ti.com/lit/ds/symlink/tca8418.pdf?ts=1761759640741 (P.g. 15).
    pub key: u8,

    /// Whether the key was pressed or released.
    pub pressed: bool,
}

/// Type for emitting key events from the TAC8418 driver.
///
/// N is double the TAC8418 FIFO queue size of 10 to ensure we have
/// enough time to process events without causing an FIFO overflow.
pub type KeyChannel = Channel<CriticalSectionRawMutex, KeyEvent, 20>;
pub type KeyChannelReceiver<'a> = Receiver<'a, CriticalSectionRawMutex, KeyEvent, 20>;

// see: https://github.com/embassy-rs/embassy/blob/abcb6e607c4f13bf99c406fbb92480c32ebd0d4a/docs/pages/faq.adoc#stm32-bdma-only-working-out-of-some-ram-regions
// Defined in memory.x
#[unsafe(link_section = ".ram_d3")]
static mut RAM_D3_BUF: GroundedArrayCell<u8, 4> = GroundedArrayCell::uninit();

pub struct Tca8418<'a, I2C: embedded_hal_async::i2c::I2c> {
    // Exti-bound input pin for the keypad interrupt signal.
    _int: ExtiInput<'a>,

    /// I2C bus device handle to use for the TAC8418.
    device: I2C,

    write_buf: &'a mut [u8],      //[u8; 2],
    write_read_buf: &'a mut [u8], // [u8; 1],
    read_buf: &'a mut [u8],       // [u8; 1],

    /// I2C device address.
    address: SevenBitAddress,

    /// Channel for emitting keypress events.
    channel: &'a KeyChannel,
}

impl<'a, I2C: embedded_hal_async::i2c::I2c> Tca8418<'a, I2C> {
    /// Constructs a new TCA8418 driver.
    pub fn new(int: ExtiInput<'a>, device: I2C, channel: &'static KeyChannel) -> Self {
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
            _int: int,
            device,
            channel,
            address: DEFAULT_ADDRESS,
            write_buf,
            write_read_buf,
            read_buf,
        }
    }

    /// Get a receiver handle for the key event channel.
    pub fn receiver(&mut self) -> KeyChannelReceiver<'a> {
        self.channel.receiver()
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

    /// Reads the head of the FIFO queue in a loop until the FIFO is cleared.
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

            trace!(
                "key {} pressed: {}",
                defmt::Display2Format(&key_index),
                key_pressed
            );

            // NOTE: We use `try_send` to imeddiatly try to push
            // to the channel to prevent backing up the FIFO.
            //
            // This may need to be revised later to use a timeout.
            match self.channel.try_send(KeyEvent {
                key: key_index,
                pressed: key_pressed,
            }) {
                Ok(_) => {}
                Err(_) => {
                    error!("Failed to add keypress event to channel, is the channel full?")
                }
            }

            Timer::after_millis(100).await;
        }
    }

    /// Reacts to the interrupt line to process interrupt related events
    pub async fn process(&mut self) -> Result<(), I2C::Error> {
        // INT is active-low.
        //
        // We need to handle the interrupt to clear the FIFO
        // if it was triggered before we started listening.
        if !self._int.is_low() {
            self._int.wait_for_falling_edge().await;
        }

        // When the interrupt is triggered, we need to read
        // the interrupt status table to see what it was.
        let mut int_status: register::InterruptStatus = self.read_register().await?;

        // If there are logged events for the FIFO stack, then we need to read it.
        if int_status.key_event_interrupt_status().get() || int_status.gpi_interrupt_status().get()
        {
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
