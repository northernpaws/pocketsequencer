// Helper that allows us to write registers as structs and read
// them directly as their corrosponding binary representation.
use bit_struct::*; 

use embassy_stm32::{
    exti::ExtiInput
};

/// Holds all the buttons on the device mapped
/// to the TCA8418RTWR keypad decoder.
pub enum Button {
    Menu1,
    Menu2,
    Menu3,
    Menu4,

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

const ADDRESS: u8 = 0b0110100;

/// Mapping of the Tca8418rtwr register names to addresses.
pub enum RegisterAddress {
    Reserved = 0x00,

    /// Configuration register (interrupt processor, interrupt enables).
    CFG = 0x01,

    /// Interrupt status register.
    IntStat = 0x02,

    /// Key lock and event counter register.
    KeyLckEC = 0x03,

    /// Key event register A.
    KeyEventA = 0x04,
    /// Key event register B.
    KeyEventB = 0x05,
    /// Key event register C.
    KeyEventC = 0x06,
    /// Key event register D.
    KeyEventD = 0x07,
    /// Key event register E.
    KeyEventE = 0x08,
    /// Key event register F.
    KeyEventF = 0x09,
    /// Key event register G.
    KeyEventG = 0x0A,
    /// Key event register H.
    KeyEventH = 0x0B,
    /// Key event register I.
    KeyEventI = 0x0C,
    /// Key event register J.
    KeyEventJ = 0x0D,

    /// Keypad lock 1 to lock 2 timer.
    KPLckTimer = 0x0E,

    /// Unlock key 1.
    Unlock1 = 0x0F,
    /// Unlock key 2.
    Unlock2 = 0x10,

    /// GPIO interrupt status.
    GPIOInterruptStatus1 = 0x11,
    /// GPIO interrupt status.
    GPIOInterruptStatus2 = 0x12,
    /// GPIO interrupt status.
    GPIOInterruptStatus3 = 0x13,
    /// GPIO data status (read twice to clear).
    GPIODataStatus1 = 0x14,
    /// GPIO data status (read twice to clear).
    GPIODataStatus2 = 0x15,
    /// GPIO data status (read twice to clear).
    GPIODataStatus3 = 0x16,

    /// GPIO data out.
    GPIODataOut1 = 0x17, 
    /// GPIO data out.
    GPIODataOut2 = 0x18,
    /// GPIO data out.
    GPIODataOut3  = 0x19,

    /// GPIO interrupt enable.
    GPIOInterruptEnable1 = 0x1A,
    /// GPIO interrupt enable.
    GPIOInterruptEnable2 = 0x1B,
    /// GPIO interrupt enable.
    GPIOInterruptEnable3 = 0x1C,

    /// Keypad or GPIO selection.
    ///  0: GPIO
    ///  1: KP matrix
    KeypadGPIO1 = 0x1D,
    /// Keypad or GPIO selection.
    ///  0: GPIO
    ///  1: KP matrix
    KeypadGPIO2 = 0x1E,
    /// Keypad or GPIO selection.
    ///  0: GPIO
    ///  1: KP matrix
    KeypadGPIO3 = 0x1F,

    /// GPI event mode 1.
    GPIEventMode1 = 0x20,
    /// GPI event mode 2.
    GPIEventMode2 = 0x21,
    /// GPI event mode 3.
    GPIEventMode3 = 0x22,

    /// GPIO data direction.
    ///  0: input
    ///  1: output
    GPIODirection1 = 0x23,
    /// GPIO data direction.
    ///  0: input
    ///  1: output
    GPIODirection2 = 0x24,
    /// GPIO data direction.
    ///  0: input
    ///  1: output
    GPIODirection3 = 0x25,

    /// GPIO edge/level detect
    ///  0: falling/low
    ///  1: rising/high
    GPIOInterruptLevel1 = 0x26,
    /// GPIO edge/level detect
    ///  0: falling/low
    ///  1: rising/high
    GPIOInterruptLevel2 = 0x27,
    /// GPIO edge/level detect
    ///  0: falling/low
    ///  1: rising/high
    GPIOInterruptLevel3 = 0x28,

    /// Debounce disable
    ///  0: debounce enabled
    ///  1: debounce disabled
    DebounceDisable1 = 0x29,
    /// Debounce disable
    ///  0: debounce enabled
    ///  1: debounce disabled
    DebounceDisable2 = 0x2A,
    /// Debounce disable
    ///  0: debounce enabled
    ///  1: debounce disabled
    DebounceDisable3 = 0x2B,

    /// GPIO pull-up disable
    ///  0: pull-up enabled
    ///  1: pull-up disabled
    GPIOPullUpDisable1 = 0x2C,
    /// GPIO pull-up disable
    ///  0: pull-up enabled
    ///  1: pull-up disabled
    GPIOPullUpDisable2 = 0x2D,
    /// GPIO pull-up disable
    ///  0: pull-up enabled
    ///  1: pull-up disabled
    GPIOPullUpDisable3 = 0x2E,

    Reserved2 = 0x2F,
}

bit_struct! {
    pub struct ConfigurationRegister(u8) {
        /// Auto-increment for read and write operations.
        ///  0 = disabled
        ///  1 = enabled
        ai: bool,
        /// GPI event mode configuration
        ///  0 = GPI events are tracked when keypad is locked.
        ///  1 = GPI events are not tracked when keypad is locked.
        gpi_event_mode_configuration: bool,
        /// Overflow mode.
        /// 0 = disabled; Overflow data is lost.
        ///  1 = enabled; Overflow data shifts with last event pushing first event out.
        overflow_mode: bool,
        /// Interrupt configuration.
        ///   0 = Processor interrupt remains asserted (or low) if
        ///        host tries to clear interrupt while there is still
        ///        a pending key press, key release or GPI interrupt.
        ///   1 = processor interrupt is deasserted for 50 μs
        ///        and reassert with pending interrupts.
        interrupt_configuration: bool,
        /// Overflow interrupt enable.
        ///  0 = disabled; INT is not asserted if the FIFO overflows.
        ///  1 = enabled; INT becomes asserted if the FIFO overflows.
        overflow_interrupt_enable: bool,
        /// Keypad lock interrupt enable.
        ///  0 = disabled; INT is not asserted after a correct unlock key sequence.
        ///  1 = enabled; INT becomes asserted after a correct unlock key sequence.
        keypad_lock_interrupt_enable: bool,
        /// GPI interrupt enable to host processor.
        ///  0 = disabled; INT is not asserted for a change on a GPI.
        ///. 1 = enabled; INT becomes asserted for a change on a GPI.
        gpi_interrupt_enable: bool,
        /// Key events interrupt enable to host processor.
        ///  0 = disabled; INT is not asserted when a key event occurs.
        ///  1 = enabled; INT becomes asserted when a key event occurs.
        key_events_interrupt_enable: bool
    }
}

pub struct Tca8418rtwr<'a> {
    // Exti-bound input pin for the keypad interrupt signal.
    int: ExtiInput<'a>,
}

impl<'a> Tca8418rtwr<'a> {
    pub fn new (
        int: ExtiInput<'a>,
    ) -> Self {
        Self{
            int,
        }
    }
}
