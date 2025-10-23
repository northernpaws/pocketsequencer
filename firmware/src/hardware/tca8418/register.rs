
// Helper that allows us to write registers as structs and read
// them directly as their corrosponding binary representation.
use bit_struct::*; 

/// Mapping of the Tca8418rtwr register names to addresses.
pub enum Address {
    Reserved = 0x00,

    /// Configuration register (interrupt processor, interrupt enables).
    Configuration = 0x01,

    /// Interrupt status register.
    IntStat = 0x02,

    /// Key lock and event counter register.
    KeyLockEventCounter = 0x03,

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
    KeypadLockTimer = 0x0E,

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

/// A trait representing a register.
/// 
/// All registers for the TCA8418 are 8 bit fields.
pub trait Register: BitStruct<true, Kind = u8> {
    /// Returns the address of the register.
    fn address() -> Address;
}

bit_struct! {
    // Configuration Register (Address 0x01)
    pub struct Configuration(u8) {
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

impl Register for Configuration {
    fn address() -> Address {
        Address::Configuration
    }
}

bit_struct! {
    // INT_STAT (Address 0x02)
    pub struct InterruptStatus(u8) {
        reserved_0: bool,
        reserved_1: bool,
        reserved_2: bool,
        /// CTRL-ALT-DEL key sequence status.
        ///  0 = interrupt not detected,
        ///  1 = interrupt detected,
        /// 
        /// Requires writing a 1 to clear interrupts.
        /// 
        /// The following key press combinations will cause a false CAD_INT:
        /// • 1 + 11
        /// • 1 + 21
        /// • 21 + 1 + 11
        ctrl_alt_delete_interrupt_status: bool,
        /// Overflow interrupt status.
        ///  0 = interrupt not detected.
        ///  1 = interrupt detected.
        /// 
        /// Requires writing a 1 to clear interrupts.
        overflow_interrupt_status: bool,
        /// Keypad lock interrupt status.
        /// 
        /// This is the interrupt to the processor when
        /// the keypad lock sequence is started.
        /// 
        ///  0 = interrupt not detected
        ///  1 = interrupt detected
        /// 
        /// Requires writing a 1 to clear interrupts.
        keypad_lock_interrupt_status: bool,
        /// GPI interrupt status. Requires writing a 1 to clear interrupts.
        ///  0 = interrupt not detected
        ///  1 = interrupt detected
        /// 
        /// Can be used to mask interrupts.
        gpi_interrupt_status: bool,
        /// Key events interrupt status.
        ///  0 = interrupt not detected
        ///  1 = interrupt detected
        /// 
        /// Requires writing a 1 to clear interrupts.
        key_event_interrupt_status: bool,
    }
}

impl Register for InterruptStatus {
    fn address() -> Address {
        Address::IntStat
    }
}

bit_struct! {
    // KEY_LCK_EC (Address 0x03)
    pub struct KeyLockAndEventCounter(u8) {
        /// Always 0.
        reserved: bool,
        /// Key lock enable.
        ///  0 = disabled; Write a 0 to this bit to unlock the keypad manually.
        ///  1 = enabled; Write a 1 to this bit to lock the keypad.
        key_lock_enable: bool, // K_LCK_EN
        /// Keypad lock status.
        ///  0 = unlock (if LCK1 is 0 too).
        ///  1 = locked (if LCK1 is 1 too).
        keypad_lock_status_2: bool, // LCK2
        /// Keypad lock status.
        ///  0 = unlock (if LCK2 is 0 too).
        ///  1 = locked (if LCK2 is 1 too).
        keypad_lock_status_1: bool, // LCK1
        /// Key event count.
        key_event_count: u4, // KEC[3:0]
    }
}

impl Register for KeyLockAndEventCounter {
    fn address() -> Address {
        Address::KeyLockEventCounter
    }
}


// Helper to write all the GPIOx registers that have the same field names.
macro_rules! gpio_register {
    ($name:ident) => {
        bit_struct! {
            // KEY_EVENT_A–J (Address 0x04–0x0D)
            //
            // All key event registers can be read, but for the purpose of
            // the FIFO, the user should only read KEY_EVENT_A register.
            // 
            // Once all the events in the FIFO have been read, reading of
            // KEY_EVENT_A register will yield a zero value.
            pub struct $name(u8) {
                /// KEA[7] indicate if a key press or key release has happened.
                /// 
                /// A ‘0’ means a key release happened.
                /// A ‘1’ means a key has been pressed (which can be cleared on a read).
                key_pressed: bool,

                /// KEA[6:0] indicates the key # pressed or released.
                /// 
                /// A value of 0 to 80 indicate which key has been
                /// pressed or released in a keypad matrix.
                /// 
                /// Values of 97 to 114 are for GPI events.
                key_index: u6,
            }
        }


        impl Register for $name {
            fn address() -> Address {
                Address::$name
            }
        }
    }
}

gpio_register!(KeyEventA);
gpio_register!(KeyEventB);
gpio_register!(KeyEventC);
gpio_register!(KeyEventD);
gpio_register!(KeyEventE);
gpio_register!(KeyEventF);
gpio_register!(KeyEventG);
gpio_register!(KeyEventH);
gpio_register!(KeyEventI);
gpio_register!(KeyEventJ);

bit_struct! {
    // KP_LCK_TIMER (Address 0x0E)
    pub struct KeypadLockTimer(u8) {
        /// Lock1 to Lock2 timer must be non-zero for keylock to be enabled.
        /// 
        /// The lock1 to lock2 bits ( KL[2:0] ) define the time
        /// in seconds the user has to press unlock key 2 after
        /// unlock key 1 before the key lock sequence times out.
        lock1_lock2_timer: u5, // KL[2:0]
        /// If the keypad lock interrupt mask timer is non-zero, a key event
        /// interrupt (K_INT) will be generated on any first key press.
        /// 
        /// The second interrupt (K_LCK_IN) will only be generated when the
        /// correct unlock sequence has been completed. If either timer expires,
        /// the keylock state machine will reset. 
        /// 
        /// When the interrupt mask timer is disabled (‘0’), a key lock interrupt
        /// will trigger only when the correct unlock sequence is completed.
        ///  
        /// The interrupt mask timer should be set for the time it takes for the LCD to dim or turn off.
        interrupt_mask_timer: u3, // KL[7:3]
    }
}

impl Register for KeypadLockTimer {
    fn address() -> Address {
        Address::KeypadLockTimer
    }
}

bit_struct! {
    // UNLOCK1 (0x0F)
    pub struct Unlock1(u8) {
        disable: bool, // TODO: not sure if this is correct?

        /// A ‘0’ disables the keylock function.
        key_index: u6,
    }
}

impl Register for Unlock1 {
    fn address() -> Address {
        Address::Unlock1
    }
}

bit_struct! {
    // UNLOCK2 (0x10)
    pub struct Unlock2(u8) {
        disable: bool, // TODO: not sure if this is correct?

        /// A ‘0’ disables the keylock function.
        key_index: u6,
    }
}

impl Register for Unlock2 {
    fn address() -> Address {
        Address::Unlock2
    }
}

// Helper to write all the GPIOx registers that have the same field names.
macro_rules! gpio_register {
    ($name1:ident, $name2:ident, $name3:ident, $typename:ident) => {
        bit_struct! {
            pub struct $name1(u8) {
                row_7: $typename,
                row_6: $typename,
                row_5: $typename,
                row_4: $typename,
                row_3: $typename,
                row_2: $typename,
                row_1: $typename,
            }
        }

        impl Register for $name1 {
            fn address() -> Address {
                Address::$name1
            }
        }

        bit_struct! {
            pub struct $name2(u8) {
                column_7: $typename,
                column_6: $typename,
                column_5: $typename,
                column_4: $typename,
                column_3: $typename,
                column_2: $typename,
                column_1: $typename,
            }
        }

        impl Register for $name2 {
            fn address() -> Address {
                Address::$name2
            }
        }

        bit_struct! {
            pub struct $name3(u8) {
                reserved_0: bool,
                reserved_1: bool,
                reserved_2: bool,
                reserved_3: bool,
                reserved_4: bool,
                reserved_5: bool,
                column_9: $typename,
                column_8: $typename,
            }
        }

        impl Register for $name3 {
            fn address() -> Address {
                Address::$name3
            }
        }
    };
}


// All these registers have 3 common sets of fields.
gpio_register!(GPIOInterruptStatus1, GPIOInterruptStatus2, GPIOInterruptStatus3, bool);

gpio_register!(GPIODataStatus1, GPIODataStatus2, GPIODataStatus3, bool);

gpio_register!(GPIODataOut1, GPIODataOut2, GPIODataOut3, bool);

enums! {
    /// A bit value of '0' in any of the unreserved bits disables the
    /// corresponding pin's ability to generate an interrupt when the
    /// state of the input changes. This is the default value.
    /// 
    /// A bit value of 1 in any of the unreserved bits enables the
    /// corresponding pin's ability to generate an interrupt when
    /// the state of the input changes.
    pub GPIOInterruptEnable { Disabled, Enabled }
}
gpio_register!(GPIOInterruptEnable1, GPIOInterruptEnable2, GPIOInterruptEnable3, GPIOInterruptEnable);


enums! {
    /// A bit value of '0' in any of the unreserved bits puts the corresponding pin in GPIO mode.
    /// 
    /// A pin in GPIO mode can be configured as an input or an output in the GPIO_DIR1-3 registers.
    /// 
    /// This is the default value.
    /// 
    /// A 1 in any of these bits puts the pin in key scan mode and becomes part of the keypad array,
    /// then it is configured as a row or column accordingly (this is not adjustable).
    pub GPIOMode { GPIOMode, KeyScanMode }
}
gpio_register!(KeypadGPIO1, KeypadGPIO2, KeypadGPIO3, GPIOMode);

enums! {
    /// A bit value of '0' in any of the unreserved bits indicates that
    /// it is not part of the event FIFO. This is the default value.
    /// 
    /// A 1 in any of these bits means it is part of the event FIFO. When
    /// the Event Mode register, then any key presses will be added to the
    /// FIFO. Please see Key Event Table for more a pin is setup as a GPI
    /// and has a value of 1 in information.
    pub GPIEventMode { FIFODisabled, FIFOEnabled }
}
gpio_register!(GPIEventMode1, GPIEventMode2, GPIEventMode3, GPIEventMode);

enums! {
    /// A bit value of '0' in any of the unreserved bits sets the corresponding pin as an input.
    /// 
    /// This is the default value. A 1 in any of these bits sets the pin as an output.
    pub GPIODirection { Input, Output }
}
gpio_register!(GPIODirection1, GPIODirection2, GPIODirection3, GPIODirection);

enums! {
    /// A bit value of '0' indicates that interrupt will be triggered
    /// on a high-to-low/low-level transition for the inputs in GPIO
    /// mode. This is the default value.
    /// 
    /// A bit value of 1 indicates that interrupt will be triggered on
    /// a low-to-high/high-level value for the inputs in GPIO mode.
    pub GPIOInterruptLevel { HighToLow, LoWToHigh }
}
gpio_register!(GPIOInterruptLevel1, GPIOInterruptLevel2, GPIOInterruptLevel3, GPIOInterruptLevel);

enums! {
    /// This is for pins configured as inputs. A bit value of ‘0’ in any of
    /// the unreserved bits enables the debounce. This is the default value.
    /// 
    /// A bit value of ‘1’ disables the debounce.
    pub GPIODebounce { Enabled, Disabled }
}
gpio_register!(DebounceDisable1, DebounceDisable2, DebounceDisable3, GPIODebounce);

enums! {
    /// This register enables or disables pull-up registers from inputs.
    /// 
    /// A bit value of '0' will enable the internal pull-up resistors. This is the default value.
    /// 
    /// A bit value of 1 will disable the internal pull-up resistors.
    pub GPIOPullUp { Enabled, Disabled }
}
gpio_register!(GPIOPullUpDisable1, GPIOPullUpDisable2, GPIOPullUpDisable3, GPIOPullUp);


#[repr(u8)]
pub enum Registers {
    Reserved(u8),

    /// Configuration register (interrupt processor, interrupt enables).
    Configuration(Configuration),

    /// Interrupt status register.
    IntStat(InterruptStatus),

    /// Key lock and event counter register.
    KeyLockEventCounter(KeyLockAndEventCounter),

    /// Key event register A.
    KeyEventA(KeyEventA),
    /// Key event register B.
    KeyEventB(KeyEventB),
    /// Key event register C.
    KeyEventC(KeyEventC),
    /// Key event register D.
    KeyEventD(KeyEventD),
    /// Key event register E.
    KeyEventE(KeyEventE),
    /// Key event register F.
    KeyEventF(KeyEventF),
    /// Key event register G.
    KeyEventG(KeyEventG),
    /// Key event register H.
    KeyEventH(KeyEventH),
    /// Key event register I.
    KeyEventI(KeyEventI),
    /// Key event register J.
    KeyEventJ(KeyEventJ),

    /// Keypad lock 1 to lock 2 timer.
    KeypadLockTimer(KeypadLockTimer),

    /// Unlock key 1.
    Unlock1(Unlock1),
    /// Unlock key 2.
    Unlock2(Unlock2),

    /// GPIO interrupt status (row 7 - row 0).
    /// 
    /// These registers are used to check GPIO interrupt status.
    /// 
    /// If the GPI_INT bit is set in INT_STAT register, then the GPI which
    /// set the interrupt is marked with a 1 in the corresponding table.
    ///
    /// To clear the GPI_INT bit, these registers must all be 0x00.
    ///
    /// A read to the register clears the bit.
    GPIOInterruptStatus1(GPIOInterruptStatus1),
    /// GPIO interrupt status (column 7 - column 0).
    /// 
    /// These registers are used to check GPIO interrupt status.
    /// 
    /// If the GPI_INT bit is set in INT_STAT register, then the GPI which
    /// set the interrupt is marked with a 1 in the corresponding table.
    ///
    /// To clear the GPI_INT bit, these registers must all be 0x00.
    ///
    /// A read to the register clears the bit.
    GPIOInterruptStatus2(GPIOInterruptStatus2),
    /// GPIO interrupt status.
    /// 
    /// These registers are used to check GPIO interrupt status.
    /// 
    /// If the GPI_INT bit is set in INT_STAT register, then the GPI which
    /// set the interrupt is marked with a 1 in the corresponding table.
    ///
    /// To clear the GPI_INT bit, these registers must all be 0x00.
    ///
    /// A read to the register clears the bit.
    GPIOInterruptStatus3(GPIOInterruptStatus3),

    /// GPIO data status (read twice to clear) (row 7 - row 0).
    /// 
    /// These registers show the GPIO state when read for inputs and outputs.
    /// 
    /// Read these twice to clear them.
    /// 
    /// If debouncing is enabled, these registers return their
    /// default values until a change of state occurs at an input.
    /// 
    /// Initial pin states can be read by disabling debouncing.
    GPIODataStatus1(GPIODataStatus1),
    /// GPIO data status (read twice to clear) (column 7 - column 0).
    /// 
    /// These registers show the GPIO state when read for inputs and outputs.
    /// 
    /// Read these twice to clear them.
    /// 
    /// If debouncing is enabled, these registers return their
    /// default values until a change of state occurs at an input.
    /// 
    /// Initial pin states can be read by disabling debouncing.
    GPIODataStatus2(GPIODataStatus2),
    /// GPIO data status (read twice to clear) (column 9 - column 8).
    /// 
    /// These registers show the GPIO state when read for inputs and outputs.
    /// 
    /// Read these twice to clear them.
    /// 
    /// If debouncing is enabled, these registers return their
    /// default values until a change of state occurs at an input.
    /// 
    /// Initial pin states can be read by disabling debouncing.
    GPIODataStatus3(GPIODataStatus3),

    /// GPIO data out (row 7 - row 0).
    /// 
    /// These registers contain GPIO data to be written
    /// to GPIO out driver; inputs are not affected.
    /// 
    /// This sets the output for the corresponding GPIO output.
    GPIODataOut1(GPIODataOut1), 
    /// GPIO data out (column 7 - column 0).
    /// 
    /// These registers contain GPIO data to be written
    /// to GPIO out driver; inputs are not affected.
    /// 
    /// This sets the output for the corresponding GPIO output.
    GPIODataOut2(GPIODataOut2),
    /// GPIO data out (column 9 - column 8).
    /// 
    /// These registers contain GPIO data to be written
    /// to GPIO out driver; inputs are not affected.
    /// 
    /// This sets the output for the corresponding GPIO output.
    GPIODataOut3(GPIODataOut3),

    /// GPIO interrupt enable (row 7 - row 0).
    /// 
    /// These registers enable interrupts (bit value 1) or disable
    /// interrupts (bit value '0') for general purpose inputs (GPI)
    /// only. If the input changes on a pin which is setup as a GPI,
    /// then the GPI_INT bit will be set in the INT_STAT register.
    /// 
    /// A bit value of '0' in any of the unreserved bits disables the
    /// corresponding pin's ability to generate an interrupt when the
    /// state of the input changes. This is the default value.
    /// 
    /// A bit value of 1 in any of the unreserved bits enables the
    /// corresponding pin's ability to generate an interrupt when
    /// the state of the input changes.
    GPIOInterruptEnable1(GPIOInterruptEnable1),
    /// GPIO interrupt enable (column 7 - column 0).
    /// 
    /// These registers enable interrupts (bit value 1) or disable
    /// interrupts (bit value '0') for general purpose inputs (GPI)
    /// only. If the input changes on a pin which is setup as a GPI,
    /// then the GPI_INT bit will be set in the INT_STAT register.
    /// 
    /// A bit value of '0' in any of the unreserved bits disables the
    /// corresponding pin's ability to generate an interrupt when the
    /// state of the input changes. This is the default value.
    /// 
    /// A bit value of 1 in any of the unreserved bits enables the
    /// corresponding pin's ability to generate an interrupt when
    /// the state of the input changes.
    GPIOInterruptEnable2(GPIOInterruptEnable2),
    /// GPIO interrupt enable (column 9 - column 8).
    /// 
    /// These registers enable interrupts (bit value 1) or disable
    /// interrupts (bit value '0') for general purpose inputs (GPI)
    /// only. If the input changes on a pin which is setup as a GPI,
    /// then the GPI_INT bit will be set in the INT_STAT register.
    /// 
    /// A bit value of '0' in any of the unreserved bits disables the
    /// corresponding pin's ability to generate an interrupt when the
    /// state of the input changes. This is the default value.
    /// 
    /// A bit value of 1 in any of the unreserved bits enables the
    /// corresponding pin's ability to generate an interrupt when
    /// the state of the input changes.
    GPIOInterruptEnable3(GPIOInterruptEnable3),

    /// Keypad or GPIO selection (row 7 - row 0).
    ///  0: GPIO
    ///  1: KP matrix
    /// 
    /// A bit value of '0' in any of the unreserved bits puts the corresponding pin in GPIO mode.
    /// 
    /// A pin in GPIO mode can be configured as an input or an output in the GPIO_DIR1-3 registers.
    /// 
    /// This is the default value.
    /// 
    /// A 1 in any of these bits puts the pin in key scan mode and becomes part of the keypad array,
    /// then it is configured as a row or column accordingly (this is not adjustable).
    KeypadGPIO1(KeypadGPIO1),
    /// Keypad or GPIO selection (column 7 - column 0).
    ///  0: GPIO
    ///  1: KP matrix
    /// 
    /// 
    /// A bit value of '0' in any of the unreserved bits puts the corresponding pin in GPIO mode.
    /// 
    /// A pin in GPIO mode can be configured as an input or an output in the GPIO_DIR1-3 registers.
    /// 
    /// This is the default value.
    /// 
    /// A 1 in any of these bits puts the pin in key scan mode and becomes part of the keypad array,
    /// then it is configured as a row or column accordingly (this is not adjustable).
    KeypadGPIO2(KeypadGPIO2),
    /// Keypad or GPIO selection (column 9 - column 8).
    ///  0: GPIO
    ///  1: KP matrix
    /// 
    /// 
    /// A bit value of '0' in any of the unreserved bits puts the corresponding pin in GPIO mode.
    /// 
    /// A pin in GPIO mode can be configured as an input or an output in the GPIO_DIR1-3 registers.
    /// 
    /// This is the default value.
    /// 
    /// A 1 in any of these bits puts the pin in key scan mode and becomes part of the keypad array,
    /// then it is configured as a row or column accordingly (this is not adjustable).
    KeypadGPIO3(KeypadGPIO3),

    /// GPI event mode 1 (row 7 - row 0).
    /// 
    /// A bit value of '0' in any of the unreserved bits indicates that
    /// it is not part of the event FIFO. This is the default value.
    /// 
    /// A 1 in any of these bits means it is part of the event FIFO. When
    /// the Event Mode register, then any key presses will be added to the
    /// FIFO. Please see Key Event Table for more a pin is setup as a GPI
    /// and has a value of 1 in information.
    GPIEventMode1(GPIEventMode1),
    /// GPI event mode 2 (column 7 - column 0).
    /// 
    /// A bit value of '0' in any of the unreserved bits indicates that
    /// it is not part of the event FIFO. This is the default value.
    /// 
    /// A 1 in any of these bits means it is part of the event FIFO. When
    /// the Event Mode register, then any key presses will be added to the
    /// FIFO. Please see Key Event Table for more a pin is setup as a GPI
    /// and has a value of 1 in information.
    GPIEventMode2(GPIEventMode2),
    /// GPI event mode 3 (column 9 - column 8).
    /// 
    /// A bit value of '0' in any of the unreserved bits indicates that
    /// it is not part of the event FIFO. This is the default value.
    /// 
    /// A 1 in any of these bits means it is part of the event FIFO. When
    /// the Event Mode register, then any key presses will be added to the
    /// FIFO. Please see Key Event Table for more a pin is setup as a GPI
    /// and has a value of 1 in information.
    GPIEventMode3(GPIEventMode3),

    /// GPIO data direction (row 7 - row 0).
    ///  0: input
    ///  1: output
    /// 
    /// A bit value of '0' in any of the unreserved bits sets the corresponding pin as an input.
    /// 
    /// This is the default value. A 1 in any of these bits sets the pin as an output.
    GPIODirection1(GPIODirection1),
    /// GPIO data direction (column 7 - column 0).
    ///  0: input
    ///  1: output
    /// 
    /// A bit value of '0' in any of the unreserved bits sets the corresponding pin as an input.
    /// 
    /// This is the default value. A 1 in any of these bits sets the pin as an output.
    GPIODirection2(GPIODirection2),
    /// GPIO data direction (column 9 - column 8).
    ///  0: input
    ///  1: output
    /// 
    /// A bit value of '0' in any of the unreserved bits sets the corresponding pin as an input.
    /// 
    /// This is the default value. A 1 in any of these bits sets the pin as an output.
    GPIODirection3(GPIODirection3),

    /// GPIO edge/level detect (row 7 - row 0).
    ///  0: falling/low
    ///  1: rising/high
    /// 
    /// A bit value of '0' indicates that interrupt will be triggered
    /// on a high-to-low/low-level transition for the inputs in GPIO
    /// mode. This is the default value.
    /// 
    /// A bit value of 1 indicates that interrupt will be triggered on
    /// a low-to-high/high-level value for the inputs in GPIO mode.
    GPIOInterruptLevel1(GPIOInterruptLevel1),
    /// GPIO edge/level detect (column 7 - column 0).
    ///  0: falling/low
    ///  1: rising/high
    /// 
    /// A bit value of '0' indicates that interrupt will be triggered
    /// on a high-to-low/low-level transition for the inputs in GPIO
    /// mode. This is the default value.
    /// 
    /// A bit value of 1 indicates that interrupt will be triggered on
    /// a low-to-high/high-level value for the inputs in GPIO mode.
    GPIOInterruptLevel2(GPIOInterruptLevel2),
    /// GPIO edge/level detect (column 9 - column 8).
    ///  0: falling/low
    ///  1: rising/high
    /// 
    /// A bit value of '0' indicates that interrupt will be triggered
    /// on a high-to-low/low-level transition for the inputs in GPIO
    /// mode. This is the default value.
    /// 
    /// A bit value of 1 indicates that interrupt will be triggered on
    /// a low-to-high/high-level value for the inputs in GPIO mode.
    GPIOInterruptLevel3(GPIOInterruptLevel3),

    /// Debounce disable  (row 7 - row 0).
    ///  0: debounce enabled
    ///  1: debounce disabled
    /// 
    /// This is for pins configured as inputs. A bit value of ‘0’ in any of
    /// the unreserved bits enables the debounce. This is the default value.
    /// 
    /// A bit value of ‘1’ disables the debounce.
    DebounceDisable1(DebounceDisable1),
    /// Debounce disable (column 7 - column 0).
    ///  0: debounce enabled
    ///  1: debounce disabled
    /// 
    /// This is for pins configured as inputs. A bit value of ‘0’ in any of
    /// the unreserved bits enables the debounce. This is the default value.
    /// 
    /// A bit value of ‘1’ disables the debounce.
    DebounceDisable2(DebounceDisable2),
    /// Debounce disable (column 9 - column 8).
    ///  0: debounce enabled
    ///  1: debounce disabled
    /// 
    /// This is for pins configured as inputs. A bit value of ‘0’ in any of
    /// the unreserved bits enables the debounce. This is the default value.
    /// 
    /// A bit value of ‘1’ disables the debounce.
    DebounceDisable3(DebounceDisable3),

    /// GPIO pull-up disable (row 7 - row 0).
    ///  0: pull-up enabled
    ///  1: pull-up disabled
    /// 
    /// This register enables or disables pull-up registers from inputs.
    /// 
    /// A bit value of '0' will enable the internal pull-up resistors. This is the default value.
    /// 
    /// A bit value of 1 will disable the internal pull-up resistors.
    GPIOPullUpDisable1(GPIOPullUpDisable1),
    /// GPIO pull-up disable (column 7 - column 0).
    ///  0: pull-up enabled
    ///  1: pull-up disabled
    /// 
    /// This register enables or disables pull-up registers from inputs.
    /// 
    /// A bit value of '0' will enable the internal pull-up resistors. This is the default value.
    /// 
    /// A bit value of 1 will disable the internal pull-up resistors.
    GPIOPullUpDisable2(GPIOPullUpDisable2),
    /// GPIO pull-up disable (column 9 - column 8).
    ///  0: pull-up enabled
    ///  1: pull-up disabled
    /// 
    /// This register enables or disables pull-up registers from inputs.
    /// 
    /// A bit value of '0' will enable the internal pull-up resistors. This is the default value.
    /// 
    /// A bit value of 1 will disable the internal pull-up resistors.
    GPIOPullUpDisable3(GPIOPullUpDisable3),
    
    Reserved2(u8),
}

impl Registers {
    /// Returns the register address for the associated register.
    pub fn address(&self) -> Address {
        match self {
            Registers::Reserved(_) => Address::Reserved,
            Registers::Configuration(_) => Address::Configuration,
            Registers::IntStat(_) => Address::IntStat,
            Registers::KeyLockEventCounter(_) => Address::KeyLockEventCounter,
            Registers::KeyEventA(_) => Address::KeyEventA,
            Registers::KeyEventB(_) => Address::KeyEventB,
            Registers::KeyEventC(_) => Address::KeyEventC,
            Registers::KeyEventD(_) => Address::KeyEventD,
            Registers::KeyEventE(_) => Address::KeyEventE,
            Registers::KeyEventF(_) => Address::KeyEventF,
            Registers::KeyEventG(_) => Address::KeyEventG,
            Registers::KeyEventH(_) => Address::KeyEventH,
            Registers::KeyEventI(_) => Address::KeyEventI,
            Registers::KeyEventJ(_) => Address::KeyEventJ,
            Registers::KeypadLockTimer(_) => Address::KeypadLockTimer,
            Registers::Unlock1(_) => Address::Unlock1,
            Registers::Unlock2(_) => Address::Unlock2,
            Registers::GPIOInterruptStatus1(_) => Address::GPIOInterruptStatus1,
            Registers::GPIOInterruptStatus2(_) => Address::GPIOInterruptStatus2,
            Registers::GPIOInterruptStatus3(_) => Address::GPIOInterruptStatus3,
            Registers::GPIODataStatus1(_) => Address::GPIODataStatus1,
            Registers::GPIODataStatus2(_) => Address::GPIODataStatus2,
            Registers::GPIODataStatus3(_) => Address::GPIODataStatus3,
            Registers::GPIODataOut1(_) => Address::GPIODataOut1,
            Registers::GPIODataOut2(_) => Address::GPIODataOut2,
            Registers::GPIODataOut3(_) => Address::GPIODataOut3,
            Registers::GPIOInterruptEnable1(_) => Address::GPIOInterruptEnable1,
            Registers::GPIOInterruptEnable2(_) => Address::GPIOInterruptEnable2,
            Registers::GPIOInterruptEnable3(_) => Address::GPIOInterruptEnable3,
            Registers::KeypadGPIO1(_) => Address::KeypadGPIO1,
            Registers::KeypadGPIO2(_) => Address::KeypadGPIO2,
            Registers::KeypadGPIO3(_) => Address::KeypadGPIO3,
            Registers::GPIEventMode1(_) => Address::GPIEventMode1,
            Registers::GPIEventMode2(_) => Address::GPIEventMode2,
            Registers::GPIEventMode3(_) => Address::GPIEventMode3,
            Registers::GPIODirection1(_) => Address::GPIODirection1,
            Registers::GPIODirection2(_) => Address::GPIODirection2,
            Registers::GPIODirection3(_) => Address::GPIODirection3,
            Registers::GPIOInterruptLevel1(_) => Address::GPIOInterruptLevel1,
            Registers::GPIOInterruptLevel2(_) => Address::GPIOInterruptLevel2,
            Registers::GPIOInterruptLevel3(_) => Address::GPIOInterruptLevel3,
            Registers::DebounceDisable1(_) => Address::DebounceDisable1,
            Registers::DebounceDisable2(_) => Address::DebounceDisable2,
            Registers::DebounceDisable3(_) => Address::DebounceDisable3,
            Registers::GPIOPullUpDisable1(_) => Address::GPIOPullUpDisable1,
            Registers::GPIOPullUpDisable2(_) => Address::GPIOPullUpDisable2,
            Registers::GPIOPullUpDisable3(_) => Address::GPIOPullUpDisable3,
            Registers::Reserved2(_) => Address::Reserved2,
        }
    }
}
