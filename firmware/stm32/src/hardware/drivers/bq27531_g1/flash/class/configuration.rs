use embedded_hal_async::i2c::{I2c, SevenBitAddress};
use num::PrimInt;

use crate::hardware::drivers::bq27531_g1::DEVICE_ADDRESS;

// https://www.ti.com/lit/ug/sluua96a/sluua96a.pdf?ts=1761223531200 (P.g. 34)

// Configuration class
pub const SUBCLASS_SAFETY: u8 = 2;
pub const SUBCLASS_CHARGE_TERMINATION: u8 = 36;
pub const SUBCLASS_DATA: u8 = 48;
pub const SUBCLASS_DISCHARGE: u8 = 49;
pub const SUBCLASS_INTEGRITY_DATA: u8 = 56;
pub const SUBCLASS_REGISTERS: u8 = 64;
pub const SUBCLASS_POWER: u8 = 68;

pub trait Field<const RESULT_BYTES: usize> {
    const SUBCLASS: u8;
    const OFFSET: u8;
    type Result: PrimInt;

    fn result(bytes: [u8; RESULT_BYTES]) -> Self::Result;
}

macro_rules! field {
    ($name: ident, $subclass: ident, $offset: tt, i16) => {
        pub struct $name {}
        impl Field<2> for $name {
            const SUBCLASS: u8 = $subclass;
            const OFFSET: u8 = $offset;

            type Result = i16;

            fn result(bytes: [u8; 2]) -> i16 {
                i16::from_le_bytes(bytes)
            }
        }
    };

    ($name: ident, $subclass: ident, $offset: tt, u8) => {
        pub struct $name {}
        impl Field<1> for $name {
            const SUBCLASS: u8 = $subclass;
            const OFFSET: u8 = $offset;

            type Result = u8;

            fn result(bytes: [u8; 1]) -> u8 {
                bytes[0]
            }
        }
    };

    ($name: ident, $subclass: ident, $offset: tt, i8) => {
        pub struct $name {}
        impl Field<1> for $name {
            const SUBCLASS: u8 = $subclass;
            const OFFSET: u8 = $offset;

            type Result = i8;

            fn result(bytes: [u8; 1]) -> i8 {
                bytes[0] as i8
            }
        }
    };
}

field!(SafetyOverTemp, SUBCLASS_SAFETY, 0, i16);
field!(SafetyUnderTemp, SUBCLASS_SAFETY, 2, i16);
field!(SafetyTempHys, SUBCLASS_SAFETY, 4, u8);

field!(
    ChargeTerminationChargingVoltate,
    SUBCLASS_CHARGE_TERMINATION,
    0,
    i16
);
field!(
    ChargeTerminationTaperCurrent,
    SUBCLASS_CHARGE_TERMINATION,
    2,
    i16
);
field!(
    ChargeTerminationMinTaperCurrent,
    SUBCLASS_CHARGE_TERMINATION,
    4,
    i16
);
field!(
    ChargeTerminationTaperVoltage,
    SUBCLASS_CHARGE_TERMINATION,
    6,
    i16
);
field!(
    ChargeTerminationCurrentTaperWindow,
    SUBCLASS_CHARGE_TERMINATION,
    8,
    u8
);
field!(
    ChargeTerminationFCClearPercent,
    SUBCLASS_CHARGE_TERMINATION,
    11,
    i8
);
field!(
    ChargeTerminationFCClearVolt,
    SUBCLASS_CHARGE_TERMINATION,
    12,
    i16
);
field!(
    ChargeTerminationDODatEOCDeltaT,
    SUBCLASS_CHARGE_TERMINATION,
    14,
    i16
);

// field!(RegistersTemps, SUBCLASS_REGISTERS, 0, bool);

pub struct Configuration<'a, I2C: I2c> {
    /// I2C device on the bus.
    device: &'a mut I2C,
}

impl<'a, I2C: I2c> super::Class<'a, I2C> for Configuration<'a, I2C> {}

impl<'a, I2C: I2c> Configuration<'a, I2C> {
    pub fn new(device: &'a mut I2C) -> Self {
        Self { device }
    }

    pub async fn read<const RESULT_BYTES: usize, F: Field<RESULT_BYTES> + Into<F::Result>>(
        &mut self,
    ) -> Result<F::Result, I2C::Error> {
        let result: [u8; RESULT_BYTES] =
            super::read_subclass(self.device, F::SUBCLASS, F::OFFSET).await?;

        Ok(F::result(result))
    }
}
