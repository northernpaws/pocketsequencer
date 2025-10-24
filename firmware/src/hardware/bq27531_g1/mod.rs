pub mod register;
pub mod command;

use core::marker::Copy;

use defmt::{trace, warn};

use embassy_stm32::exti::ExtiInput;

use embedded_hal_async::i2c::{
    I2c, SevenBitAddress
};

use embedded_hal::delay::DelayNs;

const DEVICE_ADDRESS: SevenBitAddress = 0b1010101;

pub struct Bq27531<'a,
    I2C: I2c,
    DELAY: DelayNs,
> {
    // Exti-bound input pin for the keypad interrupt signal.
    _int: ExtiInput<'a>,

    /// I2C device on the bus.
    device: I2C,

    delay: DELAY,

    sealed: bool,
}

impl<'a, I2C: I2c, DELAY: DelayNs> Bq27531<'a, I2C, DELAY> {
    /// Constructs a new BG27531-G1 driver.
    pub fn new (
        int: ExtiInput<'a>,
        device: I2C,
        delay: DELAY,
    ) -> Self {
        Self{
            _int: int,
            device,
            delay: delay,
            sealed: false,
        }
    }

    /// Reads an unsigned word from the specified I2C command code.
    async fn read_u16(&mut self, code: &command::CommandCode) -> Result<u16, I2C::Error> {
        let mut result = [0u8; 2];
        self.device.write_read(DEVICE_ADDRESS, code, &mut result).await?;
        Ok(u16::from_le_bytes(result))
    }

    /// Reads a signed word from the specified I2C command code.
    async fn read_i16(&mut self, code: &command::CommandCode) -> Result<i16, I2C::Error> {
        let mut result = [0u8; 2];
        self.device.write_read(DEVICE_ADDRESS, code, &mut result).await?;
        Ok(i16::from_le_bytes(result))
    }

    /// The AtRate() read- and write-word function is the first half of a two-function
    /// command set that sets the AtRate value used in calculations made by the AtRateTimeToEmpty() function.
    /// 
    /// The AtRate() units are in mA.
    /// 
    /// The AtRate() value is a signed integer, with negative values interpreted as a discharge current value.
    /// 
    /// The AtRateTimeToEmpty() function returns the predicted operating time at the AtRate value of discharge.
    /// 
    /// The default value for AtRate() is 0 and forces AtRateTimeToEmpty() to return 65,535. Both the AtRate()
    /// and AtRateTimeToEmpty() commands must only be used in NORMAL mode.
    pub async fn read_at_rate(&mut self) -> Result<i16, I2C::Error> {
        self.read_i16(&command::AT_RATE_COMMAND_CODE).await
    }

    // The AtRate() read- and write-word function is the first half of a two-function
    /// command set that sets the AtRate value used in calculations made by the AtRateTimeToEmpty() function.
    /// 
    /// The AtRate() units are in mA.
    /// 
    /// The AtRate() value is a signed integer, with negative values interpreted as a discharge current value.
    /// 
    /// The AtRateTimeToEmpty() function returns the predicted operating time at the AtRate value of discharge.
    /// 
    /// The default value for AtRate() is 0 and forces AtRateTimeToEmpty() to return 65,535. Both the AtRate()
    /// and AtRateTimeToEmpty() commands must only be used in NORMAL mode.
    pub async fn write_at_rate(&mut self, value: i16) -> Result<(), I2C::Error> {
        let bytes = value.to_le_bytes();
        self.device.write(DEVICE_ADDRESS, &[
            command::AT_RATE_COMMAND_CODE[0],
            command::AT_RATE_COMMAND_CODE[1],
            bytes[0],
            bytes[1],
        ]).await
    }

    /// If the battery is discharged at the AtRate() value, then this read-word function returns an unsigned
    /// integer value of the predicted remaining operating time in minutes with a range of 0 to 65,534.
    /// 
    /// A value of 65,535 indicates AtRate()= 0.
    /// 
    /// The fuel gauge updates AtRateTimeToEmpty() within 1 second after the system sets the AtRate() value.
    /// 
    /// The fuel gauge automatically updates AtRateTimeToEmpty() based on the AtRate() value every 1 second.
    /// 
    /// Both the AtRate() and AtRateTimeToEmpty() commands must only be used in NORMAL mode.
    pub async fn read_at_rate_time_to_empty(&mut self) -> Result<u16, I2C::Error> {
        self.read_u16(&command::AT_RATE_TIME_TO_EMPTY_COMMAND_CODE).await
    }

    /// If the battery is discharged at the AtRate() value, then this read-word function returns an unsigned
    /// integer value of the predicted remaining operating time in minutes with a range of 0 to 65,534.
    /// 
    /// A value of 65,535 indicates AtRate()= 0.
    /// 
    /// The fuel gauge updates AtRateTimeToEmpty() within 1 second after the system sets the AtRate() value.
    /// 
    /// The fuel gauge automatically updates AtRateTimeToEmpty() based on the AtRate() value every 1 second.
    /// 
    /// Both the AtRate() and AtRateTimeToEmpty() commands must only be used in NORMAL mode.
    pub async fn write_at_rate_time_to_empty(&mut self, value: u16) -> Result<(), I2C::Error> {
        let bytes = value.to_le_bytes();
        self.device.write(DEVICE_ADDRESS, &[
            command::AT_RATE_TIME_TO_EMPTY_COMMAND_CODE[0],
            command::AT_RATE_TIME_TO_EMPTY_COMMAND_CODE[1],
            bytes[0],
            bytes[1],
        ]).await
    }

    /// This read- and write-word function returns an unsigned integer value
    /// of the temperature in units of 0.1°K as measured by the fuel gauge.
    /// 
    /// If the [WRTEMP] bit = 0 and the Op Config [TEMPS] bit = 0, a read
    /// command returns the internal temperature sensor value.
    pub async fn read_temperature(&mut self) -> Result<u16, I2C::Error> {
        self.read_u16(&command::TEMPERATURE_COMMAND_CODE).await
    }

    /// his read- and write-word function returns an unsigned integer value
    /// of the temperature in units of 0.1°K as measured by the fuel gauge.
    /// 
    /// If the OpConfig B [WRTEMP] bit = 1, a write command sets the
    /// temperature to be used for gauging calculations while a read
    /// command returns to temperature previously written.
    pub async fn write_temperature(&mut self, value: u16) -> Result<(), I2C::Error> {
        let bytes = value.to_le_bytes();
        self.device.write(DEVICE_ADDRESS, &[
            command::TEMPERATURE_COMMAND_CODE[0],
            command::TEMPERATURE_COMMAND_CODE[1],
            bytes[0],
            bytes[1],
        ]).await
    }

    /// This read-word function returns an unsigned integer value of the
    /// measured cell-pack voltage in mV with a range of 0 to 6000 mV.
    pub async fn read_voltage(&mut self) -> Result<u16, I2C::Error> {
        self.read_u16(&command::VOLTAGE_COMMAND_CODE).await
    }

    /// This read-word function returns the contents of the fuel-gauge
    /// status register, depicting the current operating status.
    pub async fn read_flags (&mut self) -> Result<u16, I2C::Error> {
        todo!("convert result to register bitfield"); // https://www.ti.com/lit/ug/sluua96a/sluua96a.pdf?ts=1761223531200 p.g. 15

        self.read_u16(&command::FLAGS_COMMAND_CODE).await
    }
 
    /// This read-only command pair returns the uncompensated (less than
    /// C/20 load) remaining battery capacity. Units are mAh.
    // TODO: not sure if this returns a word.
    pub async fn read_nominal_available_capacity(&mut self) -> Result<u16, I2C::Error> {
        todo!("is 8 or 16?");
        self.read_u16(&command::NOMINAL_AVAILABLE_CAPACITY_COMMAND_CODE).await
    }

    /// This read-only command pair returns the uncompensated (less
    /// than C/20 load) capacity of the battery when fully charged.
    /// 
    /// Units are mAh.
    /// 
    /// FullAvailableCapacity() is updated at regular intervals, as specified by the IT algorithm.
    pub async fn read_full_available_capacity(&mut self) -> Result<u16, I2C::Error> {
        todo!("is 8 or 16?");
        self.read_u16(&command::FULL_AVAILABLE_CAPACITY_COMMAND_CODE).await
    }

    /// When the OpConfig C [SmoothEn] bit is cleared, this read-only command pair returns
    /// the compensated capacity of a fully charged battery (FullChargeCapacityUnfiltered()).
    /// 
    /// When the [SmoothEn] bit is set, this command pair returns the filtered compensated
    /// capacity of a fully charged battery (FullChargeCapacityFiltered()).
    /// 
    /// Units are mAh.
    /// 
    /// FullChargeCapacity() is updated at regular intervals, as specified by the IT algorithm.
    pub async fn read_remaining_capacity(&mut self) -> Result<u16, I2C::Error> {
        todo!("is 8 or 16?");
        self.read_u16(&command::REMAINING_CAPACIY_COMMAND_CODE).await
    }

    /// This read-only command pair returns a signed integer value that
    /// is the average current flow through the sense resistor.
    /// 
    /// It is negative during discharge and positive during charge.
    /// 
    /// It is updated every 1 second. Units are mA.
    pub async fn read_average_current(&mut self) -> Result<i16, I2C::Error> {
        todo!("is 8 or 16?");
        self.read_i16(&command::AVERAGE_CURRENT_COMMAND_CODE).await
    }

    /// This read-only function returns an unsigned integer value of the predicted
    /// remaining battery life at the present rate of discharge, in minutes.
    /// 
    /// A value of 65,535 indicates the battery is not being discharged.
    pub async fn read_time_to_empty(&mut self) -> Result<u16, I2C::Error> {
        self.read_u16(&command::TIME_TO_EMPTY_CODE).await
    }

    /// This read-only command pair returns the compensated remaining battery capacity.
    /// 
    /// Units are mAh.
    pub async fn read_remaining_capacity_unfiltered(&mut self) -> Result<u16, I2C::Error> {
        todo!("is 8 or 16?");
        self.read_u16(&command::REMAINING_CAPACITY_UNFILTERED_COMMAND_CODE).await
    }

    /// This read-only function returns a signed integer value of the measured standby current
    /// through the sense resistor. The StandbyCurrent() is an adaptive measurement. Initially,
    /// it reports the standby current programmed in Initial Standby, and, after spending several
    /// seconds in standby, reports the measured standby current.
    /// 
    /// The register value is updated every 1 second when the measured current is above the Deadband
    /// and is less than or equal to 2 × Initial Standby. The first and last values that meet this
    /// criteria are not averaged in because they may not be stable values. To approximate a 1-minute
    /// time constant, each new StandbyCurrent() value is computed by taking approximately 93% of the
    /// last measured standby current and approximately 7% of the currently measured average current.
    pub async fn read_standby_current(&mut self) -> Result<i16, I2C::Error> {
        self.read_i16(&command::STANDBY_CURRENT_COMMAND_CODE).await
    }

    /// When the OpConfig C [SmoothEn] bit is set, this read-only command pair returns
    /// the filtered compensated remaining battery capacity. Units are mAh.
    pub async fn read_remaining_capacity_filtered(&mut self) -> Result<u16, I2C::Error> {
        self.read_u16(&command::REMAINING_CAPACITY_FILTERED_COMMAND_CODE).await
    }

    /// This read-only function returns an unsigned integer value of the charging current
    /// that is programmed to the charger. This value is dependent on the offset (512 mA)
    /// and resolution (64 mA) of the programmable charge current of the charger and the
    /// CalcChargingCurrent() obtained through the fuel gauge charging algorithm.
    pub async fn read_prog_charging_current(&mut self) -> Result<u16, I2C::Error> {
        self.read_u16(&command::PROG_CHARGING_CURRENT_COMMAND_CODE).await
    }

    /// This read-only function returns an unsigned integer value of the charging voltage
    /// that is programmed to the charger. This value is dependent on the offset (3504 mV)
    /// and resolution (16 mV) of the programmable charge voltage of the charger and the
    /// CalcChargingVoltage() obtained through the fuel gauge charging algorithm.
    pub async fn read_prog_charging_voltage(&mut self) -> Result<u16, I2C::Error> {
        self.read_u16(&command::PROG_CHARGING_VOLTAGE_COMMAND_CODE).await
    }

    /// This read-only command pair returns the compensated capacity of the battery when fully charged.
    /// 
    /// Units are mAh.
    /// 
    /// FullChargeCapacityUnfiltered() is updated at regular intervals, as specified by the IT algorithm.
    pub async fn read_full_charge_capacity_unfiltered(&mut self) -> Result<u16, I2C::Error> {
        self.read_u16(&command::FULL_CHARGE_CAPACITY_UNFILTERED_COMMAND_CODE).await
    }

    /// This read-only function returns a signed integer value of the
    /// average power during battery charging and discharging.
    /// 
    /// It is negative during discharge and positive during charge.
    /// 
    /// A value of 0 indicates that the battery is not being discharged.
    /// 
    /// Units are mW.
    pub async fn read_average_power(&mut self) -> Result<i16, I2C::Error> {
        self.read_i16(&command::AVERAGE_POWER_COMMAND_CODE).await
    }

    /// This read-only command pair returns the filtered compensated capacity of
    /// the battery when fully charged and the OpConfig C [SmoothEn] bit is set.
    /// 
    /// Units are mAh.
    /// 
    /// FullChargeCapacityFiltered() is updated at regular intervals, as specified by the IT algorithm.
    pub async fn read_full_charge_capacity_filtered(&mut self) -> Result<u16, I2C::Error> {
        self.read_i16(&command::FULL_CHARGE_CAPACITY_FILTERED_COMMAND_CODE).await
    }
}
