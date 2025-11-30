pub mod class;

use embedded_hal_async::i2c::{I2c, SevenBitAddress};

use crate::hardware::drivers::bq27531_g1::DEVICE_ADDRESS;

// Configuration class
pub const CONFIGURATION_SUBCLASS_SAFETY: u8 = 2;
pub const CONFIGURATION_SUBCLASS_CHARGE_TEMRINATION: u8 = 36;
pub const CONFIGURATION_SUBCLASS_DATA: u8 = 48;
pub const CONFIGURATION_SUBCLASS_DISCHARGE: u8 = 49;
pub const CONFIGURATION_SUBCLASS_INTEGRITY_DATA: u8 = 56;
pub const CONFIGURATION_SUBCLASS_REGISTERS: u8 = 64;
pub const CONFIGURATION_SUBCLASS_POWER: u8 = 68;

// Fuel Gauging
pub const FUEL_GAUGING_SUBCLASS_IT_CF: u8 = 80;
pub const FUEL_GAUGING_SUBCLASS_CURRENT_THRESHOLDS: u8 = 81;
pub const FUEL_GAUGING_SUBCLASS_STATE: u8 = 82;
pub const FUEL_GAUGING_SUBCLASS_LAST_RUN: u8 = 95;

// OCV Tables
pub const OCV_TABLES_SUBCLASS_OCVA0_TABLE: u8 = 83;
pub const OCV_TABLES_SUBCLASS_OCVA1_TABLE: u8 = 84;

// RA Class
pub const RA_CLASS_SUBCLASS_DEF0_RA: u8 = 87;
pub const RA_CLASS_SUBCLASS_DEF1_RA: u8 = 88;

// RA Tables
pub const RA_TABLES_SUBCLASS_PACK0_RA: u8 = 91;
pub const RA_TABLES_SUBCLASS_PACK1_RA: u8 = 92;
pub const RA_TABLES_SUBCLASS_RAM_RA_TABLE: u8 = 110;

// Calibration
pub const CALIBRATION_SUBCLASS_DATA: u8 = 104;
pub const CALIBRATION_SUBCLASS_TEMP_MODEL: u8 = 106;
pub const CALIBRATION_SUBCLASS_CURRENT: u8 = 107;

// Security Class
pub const SECURITY_SUBCLASS_CODES: u8 = 112;

// Charger class
pub const CHARGER_SUBCLASS_TEMPERATURE_TABLE: u8 = 74;
pub const CHARGER_SUBCLASS_CHARGER_INFO: u8 = 76;
pub const CHARGER_SUBCLASS_CHARGER_CONTROL_CONFIGURATION: u8 = 78;

/// Utility for reading and writing from the fuel gauge's data flash.
pub struct DataFlash<'a, I2C: I2c> {
    /// I2C device on the bus.
    device: &'a mut I2C,
}

impl<'a, I2C: I2c> DataFlash<'a, I2C> {
    pub fn new(device: &'a mut I2C) -> Self {
        Self { device }
    }

    pub async fn read_operation_configuration(&mut self) -> Result<[u8; 2], I2C::Error> {
        // Enable BlockDataControl() with command 0x61 to enable block data flash control.
        self.device.write(DEVICE_ADDRESS, &[0x61, 0x00]).await?;

        // Write the ID for the subclass we're going to access
        // with the DataFlashClass() command 0x3E.
        self.device
            .write(DEVICE_ADDRESS, &[0x3E, CONFIGURATION_SUBCLASS_REGISTERS])
            .await?;

        // Use command 0x3F to set the 32-bit offset we're going to read-write at in the subclass.
        //
        // "Occasionally, a data flash  class is larger than the 32-byte block size. In this case, the DataFlashBlock()
        // command designates in which 32-byte block the desired locations reside. The correct command address
        // is then given by 0x40 + offset modulo 32. For example, to access Terminate Voltage in the Gas Gauging
        // class, DataFlashClass() is issued 80 (0x50) to set the class. Because the offset is 50, it must reside in the
        // second 32-byte block. Hence, DataFlashBlock() is issued 0x01 to set the block offset, and the offset used
        // to index into the BlockData() memory area is 0x40 + 50 modulo 32 = 0x40 + 18 = 0x40 + 0x12 = 0x52."
        self.device.write(DEVICE_ADDRESS, &[0x3F, 0x00]).await?;

        // Set the bit offset in the block that we're reading from.
        let offset_address = 0x40 + (0 % 32);

        // Read the data for the bit we're interested in.
        let mut result = [0u8; 2];
        self.device
            .write_read(DEVICE_ADDRESS, &[offset_address], &mut result)
            .await?;

        // Read the checksum
        let mut checksum = [0u8; 1];
        self.device
            .write_read(DEVICE_ADDRESS, &[0x60], &mut checksum)
            .await?;

        // TODO: something with checksum?

        Ok(result)
    }

    /// Reads a single bit at the specified byte offset from the specified subclass.
    pub async fn read_byte(&mut self, subclass: u8, byte_offset: u8) -> Result<u8, I2C::Error> {
        // Enable BlockDataControl() with command 0x61 to enable block data flash control.
        self.device.write(DEVICE_ADDRESS, &[0x61, 0x00]).await?;

        // Write the ID for the subclass we're going to access
        // with the DataFlashClass() command 0x3E.
        self.device.write(DEVICE_ADDRESS, &[0x3E, subclass]).await?;

        // Use command 0x3F to set the 32-bit offset we're going to read-write at in the subclass.
        //
        // "Occasionally, a data flash  class is larger than the 32-byte block size. In this case, the DataFlashBlock()
        // command designates in which 32-byte block the desired locations reside. The correct command address
        // is then given by 0x40 + offset modulo 32. For example, to access Terminate Voltage in the Gas Gauging
        // class, DataFlashClass() is issued 80 (0x50) to set the class. Because the offset is 50, it must reside in the
        // second 32-byte block. Hence, DataFlashBlock() is issued 0x01 to set the block offset, and the offset used
        // to index into the BlockData() memory area is 0x40 + 50 modulo 32 = 0x40 + 18 = 0x40 + 0x12 = 0x52."
        self.device.write(DEVICE_ADDRESS, &[0x3F, 0x00]).await?;

        // Set the bit offset in the block that we're reading from.
        let offset_address = 0x40 + (byte_offset % 32); // 0x40 = 64

        // Read the data for the bit we're interested in.
        let mut result = [0u8; 1];
        self.device
            .write_read(DEVICE_ADDRESS, &[offset_address], &mut result)
            .await?;

        // MSB is the bit of interest.
        // TODO: use `isolate_highest_one` when out of nightly.
        //  https://github.com/rust-lang/rust/issues/136909
        Ok(result[0])
    }

    /// Reads a single bit at the specified offset from the Registers subclass.
    pub async fn read_registers_byte(&mut self, byte_offset: u8) -> Result<u8, I2C::Error> {
        self.read_byte(CONFIGURATION_SUBCLASS_REGISTERS, byte_offset)
            .await
    }

    /// Reads a single bit at the specified offset from
    /// the Operation Configuration flash register.
    pub async fn read_byte_bit(
        &mut self,
        subclass: u8,
        byte_offset: u8,
        bit: u8,
    ) -> Result<bool, I2C::Error> {
        // Enable BlockDataControl() with command 0x61 to enable block data flash control.
        self.device.write(DEVICE_ADDRESS, &[0x61, 0x00]).await?;

        // Write the ID for the subclass we're going to access
        // with the DataFlashClass() command 0x3E.
        self.device.write(DEVICE_ADDRESS, &[0x3E, subclass]).await?;

        // Use command 0x3F to set the 32-bit offset we're going to read-write at in the subclass.
        //
        // "Occasionally, a data flash  class is larger than the 32-byte block size. In this case, the DataFlashBlock()
        // command designates in which 32-byte block the desired locations reside. The correct command address
        // is then given by 0x40 + offset modulo 32. For example, to access Terminate Voltage in the Gas Gauging
        // class, DataFlashClass() is issued 80 (0x50) to set the class. Because the offset is 50, it must reside in the
        // second 32-byte block. Hence, DataFlashBlock() is issued 0x01 to set the block offset, and the offset used
        // to index into the BlockData() memory area is 0x40 + 50 modulo 32 = 0x40 + 18 = 0x40 + 0x12 = 0x52."
        self.device.write(DEVICE_ADDRESS, &[0x3F, 0x00]).await?;

        // Set the bit offset in the block that we're reading from.
        let offset_address = 0x40 + (byte_offset % 32); // 0x40 = 64

        // Read the data for the bit we're interested in.
        let mut result = [0u8; 1];
        self.device
            .write_read(DEVICE_ADDRESS, &[offset_address], &mut result)
            .await?;

        //
        Ok(result[0] & (1 << bit) != 0)
    }

    /// Updates the value of a specific bit in the Operation Configuration data flash register.
    pub async fn write_byte(
        &mut self,
        subclass: u8,
        byte_offset: u8,
        value: u8,
    ) -> Result<(), I2C::Error> {
        // Enable BlockDataControl() with command 0x61 to enable block data flash control.
        self.device.write(DEVICE_ADDRESS, &[0x61, 0x00]).await?;

        // Write the ID for the subclass we're going to access
        // with the DataFlashClass() command 0x3E.
        self.device.write(DEVICE_ADDRESS, &[0x3E, subclass]).await?;

        // Use command 0x3F to set the 32-bit offset we're going to read-write at in the subclass.
        //
        // "Occasionally, a data flash  class is larger than the 32-byte block size. In this case, the DataFlashBlock()
        // command designates in which 32-byte block the desired locations reside. The correct command address
        // is then given by 0x40 + offset modulo 32. For example, to access Terminate Voltage in the Gas Gauging
        // class, DataFlashClass() is issued 80 (0x50) to set the class. Because the offset is 50, it must reside in the
        // second 32-byte block. Hence, DataFlashBlock() is issued 0x01 to set the block offset, and the offset used
        // to index into the BlockData() memory area is 0x40 + 50 modulo 32 = 0x40 + 18 = 0x40 + 0x12 = 0x52."
        self.device.write(DEVICE_ADDRESS, &[0x3F, 0x00]).await?;

        // Set the bit offset in the block that we're reading from.
        let offset_address = 0x40 + (byte_offset % 32);

        // Read the data for the bit we're interested in.
        let mut old_value = [0u8; 1];
        self.device
            .write_read(DEVICE_ADDRESS, &[offset_address], &mut old_value)
            .await?;

        // Read the block checksum.
        let mut old_checksum = [0u8; 1];
        self.device
            .write_read(DEVICE_ADDRESS, &[0x60], &mut old_checksum)
            .await?;

        // Write the data for the bit we're interested in.
        self.device
            .write(DEVICE_ADDRESS, &[offset_address, value])
            .await?;

        // Calculate the new checksum.
        let temp = ((255 as i16 - old_checksum[0] as i16 - old_value[0] as i16) % 256) as u8;
        let checksum = (255 - ((temp as u16 + value as u16) % 256)) as u8;

        // Write the new checksum to save the new flash value.
        self.device.write(DEVICE_ADDRESS, &[0x60, checksum]).await?;

        // Reset the gauge so the new parameter takes effect.
        self.device
            .write(DEVICE_ADDRESS, &[0x00, 0x41, 0x00])
            .await?;

        Ok(())
    }

    /// Updates the value of a specific bit in the Operation Configuration data flash register.
    pub async fn write_u16(
        &mut self,
        subclass: u8,
        byte_offset: u8,
        value: u16,
    ) -> Result<(), I2C::Error> {
        // Enable BlockDataControl() with command 0x61 to enable block data flash control.
        self.device.write(DEVICE_ADDRESS, &[0x61, 0x00]).await?;

        // Write the ID for the subclass we're going to access
        // with the DataFlashClass() command 0x3E.
        self.device.write(DEVICE_ADDRESS, &[0x3E, subclass]).await?;

        // Use command 0x3F to set the 32-bit offset we're going to read-write at in the subclass.
        //
        // "Occasionally, a data flash  class is larger than the 32-byte block size. In this case, the DataFlashBlock()
        // command designates in which 32-byte block the desired locations reside. The correct command address
        // is then given by 0x40 + offset modulo 32. For example, to access Terminate Voltage in the Gas Gauging
        // class, DataFlashClass() is issued 80 (0x50) to set the class. Because the offset is 50, it must reside in the
        // second 32-byte block. Hence, DataFlashBlock() is issued 0x01 to set the block offset, and the offset used
        // to index into the BlockData() memory area is 0x40 + 50 modulo 32 = 0x40 + 18 = 0x40 + 0x12 = 0x52."
        self.device.write(DEVICE_ADDRESS, &[0x3F, 0x00]).await?;

        // Set the bit offset in the block that we're reading from.
        let offset_address = 0x40 + (byte_offset % 32);

        let bytes = value.to_le_bytes();

        // Read the data for the bit we're interested in.
        let mut old_value = [0u8; 2];
        self.device
            .write_read(DEVICE_ADDRESS, &[offset_address], &mut old_value)
            .await?;

        // Read the block checksum.
        let mut old_checksum = [0u8; 1];
        self.device
            .write_read(DEVICE_ADDRESS, &[0x60], &mut old_checksum)
            .await?;

        // Write the data for the bit we're interested in.
        self.device
            .write(DEVICE_ADDRESS, &[offset_address, bytes[0], bytes[1]])
            .await?;

        // Calculate the new checksum.
        let temp = ((255 as i16 - old_checksum[0] as i16 - old_value[0] as i16) % 256) as u8;
        let checksum = (255 - ((temp as u16 + bytes[0] as u16) % 256)) as u8;
        let temp = ((255 as i16 - checksum as i16 - old_value[1] as i16) % 256) as u8;
        let checksum = (255 - ((temp as u16 + bytes[1] as u16) % 256)) as u8;

        // Write the new checksum to save the new flash value.
        self.device.write(DEVICE_ADDRESS, &[0x60, checksum]).await?;

        // Reset the gauge so the new parameter takes effect.
        self.device
            .write(DEVICE_ADDRESS, &[0x00, 0x41, 0x00])
            .await?;

        Ok(())
    }

    /// Updates the value of a specific bit in the Operation Configuration data flash register.
    pub async fn write_byte_bit(
        &mut self,
        subclass: u8,
        byte_offset: u8,
        bit: u8,
        value: bool,
    ) -> Result<(), I2C::Error> {
        // Enable BlockDataControl() with command 0x61 to enable block data flash control.
        self.device.write(DEVICE_ADDRESS, &[0x61, 0x00]).await?;

        // Write the ID for the subclass we're going to access
        // with the DataFlashClass() command 0x3E.
        self.device.write(DEVICE_ADDRESS, &[0x3E, subclass]).await?;

        // Use command 0x3F to set the 32-bit offset we're going to read-write at in the subclass.
        //
        // "Occasionally, a data flash  class is larger than the 32-byte block size. In this case, the DataFlashBlock()
        // command designates in which 32-byte block the desired locations reside. The correct command address
        // is then given by 0x40 + offset modulo 32. For example, to access Terminate Voltage in the Gas Gauging
        // class, DataFlashClass() is issued 80 (0x50) to set the class. Because the offset is 50, it must reside in the
        // second 32-byte block. Hence, DataFlashBlock() is issued 0x01 to set the block offset, and the offset used
        // to index into the BlockData() memory area is 0x40 + 50 modulo 32 = 0x40 + 18 = 0x40 + 0x12 = 0x52."
        self.device.write(DEVICE_ADDRESS, &[0x3F, 0x00]).await?;

        // Set the bit offset in the block that we're reading from.
        let offset_address = 0x40 + (byte_offset % 32);

        // Read the data for the bit we're interested in.
        let mut old_value = [0u8; 1];
        self.device
            .write_read(DEVICE_ADDRESS, &[offset_address], &mut old_value)
            .await?;

        // Read the block checksum.
        let mut old_checksum = [0u8; 1];
        self.device
            .write_read(DEVICE_ADDRESS, &[0x60], &mut old_checksum)
            .await?;

        // Update the byte with the new bit value.
        let new_byte = (old_value[0] & !(1 << bit)) | (value as u8 & (1 << bit));

        // Write the data for the bit we're interested in.
        self.device
            .write(DEVICE_ADDRESS, &[offset_address, new_byte])
            .await?;

        // Calculate the new checksum.
        let temp = ((255 as i16 - old_checksum[0] as i16 - old_value[0] as i16) % 256) as u8;
        let checksum = (255 - ((temp as u16 + new_byte as u16) % 256)) as u8;

        // Write the new checksum to save the new flash value.
        self.device.write(DEVICE_ADDRESS, &[0x60, checksum]).await?;

        // Reset the gauge so the new parameter takes effect.
        self.device
            .write(DEVICE_ADDRESS, &[0x00, 0x41, 0x00])
            .await?;

        Ok(())
    }
}
