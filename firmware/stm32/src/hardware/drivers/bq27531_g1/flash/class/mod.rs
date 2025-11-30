use embedded_hal_async::i2c::{I2c, SevenBitAddress};

use crate::hardware::drivers::bq27531_g1::DEVICE_ADDRESS;

pub trait Class<'a, I2C: I2c> {}

pub async fn read_subclass<const N: usize, I2C: I2c>(
    device: &'_ mut I2C,
    subclass: u8,
    offset: u8,
) -> Result<[u8; N], I2C::Error> {
    // Enable BlockDataControl() with command 0x61 to enable block data flash control.
    device.write(DEVICE_ADDRESS, &[0x61, 0x00]).await?;

    // Write the ID for the subclass we're going to access
    // with the DataFlashClass() command 0x3E.
    device.write(DEVICE_ADDRESS, &[0x3E, subclass]).await?;

    // Use command 0x3F to set the 32-bit offset we're going to read-write at in the subclass.
    //
    // "Occasionally, a data flash  class is larger than the 32-byte block size. In this case, the DataFlashBlock()
    // command designates in which 32-byte block the desired locations reside. The correct command address
    // is then given by 0x40 + offset modulo 32. For example, to access Terminate Voltage in the Gas Gauging
    // class, DataFlashClass() is issued 80 (0x50) to set the class. Because the offset is 50, it must reside in the
    // second 32-byte block. Hence, DataFlashBlock() is issued 0x01 to set the block offset, and the offset used
    // to index into the BlockData() memory area is 0x40 + 50 modulo 32 = 0x40 + 18 = 0x40 + 0x12 = 0x52."
    device.write(DEVICE_ADDRESS, &[0x3F, 0x00]).await?;

    // Set the bit offset in the block that we're reading from.
    let offset_address = 0x40 + (offset % 32);

    // Read the data for the bit we're interested in.
    let mut result = [0u8; N];
    device
        .write_read(DEVICE_ADDRESS, &[0x4B, offset_address], &mut result)
        .await?;

    // TODO: read and check checksum

    Ok(result)
}

pub mod configuration;
