use proc_bitfield::bitfield;

bitfield! {
    /// The MPU_RASR defines the region size and memory attributes of the MPU region
    /// specified by the MPU_RNR, and enables that region and any subregions.
    pub struct RegionAttributeSizeRegister (pub u32): Debug, FromStorage, IntoStorage {
        /// Instruction access disable bit:
        /// 0 = instruction fetches enabled
        /// 1 = instruction fetches disabled.
        pub instruction_access_disabled: bool @ 28,

        /// Access permission field.
        ///
        /// see: https://developer.arm.com/documentation/dui0552/a/cortex-m3-peripherals/optional-memory-protection-unit/mpu-access-permission-attributes?lang=en
        pub access_permission: u8 @ 24..=26,

        /// Memory access attributes.
        ///
        /// see: https://developer.arm.com/documentation/dui0552/a/cortex-m3-peripherals/optional-memory-protection-unit/mpu-access-permission-attributes?lang=en
        pub memory_access_attribute_tex: u8 @ 19..=21,
        pub memory_access_attribute_c: bool @ 17,
        pub memory_access_attribute_b: bool @ 16,

        /// Shareable bit.
        ///
        /// see: https://developer.arm.com/documentation/dui0552/a/cortex-m3-peripherals/optional-memory-protection-unit/mpu-access-permission-attributes?lang=en
        pub shareable: bool @ 18,

        /// Subregion disable bits. For each bit in this field:
        ///  0 = corresponding sub-region is enabled
        ///  1 = corresponding sub-region is disabled.
        /// Region sizes of 128 bytes and less do not support subregions.
        /// When writing the attributes for such a region, write the SRD field as 0x00.
        /// see: https://developer.arm.com/documentation/dui0552/a/cortex-m3-peripherals/optional-memory-protection-unit/updating-an-mpu-region?lang=en
        pub subregion_disable: u8 @ 8..=15,

        /// Specifies the size of the MPU protection region.
        ///
        /// The minimum permitted value is 3 (0b00010).
        ///
        /// Example sizes:
        /// 0b00100 (4)	    32B	5	Minimum permitted size
        /// 0b01001 (9)	    1KB	10	-
        /// 0b10011 (19)	1MB	20	-
        /// 0b11101 (29)	1GB	30	-
        /// 0b11111 (31)	4GB	32	Maximum possible size
        ///
        /// see: https://developer.arm.com/documentation/dui0552/a/cortex-m3-peripherals/optional-memory-protection-unit/mpu-region-attribute-and-size-register?lang=en
        pub size: u8 @ 1..=5,

        /// Region enable bit.
        pub enable: bool @ 0
    }
}
