use core::ptr;

use defmt::{Format, trace, warn};

use stm32_metapac::{
    self as pac,
    fmc::{self, vals::Accmod},
};

/// Specifies the NOR/SRAM memory device that will be used.
#[derive(Clone, Copy, Debug, PartialEq, defmt::Format)]
#[allow(unused)]
pub enum FMCRegion {
    /// Targeting the 1st SRAM bank
    ///
    /// Uses NE1 chip select.
    Bank1,
    /// Targeting the 2nd SRAM bank
    ///
    /// Uses NE2 chip select.
    Bank2,
    /// Targeting the 3nd SRAM bank
    ///
    /// Uses NE3 chip select.
    Bank3,
    /// Targeting the 4nd SRAM bank
    ///
    /// Uses NE4 chip select.
    Bank4,

    // ====================
    // BMAP 01 remapping
    /// Targeting the 1st SRAM bank
    ///
    /// Uses NE1 chip select.
    Bank1Remapped,
    /// Targeting the 2nd SRAM bank
    ///
    /// Uses NE2 chip select.
    Bank2Remapped,
    /// Targeting the 3nd SRAM bank
    ///
    /// Uses NE3 chip select.
    Bank3Remapped,
    /// Targeting the 4nd SRAM bank
    ///
    /// Uses NE4 chip select.
    Bank4Remapped,
}

impl FMCRegion {
    pub const fn base_address(&self) -> u32 {
        match self {
            FMCRegion::Bank1 => 0x60000000,
            FMCRegion::Bank2 => 0x64000000,
            FMCRegion::Bank3 => 0x68000000,
            FMCRegion::Bank4 => 0x6C000000,

            FMCRegion::Bank1Remapped => 0xC0000000,
            FMCRegion::Bank2Remapped => 0xC4000000,
            FMCRegion::Bank3Remapped => 0xC8000000,
            FMCRegion::Bank4Remapped => 0xCC000000,
        }
    }

    pub const fn min_address(&self) -> u32 {
        self.base_address()
    }

    pub const fn size(&self) -> u32 {
        0x3FFFFFF
    }

    pub fn max_address(&self) -> u32 {
        self.min_address() + self.size()
    }

    /// Maps a local memory address to what the
    /// remote will see based on the bank bit depth.
    ///
    /// RM0433 Rev 8 P.g. 803
    pub fn local_address_to_remote(
        &self,
        remote_address: u32,
        width: MemoryDataWidth,
    ) -> Result<u32, ()> {
        if remote_address < self.min_address() || remote_address >= self.max_address() {
            return Err(());
        }

        match width {
            MemoryDataWidth::Bits8 => Ok(0x3FFFFFF & remote_address), // ADDR[25:0] 00000011111111111111111111111111
            MemoryDataWidth::Bits16 => Ok((0x3FFFFFE & remote_address) >> 1), // (ADDR[25:1] >> 1) 00000011111111111111111111111110
            MemoryDataWidth::Bits32 => Ok((0x3FFFFFC & remote_address) >> 2), // (ADDR[25:2] >> 2) 00000011111111111111111111111100
        }
    }

    pub fn addr(&self) -> u32 {
        self.min_address()
    }
}

/// Like `modfiy_reg`, but applies to bank 1 or 2 based on a varaiable
macro_rules! modify_reg_banked3 {
    ( $periph:path, $instance:expr, $bank:expr, $reg2:ident, $reg3:ident, $reg4:ident, $( $field:ident : $value:expr ),+ ) => {{
        use FMCRegion::*;

        match $bank {
            Bank2 => modify_reg!( $periph, $instance, $reg2, $( $field : $value ),*),
            Bank3 => modify_reg!( $periph, $instance, $reg3, $( $field : $value ),*),
            Bank4 => modify_reg!( $periph, $instance, $reg4, $( $field : $value ),*),
            _ => panic!(),
        }
    }};
}

/// Like `modfiy_reg`, but applies to bank 1 or 2 based on a varaiable
macro_rules! modify_reg_banked4 {
    ( $periph:path, $instance:expr, $bank:expr, $reg1:ident, $reg2:ident, $reg3:ident, $reg4:ident, $( $field:ident : $value:expr ),+ ) => {{
        use FMCRegion::*;

        match $bank {
            Bank1 => modify_reg!( $periph, $instance, $reg1, $( $field : $value ),*),
            Bank2 => modify_reg!( $periph, $instance, $reg2, $( $field : $value ),*),
            Bank3 => modify_reg!( $periph, $instance, $reg3, $( $field : $value ),*),
            Bank4 => modify_reg!( $periph, $instance, $reg4, $( $field : $value ),*),
            _ => panic!(),
        }
    }};
}

/// Specifies the type of external memory attached to
/// the corresponding memory device.
///
/// Register: BCR1/2/3/4[MTYP]
#[derive(PartialEq, Debug, Format)]
pub enum MemoryType {
    /// SRAM memory
    Sram,
    /// PSRAM (CRAM) memory
    Psram,
    /// NOR Flash/OneNAND Flash
    Flash,
}

/// Specifies the external memory device width.
///
/// Register: BCR1/2/3/4[MWID]
pub enum MemoryDataWidth {
    Bits8,
    Bits16,
    Bits32,
}

/// Specifies the wait signal polarity, valid only when accessing
/// the Flash memory in burst mode.
///
/// Register: BCR1/2/3/4[WAITPOL]
pub enum WaitSignalPolarity {
    ActiveLow,
    ActiveHigh,
}

/// Specifies if the wait signal is asserted by the memory one
/// clock cycle before the wait state or during the wait state,
/// valid only when accessing memories in burst mode.
///
/// Register: BCR1/2/3/4[WAITCFG]
pub enum WaitSignalActive {
    BeforeWaitState,
    DuringWaitState,
}

/// Specifies the memory page size.
///
/// Register: BCR1/2/3/4[CPSIZE]
pub enum PageSize {
    NoBurstSplit,
    Bytes128,
    Bytes256,
    Bytes512,
    Bytes1024,
}

/// See RM0433 Rev 8 P.g. 813 for timing diagrams based on these settings.
pub struct Config {
    /// Specifies whether the address and data values are
    /// multiplexed on the data bus or not.
    ///
    /// Register: BCR1/2/3/4[MUXEN]
    data_address_mux_enabled: bool, // MUXEN

    /// Specifies the type of external memory attached to
    /// the corresponding memory device.
    memory_type: MemoryType, // MTYP

    /// Specifies the external memory device width.
    memory_data_width: MemoryDataWidth, // MWID

    /// Enables or disables the burst access mode for Flash memory,
    /// valid only with synchronous burst Flash memories.
    ///
    /// Register: BCR1/2/3/4[BURSTEN]
    burst_access_mode_enable: bool, // BURSTEN

    /// Specifies the wait signal polarity, valid only when accessing
    /// the Flash memory in burst mode.
    wait_signal_enable_polarity: WaitSignalPolarity, // WAITPOL

    /// Specifies if the wait signal is asserted by the memory one
    /// clock cycle before the wait state or during the wait state,
    /// valid only when accessing memories in burst mode.
    wait_signal_enable_active: WaitSignalActive, // WAITCFG

    /// Enables or disables the write operation in the selected device by the FMC.
    ///
    /// Register: BCR1/2/3/4[WREN]
    write_enable: bool, // WREN

    /// Enables or disables the wait state insertion via wait
    /// signal, valid for Flash memory access in burst mode.
    ///
    /// Register: BCR1/2/3/4[WAITEN]
    wait_signal_enable: bool, // WAITEN

    /// Enables or disables the extended mode.
    ///
    /// Register: BCR1/2/3/4[EXTMOD]
    extended_mode: bool, // EXTMOD

    /// Enables or disables wait signal during asynchronous transfers,
    /// valid only with asynchronous Flash memories.
    ///
    /// Register: BCR1/2/3/4[ASYNCWAIT]
    asynchronous_wait: bool, // ASYNCWAIT

    /// Enables or disables the write burst operation.
    ///
    /// Register: BCR1/2/3/4[CBURSTRW]
    write_burst_enable: bool, // CBURSTRW

    /// Enables or disables the FMC clock output to external memory devices.
    /// This parameter is only enabled through the FMC_BCR1 register,
    /// and don't care through FMC_BCR2..4 registers.
    ///
    /// Register: BCR1[CCLKEN]
    continuous_clock_enable: bool, // CCLKEN

    /// Enables or disables the write FIFO used by the FMC controller.
    /// This parameter is only enabled through the FMC_BCR1 register,
    /// and don't care through FMC_BCR2..4 registers.
    ///
    /// Register: BCR1[WFDIS]
    write_fifo_disable: bool, // WFDIS

    // Specifies the memory page size.
    page_size: PageSize, // CPSIZE
}

impl Config {
    /// Constructs a new config with LCD defaults used by STM32CubeIDE.
    pub fn new_ili9341_parallel_i(
        // byte width of the display data.
        //
        // I.e. 16-bit RB565 would be 16-bit
        memory_width: MemoryDataWidth,
    ) -> Self {
        /*modify_reg_banked!(fmc, self.regs.global(),
                self.bank, BCR1, BCR2,
                CCLKEN: Enabled, // Continous clock enabled - FMC_CONTINUOUS_CLOCK_SYNC_ONLY
                CBURSTRW:, // WriteBurst?
                ASYNCWAIT: Disabled, // Asyncronous wait - FMC_ASYNCHRONOUS_WAIT_DISABLE
                EXTMOD: Disabled, // ExtendedMode - FMC_EXTENDED_MODE_DISABLE
                WAITEN: Disabled, // Wait signal - FMC_WAIT_SIGNAL_DISABLE
                WREN: Enabled, // Write operation - FMC_WRITE_OPERATION_ENABLE
                WAITCFG: BeforeWaitState, // Wait signal active - FMC_WAIT_TIMING_BEFORE_WS
                WAITPOL: ActiveLow, // Wait signal polarity - FMC_WAIT_SIGNAL_POLARITY_LOW
                BURSTEN: Disabled, // Burst access mode - FMC_BURST_ACCESS_MODE_DISABLE
                FACCEN:, // NOR Flash memory access
                MWID: Bits16, // Memory data width - FMC_NORSRAM_MEM_BUS_WIDTH_16
                MTYP: SRAM, // Memory types - FMC_MEMORY_TYPE_SRAM
                MUXEN: Disabled, // Data address mux - FMC_DATA_ADDRESS_MUX_DISABLE
                MBKEN:, // memory bank enable/disable
                WRAPMOD:,
                WFDIS: Enabled, // Write FIFO Disable - FMC_WRITE_FIFO_ENABLE
                CPSIZE: NoBurstSplit, // Page size - FMC_PAGE_SIZE_NONE
        );*/
        Self {
            data_address_mux_enabled: false, // DataAddressMux=FMC_DATA_ADDRESS_MUX_DISABLE
            memory_type: MemoryType::Sram,   // MemoryType=FMC_MEMORY_TYPE_SRAM
            memory_data_width: memory_width, // MemoryDataWidth=FMC_NORSRAM_MEM_BUS_WIDTH_16
            burst_access_mode_enable: false, // BurstAccessMode=FMC_BURST_ACCESS_MODE_DISABLE
            wait_signal_enable_polarity: WaitSignalPolarity::ActiveLow, // WaitSignalPolarity=FMC_WAIT_SIGNAL_POLARITY_LOW
            wait_signal_enable_active: WaitSignalActive::BeforeWaitState, // WaitSignalActive=FMC_WAIT_TIMING_BEFORE_WS
            write_enable: true, // WriteOperation=FMC_WRITE_OPERATION_ENABLE
            // This ILI9341 doesn't use a wait signal.
            wait_signal_enable: false, // WaitSignal=FMC_WAIT_SIGNAL_DISABLE
            extended_mode: false,      // ExtendedMode=FMC_EXTENDED_MODE_DISABLE
            asynchronous_wait: false,  // AsynchronousWait=FMC_ASYNCHRONOUS_WAIT_DISABLE
            // TODO: should this be true?
            write_burst_enable: false, // WriteBurst=FMC_WRITE_BURST_DISABLE
            // We only want write/read clock signals when actually
            // reading/writing, so disable the continuous clock.
            continuous_clock_enable: false, // ContinuousClock=FMC_CONTINUOUS_CLOCK_SYNC_ONLY
            write_fifo_disable: true,       // WriteFifo=FMC_WRITE_FIFO_ENABLE
            page_size: PageSize::NoBurstSplit, // PageSize=FMC_PAGE_SIZE_NONE
        }
    }
}

type AddressSetupTime = u8;
type AddressHoldTime = u8;
type DataSetupTime = u8;
type BusTurnAroundDuration = u8;
type CLKDivision = u8;
type DataLatency = u8;

pub enum AccessMode {
    AccessModeA,
    AccessModeB,
    AccessModeC,
    AccessModeD,
}

impl Into<Accmod> for AccessMode {
    fn into(self) -> Accmod {
        match self {
            AccessMode::AccessModeA => Accmod::A,
            AccessMode::AccessModeB => Accmod::B,
            AccessMode::AccessModeC => Accmod::C,
            AccessMode::AccessModeD => Accmod::D,
        }
    }
}

pub struct Timing {
    /// Defines the number of HCLK cycles to configure
    /// the duration of the address setup time.
    /// This parameter can be a value between Min_Data = 0 and Max_Data = 15.
    ///
    /// Duration of the first access phase (ADDSET fmc_ker_ck cycles).
    /// Minimum value for ADDSET is 0
    ///
    /// This parameter is not used with synchronous NOR Flash memories.
    address_setup_time: AddressSetupTime, // addset

    /// Defines the number of HCLK cycles to configure
    /// the duration of the address hold time.
    /// This parameter can be a value between Min_Data = 1 and Max_Data = 15.
    ///
    ///  This parameter is not used with synchronous NOR Flash memories.
    address_hold_time: AddressHoldTime, // addhld

    /// Defines the number of HCLK cycles to configure
    /// the duration of the data setup time.
    /// This parameter can be a value between Min_Data = 1 and Max_Data = 255.
    ///
    /// Duration of the second access phase (DATAST+1 fmc_ker_ck cycles
    /// for write accesses, DATAST fmc_ker_ck cycles for read accesses).
    ///
    /// This parameter is used for SRAMs, ROMs and asynchronous multiplexed NOR Flash memories.
    data_setup_time: DataSetupTime, // datast

    /// Defines the number of HCLK cycles to configure
    /// the duration of the bus turnaround.
    /// This parameter can be a value between Min_Data = 0 and Max_Data = 15.
    ///
    /// Time between NEx high to NEx low (BUSTURN fmc_ker_ck)
    ///
    /// This parameter is only used for multiplexed NOR Flash memories.
    bus_turn_around_duration: BusTurnAroundDuration, // busturn

    /// Defines the period of CLK clock output signal, expressed in number of
    /// HCLK cycles. This parameter can be a value between Min_Data = 2 and
    /// Max_Data = 16.
    ///
    /// This parameter is not used for asynchronous NOR Flash, SRAM or ROM accesses.
    clock_division: CLKDivision, // clkdiv

    /// Defines the number of memory clock cycles to issue
    /// to the memory before getting the first data.
    /// The parameter value depends on the memory type as shown below:
    /// - It must be set to 0 in case of a CRAM
    /// - It is don't care in asynchronous NOR, SRAM or ROM accesses
    /// - It may assume a value between Min_Data = 2 and Max_Data = 17
    /// in NOR Flash memories with synchronous burst mode enable.
    data_latency: DataLatency, // datlat

    /// Specifies the asynchronous access mode.
    access_mode: AccessMode, // accmod
}

impl Timing {
    /// Constructs a set of timing defaults used by STM32CuteIDE for LCD interfaces.
    pub fn new_lcd() -> Self {
        Self {
            address_setup_time: 15,
            address_hold_time: 15,
            data_setup_time: 255,
            bus_turn_around_duration: 15,
            clock_division: 16,
            data_latency: 17,
            access_mode: AccessMode::AccessModeA,
        }
    }

    /// https://www.buydisplay.com/download/ic/ILI9341.pdf (P.g. 233).
    pub fn new_ili9341_parallel_i() -> Self {
        Self {
            address_setup_time: 0,                // tast
            address_hold_time: 1,                 // taht - should be 0, but FMC doesn't support 1?
            data_setup_time: 10,                  // trod - can probably shorted to 10 (tdst)
            bus_turn_around_duration: 15,         // unused by sram
            clock_division: 2,                    // not used? should be a way to set clock..
            data_latency: 0,                      // unused by sram
            access_mode: AccessMode::AccessModeA, // TODO: not sure of type to select
        }
    }
}

pub struct Fmc<'a> {
    fmc: pac::fmc::Fmc,
    bank: FMCRegion,
    config: Config,
    timing: Timing,
    memory: &'a mut [u16],
}

#[derive(Debug)]
pub enum InitError {
    InvalidConfig,
    InvalidTiming(TimingError),
}

impl From<TimingError> for InitError {
    fn from(value: TimingError) -> Self {
        InitError::InvalidTiming(value)
    }
}

#[derive(Debug)]

pub enum TimingError {
    InvalidAddressSetupTime,
    InvalidAddressHoldTime,
    InvalidDataSetupTime,
    InvalidDataHoldDuration,
    InvalidTurnaroundTime,
    InvalidClockDivisor,
}

impl<'a> Fmc<'a> {
    /// Note that the total size of each NOR/SRAM bank is 64Mbytes. (RM0433 Rev 8 P.g. 808)
    /// The maximum capacity is 512 Mbits (26 address lines).  (RM0433 Rev 8 P.g. 809)
    pub fn new(fmc: pac::fmc::Fmc, bank: FMCRegion, config: Config, timing: Timing) -> Self {
        let memory: &mut [u16] = unsafe {
            // Initialise controller and SDRAM
            let ram_ptr: *mut u16 = bank.addr() as *mut _;

            // Convert raw pointer to slice
            core::slice::from_raw_parts_mut(ram_ptr, 2)
        };

        Self {
            fmc,
            bank,
            config,
            timing,
            memory,
        }
    }

    /// Initializes the FMC configuration register corresponding to the configured bank.
    pub fn config_init(&mut self) -> Result<(), InitError> {
        if self.config.continuous_clock_enable == true && self.bank != FMCRegion::Bank1 {
            // See STM32 HAL for implementation detail.
            warn!(
                "Bank 1 continous clocks needs to be in synchronous mode when continous clock is enabled for banks 2, 3, and 4."
            );
        }

        if self.bank != FMCRegion::Bank1 {
            // See STM32 HAL for implementation detail.
            warn!("Write FIFO should be configured on bank 1 when enabled on banks 2, 3, and 4.")
        }

        // RM0433 Rev 8 (P.g. 838)
        if self.bank == FMCRegion::Bank2Remapped
            || self.bank == FMCRegion::Bank3Remapped
            || self.bank == FMCRegion::Bank4Remapped
        {
            warn!(
                "Ensure that FMC BCR bank 1 is remapped to allow remapped addresses of 2, 3, and 4."
            )
        }

        let mut norsram_flash_access_enable = false;
        if self.config.memory_type == MemoryType::Flash {
            norsram_flash_access_enable = true;
        }

        if self.bank == FMCRegion::Bank1 || self.bank == FMCRegion::Bank1Remapped {
            // SRAM/NOR-Flash chip-select control register
            self.fmc.bcr1().modify::<Result<(), InitError>>(|bcr| {
                // Remap the address space for the entire SRAM bank.
                if self.bank == FMCRegion::Bank1Remapped {
                    bcr.set_bmap(0b01); // NOR/PSRAM bank and SDRAM bank 1/bank2 are swapped
                }

                bcr.set_cclken(self.config.continuous_clock_enable);

                bcr.set_cburstrw(self.config.write_burst_enable);
                // tODO: is true enabled?
                bcr.set_asyncwait(self.config.asynchronous_wait); // Asyncronous wait
                bcr.set_extmod(self.config.extended_mode); // Extended mode
                bcr.set_waiten(self.config.wait_signal_enable); // Wait signal
                bcr.set_wren(self.config.write_enable); // Write operation
                bcr.set_waitcfg(match self.config.wait_signal_enable_active {
                    WaitSignalActive::BeforeWaitState => fmc::vals::Waitcfg::BEFORE_WAIT_STATE,
                    WaitSignalActive::DuringWaitState => fmc::vals::Waitcfg::DURING_WAIT_STATE,
                }); // Wait signal active

                bcr.set_waitpol(match self.config.wait_signal_enable_polarity {
                    WaitSignalPolarity::ActiveLow => fmc::vals::Waitpol::ACTIVE_LOW,
                    WaitSignalPolarity::ActiveHigh => fmc::vals::Waitpol::ACTIVE_HIGH,
                }); // Wait signal polarity

                bcr.set_bursten(self.config.burst_access_mode_enable); // Burst access mode

                bcr.set_faccen(norsram_flash_access_enable); // NOR Flash memory access 

                bcr.set_mwid(match self.config.memory_data_width {
                    MemoryDataWidth::Bits8 => fmc::vals::Mwid::BITS8,
                    MemoryDataWidth::Bits16 => fmc::vals::Mwid::BITS16,
                    MemoryDataWidth::Bits32 => fmc::vals::Mwid::BITS32,
                }); // Memory data width

                bcr.set_mtyp(match self.config.memory_type {
                    MemoryType::Sram => fmc::vals::Mtyp::SRAM,
                    MemoryType::Psram => fmc::vals::Mtyp::PSRAM,
                    MemoryType::Flash => fmc::vals::Mtyp::FLASH,
                }); // Memory types

                bcr.set_muxen(self.config.data_address_mux_enabled); // Data address mux

                bcr.set_wfdis(self.config.write_fifo_disable);

                bcr.set_cpsize(match self.config.page_size {
                    PageSize::NoBurstSplit => fmc::vals::Cpsize::NO_BURST_SPLIT,
                    PageSize::Bytes128 => fmc::vals::Cpsize::BYTES128,
                    PageSize::Bytes256 => fmc::vals::Cpsize::BYTES256,
                    PageSize::Bytes512 => fmc::vals::Cpsize::BYTES512,
                    PageSize::Bytes1024 => fmc::vals::Cpsize::BYTES1024,
                });

                trace!("fmc bcr: {}", bcr);

                Ok(())
            })?;
        } else {
            self.fmc
                .bcr(match self.bank {
                    FMCRegion::Bank1 | FMCRegion::Bank1Remapped => panic!("invalid"),
                    FMCRegion::Bank2 | FMCRegion::Bank2Remapped => 0,
                    FMCRegion::Bank3 | FMCRegion::Bank3Remapped => 1,
                    FMCRegion::Bank4 | FMCRegion::Bank4Remapped => 2,
                })
                .modify::<Result<(), InitError>>(|bcr| {
                    bcr.set_cburstrw(self.config.write_burst_enable);
                    bcr.set_asyncwait(self.config.asynchronous_wait); // Asyncronous wait
                    bcr.set_extmod(self.config.extended_mode); // Extended mode
                    bcr.set_waiten(self.config.wait_signal_enable); // Wait signal
                    bcr.set_wren(self.config.write_enable); // Write operation
                    bcr.set_waitcfg(match self.config.wait_signal_enable_active {
                        WaitSignalActive::BeforeWaitState => fmc::vals::Waitcfg::BEFORE_WAIT_STATE,
                        WaitSignalActive::DuringWaitState => fmc::vals::Waitcfg::DURING_WAIT_STATE,
                    }); // Wait signal active

                    bcr.set_waitpol(match self.config.wait_signal_enable_polarity {
                        WaitSignalPolarity::ActiveLow => fmc::vals::Waitpol::ACTIVE_LOW,
                        WaitSignalPolarity::ActiveHigh => fmc::vals::Waitpol::ACTIVE_HIGH,
                    }); // Wait signal polarity

                    bcr.set_bursten(self.config.burst_access_mode_enable); // Burst access mode

                    bcr.set_faccen(norsram_flash_access_enable); // NOR Flash memory access 

                    bcr.set_mwid(match self.config.memory_data_width {
                        MemoryDataWidth::Bits8 => fmc::vals::Mwid::BITS8,
                        MemoryDataWidth::Bits16 => fmc::vals::Mwid::BITS16,
                        MemoryDataWidth::Bits32 => fmc::vals::Mwid::BITS32,
                    }); // Memory data width

                    bcr.set_mtyp(match self.config.memory_type {
                        MemoryType::Sram => fmc::vals::Mtyp::SRAM,
                        MemoryType::Psram => fmc::vals::Mtyp::PSRAM,
                        MemoryType::Flash => fmc::vals::Mtyp::FLASH,
                    }); // Memory types

                    bcr.set_muxen(self.config.data_address_mux_enabled); // Data address mux

                    bcr.set_cpsize(match self.config.page_size {
                        PageSize::NoBurstSplit => fmc::vals::Cpsize::NO_BURST_SPLIT,
                        PageSize::Bytes128 => fmc::vals::Cpsize::BYTES128,
                        PageSize::Bytes256 => fmc::vals::Cpsize::BYTES256,
                        PageSize::Bytes512 => fmc::vals::Cpsize::BYTES512,
                        PageSize::Bytes1024 => fmc::vals::Cpsize::BYTES1024,
                    });

                    trace!("fmc bcr: {}", bcr);

                    Ok(())
                })?;
        }

        Ok(())
    }

    // Initializes the FMC timing register corresponding to the enabled bank.
    pub fn timing_init(&mut self) -> Result<(), TimingError> {
        if !(self.timing.address_setup_time <= 15) {
            return Err(TimingError::InvalidAddressSetupTime);
        }

        if !(self.timing.address_hold_time > 0 && self.timing.address_hold_time <= 15) {
            return Err(TimingError::InvalidAddressHoldTime);
        }

        if !(self.timing.data_setup_time > 0 && self.timing.data_setup_time <= 255) {
            return Err(TimingError::InvalidDataSetupTime);
        }

        if !(self.timing.bus_turn_around_duration <= 15) {
            return Err(TimingError::InvalidTurnaroundTime);
        }

        if !(self.timing.clock_division > 1 && self.timing.clock_division <= 16) {
            return Err(TimingError::InvalidClockDivisor);
        }

        // BTR1/2/3/4 SRAM/NOR-Flash write timing registers
        self.fmc
            .btr(match self.bank {
                FMCRegion::Bank1 | FMCRegion::Bank1Remapped => 0,
                FMCRegion::Bank2 | FMCRegion::Bank2Remapped => 1,
                FMCRegion::Bank3 | FMCRegion::Bank3Remapped => 2,
                FMCRegion::Bank4 | FMCRegion::Bank4Remapped => 3,
            })
            .modify(|btr| {
                btr.set_addset(self.timing.address_setup_time);
                btr.set_addhld(self.timing.address_hold_time);
                btr.set_datast(self.timing.data_setup_time);
                btr.set_busturn(self.timing.bus_turn_around_duration);
                btr.set_clkdiv(self.timing.clock_division);
                btr.set_datlat(self.timing.data_latency);
                btr.set_accmod(match self.timing.access_mode {
                    AccessMode::AccessModeA => Accmod::A,
                    AccessMode::AccessModeB => Accmod::B,
                    AccessMode::AccessModeC => Accmod::C,
                    AccessMode::AccessModeD => Accmod::D,
                });
                trace!("fmc btr: {}", btr);
            });

        Ok(())
    }

    pub fn disable_bank(&mut self) {
        if self.bank == FMCRegion::Bank1 || self.bank == FMCRegion::Bank1Remapped {
            // SRAM/NOR-Flash chip-select control register
            self.fmc.bcr1().modify(|bcr1| {
                bcr1.set_mbken(false);
            });
        } else {
            // SRAM/NOR-Flash chip-select control register
            self.fmc
                .bcr(match self.bank {
                    FMCRegion::Bank1 | FMCRegion::Bank1Remapped => panic!("invalid"),
                    FMCRegion::Bank2 | FMCRegion::Bank2Remapped => 0,
                    FMCRegion::Bank3 | FMCRegion::Bank3Remapped => 1,
                    FMCRegion::Bank4 | FMCRegion::Bank4Remapped => 2,
                })
                .modify(|bcr| {
                    bcr.set_mbken(false);
                });
        }
    }

    pub fn enable_bank(&mut self) {
        if self.bank == FMCRegion::Bank1 || self.bank == FMCRegion::Bank1Remapped {
            // SRAM/NOR-Flash chip-select control register
            self.fmc.bcr1().modify(|bcr| {
                bcr.set_mbken(true);
            });
        } else {
            // SRAM/NOR-Flash chip-select control register
            self.fmc
                .bcr(match self.bank {
                    FMCRegion::Bank1 | FMCRegion::Bank1Remapped => panic!("invalid"),
                    FMCRegion::Bank2 | FMCRegion::Bank2Remapped => 0,
                    FMCRegion::Bank3 | FMCRegion::Bank3Remapped => 1,
                    FMCRegion::Bank4 | FMCRegion::Bank4Remapped => 2,
                })
                .modify(|bcr| {
                    bcr.set_mbken(true);
                });
        }
    }

    /// Initializes the SRAM.
    pub fn init(&mut self) -> Result<(), InitError> {
        trace!("Ensuring memory bank is disabled...");
        self.disable_bank();

        trace!("Setting FMC control register...");
        self.config_init()?;
        trace!("Setting FMC timing register...");
        self.timing_init()?;
        // TODO: extended mode timing

        trace!("Enabling memory bank...");
        self.enable_bank();

        Ok(())
    }

    pub const fn addr(&self) -> u32 {
        // Banks start at 0x6000_0000 -> 01100000000000000000000000000000
        // Banks end at   0x6FFF_FFFF -> 01101111111111111111111111111111
        //
        // ADDR[27:26] Select bank:
        // 00 Bank 1 - NOR/PSRAM 1 0x60000000 -> 01100000000000000000000000000000
        // 01 Bank 1 - NOR/PSRAM 2 0x64000000 -> 01100100000000000000000000000000
        // 10 Bank 1 - NOR/PSRAM 3 0x68000000 -> 01101000000000000000000000000000
        // 11 Bank 1 - NOR/PSRAM 4 0x6C000000 -> 01101100000000000000000000000000
        self.bank.min_address()
    }

    pub const fn ptr(&self) -> *mut u16 {
        (self.addr()) as *mut u16
    }

    /// Returns the memory address for the FMC bank that, when written
    /// to, will send it as a "command" signal to the LCD.
    ///
    /// Because the LCD is tied to A0 as the D/C line, setting bit 0 to
    /// 1/0 toggles whether a data or command byte is being sent.
    pub const fn cmd_addr(&self) -> *mut u16 {
        self.ptr()
    }

    /// Returns the address that, when written to,
    /// will send it as a "data" signal to the LCD.
    ///
    /// Because the LCD is tied to A0 as the D/C line, setting bit 0 to
    /// 1/0 toggles whether a data or command byte is being sent.
    pub const fn data_addr(&self) -> *mut u16 {
        // Add one byte to get the data address.
        //
        // Setting A0 to 1/0 selects data/command
        //
        // NOTE: The 0xF comes from a hacky fix. Using A0 as the data/command line caused a
        // problem where DMA forcing aligned writes pushed data into the next even address.
        // Taking advanage of the full SRAM address space available and using an even address
        // with a 1 in the lower bit solves this problem.
        (self.addr() + 0xF) as *mut u16
    }

    pub fn write_command(&mut self, cmd: u8, args: &[u8]) {
        trace!("command: 0x{:x}", cmd);

        // Write to the lower byte so A0=0 and triggers command mode.
        // ptr::write(self.cmd_addr(), cmd as u16);
        self.memory[0] = cmd as u16;

        for i in 0..args.len() {
            // ptr::write(self.data_addr(), args[i] as u16);
            self.memory[1] = args[i] as u16;
        }
    }

    pub fn write_data(&mut self, data: u16) {
        // ptr::write(self.data_addr(), data);
        self.memory[1] = data;
    }

    pub fn read_data(&mut self) -> u16 {
        let result = self.memory[1]; // ptr::read(self.data_addr());

        trace!("read: 0x{:x}", result);

        result
    }
}
