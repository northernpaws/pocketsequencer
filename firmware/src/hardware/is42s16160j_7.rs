/// ISSI IS42S16160J-7 SDRAM
/// 256MBit 16Mx16 143MHz
/// see: https://www.issi.com/WW/pdf/42-45S83200J-16160J.pdf

/// Some constants are only used for specific mode register combinations.
#[allow(unused)] 

use stm32_fmc;

use stm32_fmc::{
    SdramChip,
    SdramConfiguration,
    SdramTiming
};

// https://www.issi.com/WW/pdf/42-45S83200J-16160J.pdf (p.g. 25)
const BURST_LENGTH_1: u16 = 0x0000; // 000
const BURST_LENGTH_2: u16 = 0x0001; // 001
const BURST_LENGTH_4: u16 = 0x0002; // 010
const BURST_LENGTH_8: u16 = 0x0003; // 011
const BURST_TYPE_SEQUENTIAL: u16 = 0x0000;
const BURST_TYPE_INTERLEAVED: u16 = 0x0008;
const CAS_LATENCY_2: u16 = 0x0020;
const CAS_LATENCY_3: u16 = 0x0030;
const OPERATING_MODE_STANDARD: u16 = 0x0000;
const WRITEBURST_MODE_PROGRAMMED: u16 = 0x0000;
const WRITEBURST_MODE_SINGLE: u16 = 0x0200;

/// Is42s16160j with Speed Grade 7
///
/// Configured with CAS latency 2, limited 133MHz
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Is42s16160j {}

impl SdramChip for Is42s16160j {
    /// Value of the mode register
    const MODE_REGISTER: u16 = BURST_LENGTH_1
        | BURST_TYPE_SEQUENTIAL
        | CAS_LATENCY_2
        | OPERATING_MODE_STANDARD
        | WRITEBURST_MODE_SINGLE;

    /// Timing Parameters
    const TIMING: SdramTiming = SdramTiming {
        startup_delay_ns: 100_000,    // 100 µs
        max_sd_clock_hz: 133_000_000, // 133 MHz / -7 CAS latency=2 updated
        refresh_period_ns: 15_625,    // 64ms / (8192 rows) = 78125ns // updated from 4096 to 8192
        mode_register_to_active: 2,   // tMRD = 2 cycles // updated
        exit_self_refresh: 7,         // tXSR = 70ns // updated
        active_to_precharge: 3,       // tRAS = 42ns // updated (4)2 to (3)7
        row_cycle: 7,                 // tRC = 60ns // updated 63 to 60
        row_precharge: 2,             // tRP = 15ns // updated
        row_to_column: 2,             // tRCD = 15ns // updated
    };

    /// SDRAM controller configuration
    const CONFIG: SdramConfiguration = SdramConfiguration {
        column_bits: 8, // A0-A8 // updated
        row_bits: 12, // A0-A12 // updated
        memory_data_width: 16, // 16-bit // updated
        internal_banks: 16,     // 16 internal banks // updated from 4 to 16
        cas_latency: 2,        // CAS latency = 2
        write_protection: false,
        read_burst: true,
        read_pipe_delay_cycles: 0,
    };
}

