// Helper that allows us to write registers as structs and read
// them directly as their corrosponding binary representation.
use bit_struct::*;

// https://www.ti.com/lit/ug/sluua96a/sluua96a.pdf?ts=1761223531200

/// The command codes in the documentation are fairly
/// confusing, showing 2 codes for most commands.
/// 
/// However, upon inspecting some drivers it appears that
/// for the vast magority of the codes only the first command
/// byte is actually used and the second code byte is irrelevant. 
pub type CommandCode = u8;

pub const CONTROL_COMMAND_CODE: CommandCode = 0x00;
pub const AT_RATE_COMMAND_CODE: CommandCode = 0x02;
pub const AT_RATE_TIME_TO_EMPTY_COMMAND_CODE: CommandCode = 0x04;
pub const TEMPERATURE_COMMAND_CODE: CommandCode = 0x06;
pub const VOLTAGE_COMMAND_CODE: CommandCode = 0x08;
pub const FLAGS_COMMAND_CODE: CommandCode = 0x0A;
pub const NOMINAL_AVAILABLE_CAPACITY_COMMAND_CODE: CommandCode = 0x0C;
pub const FULL_AVAILABLE_CAPACITY_COMMAND_CODE: CommandCode = 0x0E;
pub const REMAINING_CAPACIY_COMMAND_CODE: CommandCode = 0x10;
pub const FULL_CHARGE_CAPACITY_COMMAND_CODE: CommandCode = 0x12;
pub const AVERAGE_CURRENT_COMMAND_CODE: CommandCode = 0x14;
pub const TIME_TO_EMPTY_CODE: CommandCode = 0x16;
pub const REMAINING_CAPACITY_UNFILTERED_COMMAND_CODE: CommandCode = 0x18;
pub const STANDBY_CURRENT_COMMAND_CODE: CommandCode = 0x1A;
pub const REMAINING_CAPACITY_FILTERED_COMMAND_CODE: CommandCode = 0x1C;
pub const PROG_CHARGING_CURRENT_COMMAND_CODE: CommandCode = 0x1E;
pub const PROG_CHARGING_VOLTAGE_COMMAND_CODE: CommandCode = 0x20;
pub const FULL_CHARGE_CAPACITY_UNFILTERED_COMMAND_CODE: CommandCode = 0x22;
pub const AVERAGE_POWER_COMMAND_CODE: CommandCode = 0x24;
pub const FULL_CHARGE_CAPACITY_FILTERED_COMMAND_CODE: CommandCode = 0x26;
pub const STATE_OF_HEALTH_COMMAND_CODE_PERCENTAGE: CommandCode = 0x28;
pub const STATE_OF_HEALTH_COMMAND_CODE_STATUS: CommandCode = 0x29;
pub const CYCLE_COUNT_COMMAND_CODE: CommandCode = 0x2A;
pub const STATE_OF_CHARGE_COMMAND_CODE: CommandCode = 0x2C;
pub const TRUE_SOC_COMMAND_CODE: CommandCode = 0x2E;
pub const INSTANTANEOUS_CURRENT_READING_COMMAND_CODE: CommandCode = 0x30;
pub const INTERNAL_TEMPERATURE_COMMAND_CODE: CommandCode = 0x32;
pub const CHARGING_LEVEL_COMMAND_CODE: CommandCode = 0x34;
pub const LEVEL_TAPER_CURRENT_COMMAND_CODE: CommandCode = 0x6E;
pub const CALC_CHARGING_CURRENT_COMMAND_CODE: CommandCode = 0x70;
pub const CALC_CHARGING_VOLTAGE_COMMAND_CODE: CommandCode = 0x72;

pub const CONTROL_STATUS_SUBCOMMAND: u16 = 0x0000;
pub const CONTROL_DEVICE_TYPE_SUBCOMMAND: u16 = 0x0001;
pub const CONTROL_FW_VERSION_SUBCOMMAND: u16 = 0x0002;
pub const CONTROL_HW_VERSION_SUBCOMMAND: u16 = 0x0003;
pub const CONTROL_SPREV_MACWRITE_SUBCOMMAND: u16 = 0x0007;
pub const CONTROL_CHEM_ID_SUBCOMMAND: u16 = 0x0008;
pub const CONTROL_BOARD_OFFSET_SUBCOMMAND: u16 = 0x0009;
pub const CONTROL_CC_OFFSET_SUBCOMMAND: u16 = 0x000A;
pub const CONTROL_CC_OFFSET_SAVE_SUBCOMMAND: u16 = 0x000B;
pub const CONTROL_OCV_CMD_SUBCOMMAND: u16 = 0x000C;
pub const CONTROL_BAT_INSERT_SUBCOMMAND: u16 = 0x000D;
pub const CONTROL_BAT_REMOVE_SUBCOMMAND: u16 = 0x000E;
pub const CONTROL_SET_HIBERNATE_SUBCOMMAND: u16 = 0x0011;
pub const CONTROL_CLEAR_HIBERNATE_SUBCOMMAND: u16 = 0x0012;
pub const CONTROL_SET_SLEEP_SUBCOMMAND: u16 = 0x0013;
pub const CONTROL_CLEAR_SLEEP_SUBCOMMAND: u16 = 0x0014;
pub const CONTROL_OTG_ENABLE_SUBCOMMAND: u16 = 0x0015;
pub const CONTROL_OTG_DISABLE_SUBCOMMAND: u16 = 0x0016;
pub const CONTROL_DIV_CUR_ENABLE_SUBCOMMAND: u16 = 0x0017;
pub const CONTROL_CHOOSE_CHEM_ID_A_SUBCOMMAND: u16 = 0x0018;
pub const CONTROL_CHOOSE_CHEM_ID_B_SUBCOMMAND: u16 = 0x0018;
pub const CONTROL_CHG_ENABLE_SUBCOMMAND: u16 = 0x001A;
pub const CONTROL_CHG_DISABLE_SUBCOMMAND: u16 = 0x001B;
pub const CONTROL_GG_CHGRCTL_ENABLE_SUBCOMMAND: u16 = 0x001C;
pub const CONTROL_GG_CHGRCTL_DISABLE_SUBCOMMAND: u16 = 0x001D;
pub const CONTROL_DIV_CUR_DISABLE_SUBCOMMAND: u16 = 0x001E;
pub const CONTROL_DF_VERSION_SUBCOMMAND: u16 = 0x001F;
pub const CONTROL_SEALED_SUBCOMMAND: u16 = 0x0020;
pub const CONTROL_IT_ENABLE_SUBCOMMAND: u16 = 0x0021;
pub const CONTROL_RESET_SUBCOMMAND: u16 = 0x0041;
pub const CONTROL_SHIPMODE_ENABLE_SUBCOMMAND: u16 = 0x0050;
pub const CONTROL_SHIPMODE_DISABLE_SUBCOMMAND: u16 = 0x0051;

// Response to the CONTROL_DEVICE_TYPE_SUBCOMMAND for the bq27531-G1 IC.
pub const BQ27531_G1_DEVICE_TYPE: u16 = 0x0531;

// trait Command: BitStruct<true, Kind = u8> + BitStructExt {
//     const CODE: CommandCode;
// }

bit_struct! {
    // CONTROL_STATUS: 0x0000
    pub struct ControlStatusHigh(u8) {
        reserved: bool,
        /// Status bit indicating the fuel gauge is in FULL ACCESS SEALED state. Active when set.
        is_full_access_sealed: bool, // FAS
        /// Status bit indicating the fuel gauge is in SEALED state. Active when set.
        is_sealed: bool, // SS
        /// Status bit indicating a valid data flash checksum has been generated. Active when set.
        data_flash_checksum_generated: bool, // CSV
        /// Status bit indicating the Coulomb Counter Calibration routine is active.
        /// The CCA routine takes place approximately 1 minute after the initialization. Active when set.
        coulomb_counter_calibation_active: bool, // CCA
        /// Status bit indicating the board calibration routine is active. Active when set.
        board_calibration_active: bool, // BCA
        /// Status bit indicating the fuel gauge has executed the OCV command.
        /// 
        /// This bit can only be set with the presence of the battery. True when set.
        executed_ocv_command: bool, // OCVCMDCOMP
        /// Status bit indicating the OCV reading is failed due to the current.
        /// 
        /// This bit can only be set with the presence of the battery. True when set.
        ocv_failed: bool, // OCVFAIL
    }
}

bit_struct! {
    // CONTROL_STATUS: 0x0000
    pub struct ControlStatusLow(u8) {
        /// Initialization completion bit indicating the initialization completed.
        /// 
        /// This bit can only be set with the presence of the battery. True when set.
        initialization_complete: bool, // INITCOMP
        /// Status bit indicating a request for entry into HIBERNATE mode from SLEEP mode.
        /// 
        /// True when set. Default is 0.
        hibernate: bool, // HIBERNATE
        /// Status bit indicating the SLEEP+ mode is enabled. True when set.
        snooze: bool, // SNOOZE
        /// Status bit indicating the fuel gauge is in SLEEP mode. True when set.
        sleep: bool, // SLEEP
        /// Status bit indicating the Impedance Track algorithm is using constant-power model.
        /// 
        /// True when set. Default is 0 (constant-current model).
        impedance_track_algorithm_is_using_constant_power_model: bool, // LDMD
        /// Status bit indicating the Ra table updates are disabled.
        /// 
        /// Updates are disabled when set.
        ra_table_updates_disabled: bool, // RUP_DIS
        /// Status bit indicating the voltages are valid for Qmax. True when set.
        qmax_voltages_valid: bool, // VOK
        /// Status bit indicating the Qmax updates enabled. True when set.
        qmax_updates_enabled: bool, // QEN
    }
}
