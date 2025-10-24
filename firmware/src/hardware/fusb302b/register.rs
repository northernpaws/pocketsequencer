// Helper that allows us to write registers as structs and read
// them directly as their corrosponding binary representation.
use bit_struct::*;
use embassy_usb::descriptor::descriptor_type::DEVICE; 

pub const DEVICE_ID_REGISTER: u8 = 0x01;
pub const SWITCHES_0_REGISTER: u8 = 0x02;
pub const SWITCHES_1_REGISTER: u8 = 0x03;
pub const MEASURE_REGISTER: u8 = 0x04;
pub const SLICE_REGISTER: u8 = 0x05;
pub const CONTROL_0_REGISTER: u8 = 0x06;
pub const CONTROL_1_REGISTER: u8 = 0x07;
pub const CONTROL_2_REGISTER: u8 = 0x08;
pub const CONTROL_3_REGISTER: u8 = 0x09;
pub const MASK_REGISTER: u8 = 0x0A;
pub const POWER_REGISTER: u8 = 0x0B;
pub const RESET_REGISTER: u8 = 0x0C;
pub const OCP_REGISTER: u8 = 0x0D;
pub const MASK_A_REGISTER: u8 = 0x0E;
pub const MASK_B_REGISTER: u8 = 0x0F;
pub const CONTROL_4_REGISTER: u8 = 0x10;
pub const STATUS_0_A_REGISTER: u8 = 0x3C;
pub const STATUS_1_A_REGISTER: u8 = 0x3D;
pub const INTERRUPT_A_REGISTER: u8 = 0x3E;
pub const INTERRUPT_B_REGISTER: u8 = 0x3F;
pub const STATUS0_REGISTER: u8 = 0x40;
pub const STATUS1_REGISTER: u8 = 0x41;
pub const INTERRUPT_REGISTER: u8 = 0x42;
pub const FIFO_REGISTER: u8 = 0x43;

pub enum RegisterType {
    Read = 0,
    ReadWrite = 1
}

pub trait Register: BitStruct<true, Kind = u8> + BitStructExt + Into<u8>{
    const ADDRESS: u8;
    const WRITEABLE: bool;
    const RESET: u8;
}

bit_struct! {
    pub struct DeviceIDRegister(u8) {
        /// Device version ID by Trim or etc.
        /// A_[Revision ID]: 1000 (e.g. A_revA)
        /// B_[Revision ID]: 1001
        /// C_[Revision ID]: 1010 etc
        version_id: u4,
        /// “01”, “10” and “11” applies to MLP only:
        /// 00: FUSB302BMPX/FUSB302BVMPX(Default) & FUSB302BUCX
        /// 01: FUSB302B01MPX
        /// 10: FUSB302B10MPX
        /// 11: FUSB302B11MPX
        product_id: u2,
        /// Revision History of each version
        /// [Version ID]_revA: 00(e.g. revA)
        /// [Version ID]_revB: 01 (e.g. revB)
        /// [Version ID]_revC: 10 (e.g. revC)
        /// [Version ID]_revC: 11 (e.g. revD)
        revision_id: u2,
    }
}

impl Into<u8> for DeviceIDRegister {
    fn into(self) -> u8 {
        self.raw()
    }
}

impl Register for DeviceIDRegister {
    const ADDRESS: u8 = DEVICE_ID_REGISTER;
    const WRITEABLE: bool = false;
    const RESET: u8 = 0b1001_0000; // last 4 are chip dependent
}

bit_struct! {
    pub struct Switches0Register(u8) {
        /// 1: Apply host pull up current to CC2 pin
        cc2_host_pull_up: bool,
        /// 1: Apply host pull up current to CC1 pin
        cc1_host_pull_up: bool,
        /// 1: Turn on the VCONN current to CC2 pin
        vconn_cc2: bool,
        /// 1: Turn on the VCONN current to CC1 pin
        vconn_cc1: bool,
        /// 1: Use the measure block to monitor or measure the voltage on CC2
        measure_cc2: bool,
        /// 1: Use the measure block to monitor or measure the voltage on CC1
        measure_cc1: bool,
        /// 1: Device pull down on CC2. 0: no pull down
        cc2_device_pull_down: bool,
        /// 1: Device pull down on CC1. 0: no pull down
        cc1_device_pull_down: bool,
    }
}

impl Into<u8> for Switches0Register {
    fn into(self) -> u8 {
        self.raw()
    }
}

impl Register for Switches0Register {
    const ADDRESS: u8 = SWITCHES_0_REGISTER;
    const WRITEABLE: bool = true;
    const RESET: u8 = 0b0000_0011;
}

bit_struct! {
    pub struct Switches1Register(u8) {
        /// Bit used for constructing the GoodCRC acknowledge packet.
        /// 
        /// This bit corresponds to the Port Power Role bit in
        /// the message header if an SOP packet is received:
        ///  1: Source if SOP
        ///  0: Sink if SOP
        good_crc_source_is_sop: bool,
        /// Bits used for constructing the GoodCRC acknowledge packet.
        /// 
        /// These bits correspond to the Specification
        /// Revision bits in the message header:
        ///  00: Revision 1.0
        ///  01: Revision 2.0
        ///  10: Do Not Use
        ///  11: Do Not Use
        good_crc_spec_revision: u2,
        /// Bit used for constructing the GoodCRC acknowledge packet.
        /// 
        /// This bit corresponds to the Port Data Role bit in the message header.
        /// 
        /// For SOP:
        ///  1: SRC
        ///  0: SNK
        good_crc_port_data_role: bool,
        reserved: bool,
        /// 1: Starts the transmitter automatically when a message with a
        ///   good CRC is received and automatically sends a GoodCRC
        ///   acknowledge packet back to the relevant SOP*
        /// 0: Feature disabled
        auto_crc: bool,
        /// 1: Enable BMC transmit driver on CC2 pin
        bmc_transmit_cc2: bool,
        /// 1: Enable BMC transmit driver on CC1 pin
        bmc_transmit_cc1: bool,
    }
}

impl Into<u8> for Switches1Register {
    fn into(self) -> u8 {
        self.raw()
    }
}

impl Register for Switches1Register {
    const ADDRESS: u8 = SWITCHES_1_REGISTER;
    const WRITEABLE: bool = true;
    const RESET: u8 = 0b0010_0000;
}


bit_struct! {
    pub struct MeasureRegister(u8) {
        reserved: bool,
        /// 0: MDAC/comparator measurement is controlled by MEAS_CC*
        ///    bits
        /// 1: Measure VBUS with the MDAC/comparator. This requires
        ///    MEAS_CC* bits to be 0
        measure_vbus: bool,
        /// Measure Block DAC data input.
        /// 
        /// LSB is equivalent to 42 mV of voltage which is compared to the
        /// measured CC voltage. The measured CC is selected by MEAS_CC2,
        /// or MEAS_CC1 bits. MDAC[5:0] MEAS_VBUS = 0 MEAS_VBUS = 1 Unit.
        /// 
        /// 00_0000 0.042 0.420 V
        /// 00_0001 0.084 0.840 V
        /// 11_0000 2.058 20.58 V
        /// 11_0011 2.184 21.84 V
        /// 11_1110 2.646 26.46 V
        /// 11_1111 > 2.688 26.88 V
        measure_block_dac_data_input: u6,
    }
}

impl Into<u8> for MeasureRegister {
    fn into(self) -> u8 {
        self.raw()
    }
}

impl Register for MeasureRegister {
    const ADDRESS: u8 = MEASURE_REGISTER;
    const WRITEABLE: bool = true;
    const RESET: u8 = 0b0011_0001;
}



bit_struct! {
    pub struct SliceRegister(u8) {
        /// Adds hysteresis where there are now two thresholds, the lower
        /// threshold which is always the value programmed by SDAC[5:0]
        /// and the higher threshold that is:
        ///  11: 255 mV hysteresis: higher threshold = (SDAC value + 20hex)
        ///  10: 170 mV hysteresis: higher threshold = (SDAC value + Ahex)
        ///  01: 85 mV hysteresis: higher threshold = (SDAC value + 5)
        ///  00: No hysteresis: higher threshold = SDAC value
        hysteresis: u2,
        /// BMC Slicer DAC data input.
        /// 
        /// Allows for a programmable threshold so as to meet
        /// the BMC receive mask under all noise conditions.
        bmc_slicer_dac_data_input: u6,
    }
}

impl Into<u8> for SliceRegister {
    fn into(self) -> u8 {
        self.raw()
    }
}

impl Register for SliceRegister {
    const ADDRESS: u8 = SLICE_REGISTER;
    const WRITEABLE: bool = true;
    const RESET: u8 = 0b0110_0000;
}

bit_struct! {
    pub struct Control0Register(u8) {
        reserved0: bool,
        /// 1: Self clearing bit to flush the content of the transmit FIFO
        flush_transmit_fifo: bool, // W/C 
        /// 1: Mask all interrupts
        /// 0: Interrupts to host are enabled
        mask_all_interrupts: bool, // R/W 
        reserved1: bool,
        /// 1: Controls the host pull up current enabled by PU_EN[2:1]:
        ///  00: No current
        ///  01: 80 mA – Default USB power
        ///  10: 180 mA – Medium Current Mode: 1.5 A
        ///  11: 330 mA – High Current Mode: 3 A
        host_pull_up_current: u2, // R/W 
        /// 1: Starts the transmitter automatically when a message with
        ///  a good CRC is received. This allows the software to take as
        ///  much as 300 mS to respond after the I_CRC_CHK interrupt is
        ///  received. Before starting the transmitter, an internal timer
        ///  waits for approximately 170 mS before executing the transmit
        ///  start and preamble
        /// 0: Feature disabled
        automatically_start_transmitter: bool, // R/W 
        /// 1: Start transmitter using the data in the transmit FIFO. Preamble
        ///  is started first. During the preamble period the transmit data
        ///  can start to be written to the transmit FIFO. Self clearing.
        transmitter_start: bool, // W/C 
    }
}

impl Into<u8> for Control0Register {
    fn into(self) -> u8 {
        self.raw()
    }
}

impl Register for Control0Register {
    const ADDRESS: u8 = CONTROL_0_REGISTER;
    const WRITEABLE: bool = true;
    const RESET: u8 = 0b0010_0100;
}

bit_struct! {
    pub struct Control1Register(u8) {
        reserved0: bool,
        /// 1: Enable SOP”_DEBUG (SOP double prime debug) packets
        /// 0: Ignore SOP”_DEBUG (SOP double prime debug) packets
        enable_sop_double_prime_debug_packets: bool, // R/W 
        /// 1: Enable SOP‘_DEBUG (SOP prime debug) packets
        /// 0: Ignore SOP‘_DEBUG (SOP prime debug) packets
        enable_sop_prime_debug_packets: bool, // R/W 
        /// 1: Sent BIST Mode 01s pattern for testing
        bist_mode2: bool, // R/W 
        reserved1: bool,
        /// 1: Self clearing bit to flush the content of the receive FIFO
        flush_receive_fifo: bool, // W/C 
        /// 1: Enable SOP”(SOP double prime) packets
        /// 0: Ignore SOP”(SOP double prime) packets
        enable_sop_double_prime_packets: bool, // R/W 
        /// 1: Enable SOP‘(SOP prime) packets
        /// 0: Ignore SOP‘(SOP prime) packets
        enable_sop_prime_packets: bool, // R/W 
    }
}

impl Into<u8> for Control1Register {
    fn into(self) -> u8 {
        self.raw()
    }
}

impl Register for Control1Register {
    const ADDRESS: u8 = CONTROL_1_REGISTER;
    const WRITEABLE: bool = true;
    const RESET: u8 = 0b0000_0000;
}

enums! {
    pub ToggleSavePwr {
        DontDisable,
        Wait40ms,
        Wait80ms,
        Wait160ms
    }
}

enums! {
    pub Control2Mode {
        DoNotUse,
        EnableDRPPolling,
        EnableSNKPolling,
        EnableSRCPolling,
    }
}

bit_struct! {
    pub struct Control2Register(u8) {
        /// 00: Don’t go into the DISABLE state after one cycle of toggle
        /// 01: Wait between toggle cycles for tDIS time of 40 ms
        /// 10: Wait between toggle cycles for tDIS time of 80 ms
        /// 11: Wait between toggle cycles for tDIS time of 160 ms
        wait_between_toggle_cycles: ToggleSavePwr, // TOG_SAVE_PWR2:TOG_SAVE_PWR1 N/A
        /// 1: When TOGGLE=1 only Rd values will cause the TOGGLE state
        ///  machine to stop toggling and trigger the I_TOGGLE interrupt.
        /// 0: When TOGGLE=1, Rd and Ra values will cause the TOGGLE
        ///  state machine to stop toggling.
        toggle_rd_only: bool, // TOG_RD_ONLY R/W
        reserved: bool,
        /// 1: Enable Wake Detection functionality if the power state is correct
        /// 0: Disable Wake Detection functionality
        enable_wake_detection: bool, // WAKE_EN R/W
        /// 11: Enable SRC polling functionality if TOGGLE=1
        /// 10: Enable SNK polling functionality if TOGGLE=1
        /// 01: Enable DRP polling functionality if TOGGLE=1
        /// 00: Do Not Use
        mode: Control2Mode, // MODE R/W
        /// 1: Enable DRP, SNK or SRC Toggle autonomous functionality
        /// 0: Disable DRP, SNK and SRC Toggle functionality
        autonomous_drop_snk_src_toggle: bool, // TOGGLE R/W
    }
}

impl Into<u8> for Control2Register {
    fn into(self) -> u8 {
        self.raw()
    }
}

impl Register for Control2Register {
    const ADDRESS: u8 = CONTROL_2_REGISTER;
    const WRITEABLE: bool = true;
    const RESET: u8 = 0b0000_0010;
}

enums! {
    pub Retries {
        NoRetries,
        OneRetry,
        TwoRetries,
        ThreeRetries,
    }
}

bit_struct! {
    pub struct Control3Register(u8) {
        reserved: bool,
        /// 1: Send a hard reset packet (highest priority)
        /// *0: Don’t send a soft reset packet
        send_hard_reset: bool, // SEND_HARD_RESET W/C
        /// 1: BIST mode. Receive FIFO is cleared immediately after
        ///  sending GoodCRC response.
        /// 0: Normal operation, All packets are treated as usual
        bist_tmode: bool, // BIST_TMODE R/W
        /// 1: Enable automatic hard reset packet if soft reset fail
        /// *0: Disable automatic hard reset packet if soft reset fail
        auto_hardreset: bool, // AUTO_HARDRESET R/W
        /// 1: Enable automatic soft reset packet if retries fail
        /// *0: Disable automatic soft reset packet if retries fail
        auto_softreset: bool, // AUTO_SOFTRESET R/W
        /// 11: Three retries of packet (four total packets sent)
        /// 10: Two retries of packet (three total packets sent)
        /// 01: One retry of packet (two total packets sent)
        /// 00: No retries (similar to disabling auto retry)
        packet_retries: Retries, // N_RETRIES[1:0] R/W
        /// 1: Enable automatic packet retries if GoodCRC is not received
        /// 0: Disable automatic packet retries if GoodCRC not received
        auto_retry: bool, // AUTO_RETRY R/W
    }
}

impl Into<u8> for Control3Register {
    fn into(self) -> u8 {
        self.raw()
    }
}

impl Register for Control3Register {
    const ADDRESS: u8 = CONTROL_3_REGISTER;
    const WRITEABLE: bool = true;
    const RESET: u8 = 0b0000_0110;
}

bit_struct! {
    pub struct MaskRegister(u8) {
        /// 1: Mask I_VBUSOK interrupt bit
        /// *0: Do not mask
        mask_vbus_ok: bool, // M_VBUSOK
        /// 1: Mask interrupt for a transition in CC bus activity
        /// *0: Do not mask
        mask_cc_bus_activity: bool, // M_ACTIVITY 
        /// 1: Mask I_COMP_CHNG interrupt for change is the value of
        ///  COMP, the measure comparator
        /// *0: Do not mask
        mask_i_comp_chng: bool, // M_COMP_CHNG 
        /// 1: Mask interrupt from CRC_CHK bit
        /// *0: Do not mask
        mask_crc_check: bool, // M_CRC_CHK 
        /// 1: Mask the I_ALERT interrupt bit
        /// *0: Do not mask
        mask_alert: bool, // M_ALERT
        /// 1: Mask the I_WAKE interrupt bit
        /// *0: Do not mask
        mask_wake: bool, // M_WAKE  
        /// 1: Mask the I_COLLISION interrupt bit
        /// *0: Do not mask
        mask_collision: bool, // M_COLLISION
        /// 1: Mask a change in host requested current level
        /// *0: Do not mask
        mask_host_requested_current_level: bool, // M_BC_LVL 
    }
}

impl Into<u8> for MaskRegister {
    fn into(self) -> u8 {
        self.raw()
    }
}

impl Register for MaskRegister {
    const ADDRESS: u8 = MASK_REGISTER;
    const WRITEABLE: bool = true;
    const RESET: u8 = 0b0000_0000;
}


bit_struct! {
    pub struct PowerRegister(u8) {
        reserved: u4,
        /// PWR[3]: Enable internal oscillator
        enable_internal_ocillator: bool,
        /// PWR[2]: Measure block powered
        enable_measure_block: bool,
        /// PWR[1]: Receiver powered and current references for Measure block
        enable_receiver_and_current_references: bool,
        /// PWR[0]: Bandgap and wake circuit
        enable_bandgap_and_wake: bool,
    }
}

impl Into<u8> for PowerRegister {
    fn into(self) -> u8 {
        self.raw()
    }
}

impl Register for PowerRegister {
    const ADDRESS: u8 = POWER_REGISTER;
    const WRITEABLE: bool = true;
    const RESET: u8 = 0b0000_0001;
}

bit_struct! {
    pub struct ResetRegister(u8) {
        reserved: u6,
        /// 1: Reset just the PD logic for both
        ///  the PD transmitter and receiver.
        pd_reset: bool,
        /// 1: Reset the FUSB302B including the I2C
        ///  registers to their default values.
        full_reset: bool,
    }
}

impl Into<u8> for ResetRegister {
    fn into(self) -> u8 {
        self.raw()
    }
}

impl Register for ResetRegister {
    const ADDRESS: u8 = RESET_REGISTER;
    const WRITEABLE: bool = true; // W/C
    const RESET: u8 = 0b0000_0000;
}

enums! {
    pub OCPCurrent {
        /// max_range / 8
        DIV8,
        /// 2 × max_range / 8
        MUL2DIV8,
        /// 3 × max_range / 8
        MUL3DIV8,
        /// 4 × max_range / 8
        MUL4DIV8, 
        /// 5 × max_range / 8
        MUL5DIV8, 
        /// 6 × max_range / 8
        MUL6DIV8, 
        /// 7 × max_range / 8
        MUL7DIV8, 
        /// max_range (see bit definition above for OCP_RANGE)
        MaxRange,
    }
}

bit_struct! {
    pub struct OCPRegister(u8) {
        reserved: u4,
        /// 1: OCP range between 100−800 mA (max_range = 800 mA)
        /// 0: OCP range between 10−80 mA (max_range = 80 mA)
        ocp_range: bool,
        /// 111: max_range (see bit definition above for OCP_RANGE)
        /// 110: 7 × max_range / 8
        /// 101: 6 × max_range / 8
        /// 100: 5 × max_range / 8
        /// 011: 4 × max_range / 8
        /// 010: 3 × max_range / 8
        /// 001: 2 × max_range / 8
        /// 000: max_range / 8
        ocp_current: OCPCurrent,
    }
}

impl Into<u8> for OCPRegister {
    fn into(self) -> u8 {
        self.raw()
    }
}

impl Register for OCPRegister {
    const ADDRESS: u8 = OCP_REGISTER;
    const WRITEABLE: bool = true; // W/C
    const RESET: u8 = 0b0000_1111;
}


bit_struct! {
    pub struct MaskARegister(u8) {
        /// 1: Mask the I_OCP_TEMP interrupt
        mask_ocp_temp: bool, // M_OCP_TEMP
        /// 1: Mask the I_TOGDONE interrupt
        mask_togdone: bool, // M_TOGDONE
        /// 1: Mask the I_SOFTFAIL interrupt
        mask_softfail: bool, // M_SOFTFAIL
        /// 1: Mask the I_RETRYFAIL interrupt
        mask_retryfail: bool, // M_RETRYFAIL
        /// 1: Mask the I_HARDSENT interrupt
        mask_hardsent: bool, // M_HARDSENT 
        /// 1: Mask the I_TXSENT interrupt
        mask_txsent: bool, // M_TXSENT
        /// 1: Mask the I_SOFTRST interrupt
        mask_softrst: bool, // M_SOFTRST 
        /// 1: Mask the I_HARDRST interrupt
        mask_hardrst: bool, // M_HARDRST 
    }
}

impl Into<u8> for MaskARegister {
    fn into(self) -> u8 {
        self.raw()
    }
}

impl Register for MaskARegister {
    const ADDRESS: u8 = MASK_A_REGISTER;
    const WRITEABLE: bool = true;
    const RESET: u8 = 0b0000_0000;
}


bit_struct! {
    pub struct MaskBRegister(u8) {
        reserved: u7,
        /// 1: Mask the I_GCRCSENT interrupt
        mask_gcrc_sent: bool, // M_GCRCSENT 
    }
}

impl Into<u8> for MaskBRegister {
    fn into(self) -> u8 {
        self.raw()
    }
}

impl Register for MaskBRegister {
    const ADDRESS: u8 = MASK_B_REGISTER;
    const WRITEABLE: bool = true; 
    const RESET: u8 = 0b0000_0000;
}

bit_struct! {
    pub struct Control4Register(u8) {
        reserved: u7,
        /// 1: In auto Rd only Toggle mode, stop Toggle
        ///  at Audio accessory (Ra on both CC) ,
        taggle_exit_audio_accessory: bool,
    }
}

impl Into<u8> for Control4Register {
    fn into(self) -> u8 {
        self.raw()
    }
}

impl Register for Control4Register {
    const ADDRESS: u8 = CONTROL_4_REGISTER;
    const WRITEABLE: bool = true; 
    const RESET: u8 = 0b0000_0000;
}

bit_struct! {
    pub struct Status0ARegister(u8) {
        reserved: u2,
        /// 1: All soft reset packets with retries have failed to get
        ///  a GoodCRC acknowledge. This status is cleared when
        ///  a START_TX, TXON or SEND_HARD_RESET is executed.
        soft_fail: bool, // SOFTFAIL R
        /// 1: All packet retries have failed to get a GoodCRC acknowledge.
        ///  This status is cleared when a START_TX, TXON or
        ///  SEND_HARD_RESET is executed.
        retry_fail: bool, // RETRYFAIL R
        /// Internal power state when logic internals needs to control the
        /// power state. POWER3 corresponds to PWR3 bit and POWER2
        /// corresponds to PWR2 bit. The power state is the higher of both
        /// PWR[3:0] and {POWER3, POWER2, PWR[1:0]} so that if one is
        /// 03 and the other is F then the internal power state is F.
        power_state: u2, // POWER3:POWER2 R
        /// 1: One of the packets received was a soft reset packet
        soft_reset: bool, // SOFTRST R
        /// 1: Hard Reset PD ordered set has been received
        hard_reset: bool, // HARDRST R
    }
}

impl Into<u8> for Status0ARegister {
    fn into(self) -> u8 {
        self.raw()
    }
}

impl Register for Status0ARegister {
    const ADDRESS: u8 = STATUS_0_A_REGISTER;
    const WRITEABLE: bool = false; 
    const RESET: u8 = 0b0000_0000;
}

enums! {
    /// 000 -
    /// 001 -
    /// 010 -
    /// 011
    /// 100
    /// 101
    /// 110
    /// 111
    pub ToggleLogic {
        /// 000: Toggle logic running (processor has previously written TOGGLE=1)
        LoginRunning, // 000
        /// 001: Toggle functionality has settled to SRCon CC1 (STOP_SRC1 state)
        SRConCC1,// 001
        /// 010: Toggle functionality has settled to SRCon CC2 (STOP_SRC2 state)
        SRConCC2,// 010
        Reserved1, // 011
        Reserved2, // 100
        /// 101: Toggle functionality has settled to SNKon CC1 (STOP_SNK1 state)
        SNKonCC1, // 101
        /// 110: Toggle functionality has settled to SNKon CC2 (STOP_SNK2 state)
        SNKonCC2, // 110
        /// 111: Toggle functionality has detected AudioAccessory with vRa
        ///  on both CC1 and CC2 (settles to STOP_SRC1 state)
        AudioAccessory // 111
    }
}

bit_struct! {
    pub struct Status1ARegister(u8) {
        reserved: u2,
        /// 000: Toggle logic running (processor has previously written TOGGLE=1)
        /// 001: Toggle functionality has settled to SRCon CC1 (STOP_SRC1 state)
        /// 010: Toggle functionality has settled to SRCon CC2 (STOP_SRC2 state)
        /// 101: Toggle functionality has settled to SNKon CC1 (STOP_SNK1 state)
        /// 110: Toggle functionality has settled to SNKon CC2 (STOP_SNK2 state)
        /// 111: Toggle functionality has detected AudioAccessory with vRa
        /// on both CC1 and CC2 (settles to STOP_SRC1 state)
        /// Otherwise: Not defined (do not interpret)
        toggle_logic: ToggleLogic, // TOGSS R
        /// 1: Indicates the last packet placed in the RxFIFO is type
        ///  SOP”_DEBUG (SOP double prime debug).
        sop_double_prime_debug_packet: bool, // RXSOP2DB R
        /// 1: Indicates the last packet placed in the RxFIFO is type
        ///  SOP’_DEBUG (SOP prime debug).
        sop_prime_debug_packet: bool, // RXSOP1DB R
        /// 1: Indicates the last packet placed in the RxFIFO is type SOP.
        rx_sop: bool, // RXSOP R
    }
}

impl Into<u8> for Status1ARegister {
    fn into(self) -> u8 {
        self.raw()
    }
}

impl Register for Status1ARegister {
    const ADDRESS: u8 = STATUS_1_A_REGISTER;
    const WRITEABLE: bool = false; 
    const RESET: u8 = 0b0000_0000;
}

bit_struct! {
    pub struct InterruptARegister(u8) {
        /// 1: Interrupt from either a OCP event on one of
        ///  the VCONN switches or an over-temperature event.
        ocp_or_temp: bool, // I_OCP_TEMP R/C
        /// 1: Interrupt indicating the TOGGLE functionality
        ///  was terminated because a device was detected.
        toggle_done: bool, // I_TOGDONE R/C
        /// 1: Interrupt from automatic soft reset
        ///  packets with retries have failed.
        soft_fail: bool, // I_SOFTFAIL R/C
        /// 1: Interrupt from automatic packet retries have failed.
        retry_fail: bool, // I_RETRYFAIL  R/C
        /// 1: Interrupt from successfully sending a hard reset ordered set.
        hard_sent: bool, // I_HARDSENT R/C
        /// 1: Interrupt to alert that we sent a packet that was
        ///  acknowledged with a GoodCRC response packet.
        tx_sent: bool, // I_TXSENT R/C
        /// 1: Received a soft reset packet.
        soft_reset: bool, // I_SOFTRST R/C
        /// 1: Received a hard reset ordered set.
        hard_reset: bool, // I_HARDRST R/C
    }
}

impl Into<u8> for InterruptARegister {
    fn into(self) -> u8 {
        self.raw()
    }
}

impl Register for InterruptARegister {
    const ADDRESS: u8 = INTERRUPT_A_REGISTER;
    const WRITEABLE: bool = false; // R/C
    const RESET: u8 = 0b0000_0000;
}

bit_struct! {
    pub struct InterruptBRegister(u8) {
        reserved: u7,
        /// 1: Sent a GoodCRC acknowledge packet in response to
        ///  an incoming packet that has the correct CRC value.
        good_crc_sent: bool, // I_GCRCSENT R/C
    }
}

impl Into<u8> for InterruptBRegister {
    fn into(self) -> u8 {
        self.raw()
    }
}

impl Register for InterruptBRegister {
    const ADDRESS: u8 = INTERRUPT_B_REGISTER;
    const WRITEABLE: bool = false; // R/C
    const RESET: u8 = 0b0000_0000;
}

bit_struct! {
    pub struct Status0Register(u8) {
        /// 1: Interrupt occurs when VBUS transitions through vVBUSthr.
        ///
        /// This bit typically is used to recognize port partner during startup.
        vbus_ok: bool, // VBUSOK R
        /// 1: Transitions are detected on the active CC* line. This bit goes
        ///  high after a minimum of 3 CC transitions, and goes low with
        ///  no Transitions.
        /// *0: Inactive
        activity: bool, // ACTIVITY R
        /// 1: Measured CC* input is higher than
        ///  reference level driven from the MDAC.
        /// *0: Measured CC* input is lower than
        ///  reference level driven from the MDAC.
        comp: bool, // COMP R
        /// 1: Indicates the last received packet had the correct CRC. This
        ///  bit remains set until the SOP of the next packet.
        /// 0: Packet received for an enabled SOP* and CRC for the
        ///  enabled packet received was incorrect.
        crc_check: bool, // CRC_CHK R
        /// 1: Alert software an error condition has
        ///  occurred. An alert is caused by:
        ///   TX_FULL: the transmit FIFO is full
        ///   RX_FULL: the receive FIFO is full
        ///  See Status1 bits
        alert: bool, // ALERT R
        /// 1: Voltage on CC indicated a device attempting to attach.
        /// *0: WAKE either not enabled (WAKE_EN=0) or no device attached.
        wake: bool, // WAKE R
        /// Current voltage status of the measured CC pin
        /// interpreted as host current levels as follows:
        ///  00: < 200 mV
        ///  01: > 200 mV, < 660 mV
        ///  10: > 660 mV, < 1.23 V
        ///  11: > 1.23 V
        /// Note the software must measure these at an appropriate time,
        /// while there is no signaling activity on the selected CC line.
        /// 
        /// BC_LVL is only defined when Measure block is on which is when
        /// register bits PWR[2]=1 and either MEAS_CC1=1 or MEAS_CC2=1
        bc_lvl: bool, // BC_LVL[1:0] R
    }
}

impl Into<u8> for Status0Register {
    fn into(self) -> u8 {
        self.raw()
    }
}

impl Register for Status0Register {
    const ADDRESS: u8 = STATUS0_REGISTER;
    const WRITEABLE: bool = false; // R
    const RESET: u8 = 0b0000_0000;
}


bit_struct! {
    pub struct Status1Register(u8) {
        /// 1: Indicates the last packet placed in the
        ///  RxFIFO is type SOP” (SOP double prime).
        rx_sop_double_prime: bool, // RXSOP2 R
        /// 1: Indicates the last packet placed in the
        ///  RxFIFO is type SOP’ (SOP prime).
        rx_sop_prime: bool, // RXSOP1 R
        /// 1: The receive FIFO is empty.
        rx_empty: bool, // RX_EMPTY R
        /// 1: The receive FIFO is full.
        rx_full: bool, // RX_FULL R
        /// 1: The transmit FIFO is empty.
        tx_empty: bool, // TX_EMPTY R
        /// 1: The transmit FIFO is full.
        tx_full: bool, // TX_FULL R
        /// 1: Temperature of the device is too high.
        overtemperature: bool, // OVRTEMP R
        /// 1: Indicates an over-current or short condition
        ///  has occurred on the VCONN switch.
        ocp: bool, // OCP R
    }
}

impl Into<u8> for Status1Register {
    fn into(self) -> u8 {
        self.raw()
    }
}

impl Register for Status1Register {
    const ADDRESS: u8 = STATUS1_REGISTER;
    const WRITEABLE: bool = false; // R
    const RESET: u8 = 0b0010_1000;
}


bit_struct! {
    pub struct InterruptRegister(u8) {
        /// 1: Interrupt occurs when VBUS transitions through 4.5 V. This bit
        ///  typically is used to recognize port partner during startup.
        vbus_ok: bool, // I_VBUSOK R/C
        /// 1: A change in the value of ACTIVITY
        ///  of the CC bus has occurred.
        activity: bool, // I_ACTIVITY R/C
        /// 1: A change in the value of COMP has occurred.
        ///  Indicates se-lected CC line has tripped a
        ///  threshold programmed into the MDAC.
        comp_change: bool, // I_COMP_CHNG R/C
        /// 1: The value of CRC_CHK newly valid.
        ///  I.e. The validity of the incoming packet has been checked.
        crc_check: bool, // I_CRC_CHK R/C
        /// 1: Alert software an error condition has occurred.
        ///  An alert is caused by:
        ///   TX_FULL: the transmit FIFO is full
        ///   RX_FULL: the receive FIFO is full
        ///  See Status1 bits.
        alert: bool, // I_ALERT R/C
        /// 1: Voltage on CC indicated a device attempting to attach.
        /// Software must then power up the clock and receiver blocks.
        wake: bool, // I_WAKE R/C
        /// 1: When a transmit was attempted, activity was detected on the
        ///  active CC line. Transmit is not done. The packet is received
        ///  normally.
        collision: bool, // I_COLLISION R/C
        /// 1: A change in host requested current level has occurred.
        bc_level: bool, // I_BC_LVL R/C
    }
}

impl Into<u8> for InterruptRegister {
    fn into(self) -> u8 {
        self.raw()
    }
}

impl Register for InterruptRegister {
    const ADDRESS: u8 = INTERRUPT_REGISTER;
    const WRITEABLE: bool = false; // R/C
    const RESET: u8 = 0b0000_0000;
}


bit_struct! {
    pub struct FIFOsRegister(u8) {
        /// Writing to this register writes a byte into the transmit FIFO.
        /// Reading from this register reads from the receive FIFO.
        /// Each byte is a coded token. Or a token followed by a fixed number
        /// of packed data byte (see token coding in Table 41).
        token: u8,
    }
}

impl Into<u8> for FIFOsRegister {
    fn into(self) -> u8 {
        self.raw()
    }
}

impl Register for FIFOsRegister {
    const ADDRESS: u8 = FIFO_REGISTER;
    const WRITEABLE: bool = true;
    const RESET: u8 = 0b0000_0000;
}

// TODO: FIFO tokens (p.g. 29)
