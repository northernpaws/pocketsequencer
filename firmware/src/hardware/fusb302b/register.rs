use defmt::Format;

// Helper that allows us to write registers as structs and read
// them directly as their corrosponding binary representation.
use bit_struct::*;

// Use of proc_bitfield adapted from:
// https://github.com/fmckeogh/usb-pd-rs/blob/main/fusb302b/src/registers.rs
use proc_bitfield::{
    bitfield,
    Bitfield
};
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

pub trait Register: Bitfield<Storage = u8> {
    const ADDRESS: u8;
    const WRITEABLE: bool;
    const RESET: u8;
}

bitfield! {
    #[derive(Format)]
    pub struct DeviceID(pub u8): FromStorage, IntoStorage {
        /// Device version ID by Trim or etc.
        /// A_[Revision ID]: 1000 (e.g. A_revA)
        /// B_[Revision ID]: 1001
        /// C_[Revision ID]: 1010 etc
        pub version_id: u8 [read_only] @ 4..=7,
        /// “01”, “10” and “11” applies to MLP only:
        /// 00: FUSB302BMPX/FUSB302BVMPX(Default) & FUSB302BUCX
        /// 01: FUSB302B01MPX
        /// 10: FUSB302B10MPX
        /// 11: FUSB302B11MPX
        pub product_id: u8 [read_only] @ 2..=3,
        /// Revision History of each version
        /// [Version ID]_revA: 00(e.g. revA)
        /// [Version ID]_revB: 01 (e.g. revB)
        /// [Version ID]_revC: 10 (e.g. revC)
        /// [Version ID]_revC: 11 (e.g. revD)
        pub revison_id: u8 [read_only] @ 0..=1,
    }
}

impl Default for DeviceID {
    fn default() -> Self {
        Self(Self::RESET)
    }
}

impl Register for DeviceID {
    const ADDRESS: u8 = DEVICE_ID_REGISTER;
    const WRITEABLE: bool = false;
    const RESET: u8 = 0b1001_0000; // last 4 are chip dependent
}

bitfield! {
    #[derive(Format)]
    pub struct Switches0(pub u8): FromStorage, IntoStorage {
        /// 1: Apply host pull up current to CC2 pin
        pub pu_en2: bool @ 7,
        /// 1: Apply host pull up current to CC1 pin
        pub pu_en1: bool @ 6,
        /// 1: Turn on the VCONN current to CC2 pin
        pub vconn_cc2: bool @ 5,
        /// 1: Turn on the VCONN current to CC1 pin
        pub vconn_cc1: bool @ 4,
        /// 1: Use the measure block to monitor or measure the voltage on CC2
        pub meas_cc2: bool @ 3,
        /// 1: Use the measure block to monitor or measure the voltage on CC1
        pub meas_cc1: bool @ 2,
        /// 1: Device pull down on CC2. 0: no pull down
        pub pdwn2: bool @ 1,
        /// 1: Device pull down on CC1. 0: no pull down
        pub pdwn1: bool @ 0,
    }
}

impl Default for Switches0 {
    fn default() -> Self {
        Self(Self::RESET)
    }
}

impl Register for Switches0 {
    const ADDRESS: u8 = SWITCHES_0_REGISTER;
    const WRITEABLE: bool = true;
    const RESET: u8 = 0b0000_0011;
}

#[derive(Clone, Copy, Debug, Format)]
pub enum PowerRole {
    Source,
    Sink,
}

impl From<bool> for PowerRole {
    fn from(value: bool) -> Self {
        match value {
            false => Self::Sink,
            true => Self::Source,
        }
    }
}

impl From<PowerRole> for bool {
    fn from(role: PowerRole) -> bool {
        match role {
            PowerRole::Sink => false,
            PowerRole::Source => true,
        }
    }
}

#[derive(Clone, Copy, Debug, Format)]
pub enum DataRole {
    Ufp,
    Dfp,
}

impl From<bool> for DataRole {
    fn from(value: bool) -> Self {
        match value {
            false => Self::Ufp,
            true => Self::Dfp,
        }
    }
}

impl From<DataRole> for bool {
    fn from(role: DataRole) -> bool {
        match role {
            DataRole::Ufp => false,
            DataRole::Dfp => true,
        }
    }
}

/// Bit used for constructing the GoodCRC acknowledge packet
/// 
/// Adapted from: https://github.com/fmckeogh/usb-pd-rs/blob/main/fusb302b/src/registers.rs
pub enum Revision {
    R1_0,
    R2_0,
}

impl From<bool> for Revision {
    fn from(value: bool) -> Self {
        match value {
            false => Self::R1_0,
            true => Self::R2_0,
        }
    }
}

impl From<Revision> for bool {
    fn from(revision: Revision) -> bool {
        match revision {
            Revision::R1_0 => false,
            Revision::R2_0 => true,
        }
    }
}

bitfield! {
    #[derive(Format)]
    pub struct Switches1(pub u8): FromStorage, IntoStorage {
        /// Bit used for constructing the GoodCRC acknowledge packet.
        /// 
        /// This bit corresponds to the Port Power Role bit in
        /// the message header if an SOP packet is received:
        ///  1: Source if SOP
        ///  0: Sink if SOP
        pub powerrole: bool [set PowerRole, get PowerRole] @ 7,
        /// Bits used for constructing the GoodCRC acknowledge packet.
        /// 
        /// These bits correspond to the Specification
        /// Revision bits in the message header:
        ///  00: Revision 1.0
        ///  01: Revision 2.0
        ///  10: Do Not Use
        ///  11: Do Not Use
        pub specrev: bool [set Revision, get Revision] @ 5,
        /// Bit used for constructing the GoodCRC acknowledge packet.
        /// 
        /// This bit corresponds to the Port Data Role bit in the message header.
        /// 
        /// For SOP:
        ///  1: SRC
        ///  0: SNK
        pub datarole: bool [set DataRole, get DataRole] @ 4,
        /// 1: Starts the transmitter automatically when a message with a
        ///   good CRC is received and automatically sends a GoodCRC
        ///   acknowledge packet back to the relevant SOP*
        /// 0: Feature disabled
        pub auto_src: bool @ 2,
        /// 1: Enable BMC transmit driver on CC2 pin
        pub txcc2: bool @ 1,
        /// 1: Enable BMC transmit driver on CC1 pin
        pub txcc1: bool @ 0,
    }
}

impl Default for Switches1 {
    fn default() -> Self {
        Self(Self::RESET)
    }
}

impl Register for Switches1 {
    const ADDRESS: u8 = SWITCHES_1_REGISTER;
    const WRITEABLE: bool = true;
    const RESET: u8 = 0b0010_0000;
}


bitfield! {
    #[derive(Format)]
    pub struct Measure(pub u8): FromStorage, IntoStorage {
        /// 0: MDAC/comparator measurement is controlled by MEAS_CC*
        ///    bits
        /// 1: Measure VBUS with the MDAC/comparator. This requires
        ///    MEAS_CC* bits to be 0
        pub meas_vbus: bool @ 6,
        /// Measure Block DAC data input.
        /// 
        /// LSB is equivalent to 42 mV of voltage which is compared to the
        /// measured CC voltage. The measured CC is selected by MEAS_CC2,
        /// or MEAS_CC1 bits. MDAC[5:0] MEAS_VBUS = 0 MEAS_VBUS = 1 Unit.
        /// 
        /// | `MDAC[5:0]` | `MEAS_VBUS = 0` | `MEAS_VBUS = 1` | Unit |
        /// |-------------|-----------------|-----------------|------|
        /// | `00_0000`   | 0.042           | 0.420           | V    |
        /// | `00_0001`   | 0.084           | 0.840           | V    |
        /// | `11_0000`   | 2.058           | 20.58           | V    |
        /// | `11_0011`   | 2.184           | 21.84           | V    |
        /// | `11_1110`   | 2.646           | 26.46           | V    |
        /// | `11_1111`   | >2.688          | 26.88           | V    |
        /// | `11_1111`   | >2.688          | 26.88           | V    |
        pub mdac: u8 @ 0..=5,
    }
}

impl Default for Measure {
    fn default() -> Self {
        Self(Self::RESET)
    }
}

impl Register for Measure {
    const ADDRESS: u8 = MEASURE_REGISTER;
    const WRITEABLE: bool = true;
    const RESET: u8 = 0b0011_0001;
}



bitfield! {
    #[derive(Format)]
    pub struct Slice(pub u8): FromStorage, IntoStorage {
        /// Adds hysteresis where there are now two thresholds, the lower
        /// threshold which is always the value programmed by SDAC[5:0]
        /// and the higher threshold that is:
        ///  11: 255 mV hysteresis: higher threshold = (SDAC value + 20hex)
        ///  10: 170 mV hysteresis: higher threshold = (SDAC value + Ahex)
        ///  01: 85 mV hysteresis: higher threshold = (SDAC value + 5)
        ///  00: No hysteresis: higher threshold = SDAC value
        pub sda_hys: u8 @ 6..=7,
        /// BMC Slicer DAC data input.
        /// 
        /// Allows for a programmable threshold so as to meet
        /// the BMC receive mask under all noise conditions.
        pub sdac: u8 @ 0..=5,
    }
}

impl Default for Slice {
    fn default() -> Self {
        Self(Self::RESET)
    }
}

impl Register for Slice {
    const ADDRESS: u8 = SLICE_REGISTER;
    const WRITEABLE: bool = true;
    const RESET: u8 = 0b0110_0000;
}

bitfield! {
    #[derive(Format)]
    pub struct Control0(pub u8): FromStorage, IntoStorage {
        /// 1: Self clearing bit to flush the content of the transmit FIFO
        pub tx_flush: bool [write_only] @ 6,
        /// 1: Mask all interrupts
        /// 0: Interrupts to host are enabled
        pub int_mask: bool @ 5,
        /// 1: Controls the host pull up current enabled by PU_EN[2:1]:
        ///  00: No current
        ///  01: 80 mA – Default USB power
        ///  10: 180 mA – Medium Current Mode: 1.5 A
        ///  11: 330 mA – High Current Mode: 3 A
         pub host_cur: u8 @ 2..=3,
        /// 1: Starts the transmitter automatically when a message with
        ///  a good CRC is received. This allows the software to take as
        ///  much as 300 mS to respond after the I_CRC_CHK interrupt is
        ///  received. Before starting the transmitter, an internal timer
        ///  waits for approximately 170 mS before executing the transmit
        ///  start and preamble
        /// 0: Feature disabled
        pub auto_pre: bool @ 1,
        /// 1: Start transmitter using the data in the transmit FIFO. Preamble
        ///  is started first. During the preamble period the transmit data
        ///  can start to be written to the transmit FIFO. Self clearing.
        pub tx_start: bool @ 0,
    }
}

impl Default for Control0 {
    fn default() -> Self {
        Self(Self::RESET)
    }
}

impl Register for Control0 {
    const ADDRESS: u8 = CONTROL_0_REGISTER;
    const WRITEABLE: bool = true;
    const RESET: u8 = 0b0010_0100;
}

bitfield! {
    #[derive(Format)]
    pub struct Control1(pub u8): FromStorage, IntoStorage {
        /// 1: Enable SOP”_DEBUG (SOP double prime debug) packets
        /// 0: Ignore SOP”_DEBUG (SOP double prime debug) packets
        pub ensop2db: bool @ 6, 
        /// 1: Enable SOP‘_DEBUG (SOP prime debug) packets
        /// 0: Ignore SOP‘_DEBUG (SOP prime debug) packets
        pub ensop1db: bool @ 5,
        /// 1: Sent BIST Mode 01s pattern for testing
        pub bist_mode2: bool @ 4,
        /// 1: Self clearing bit to flush the content of the receive FIFO
        pub rx_flush: bool [write_only] @ 2,
        /// 1: Enable SOP”(SOP double prime) packets
        /// 0: Ignore SOP”(SOP double prime) packets
        pub ensop2: bool @ 1,
        /// 1: Enable SOP‘(SOP prime) packets
        /// 0: Ignore SOP‘(SOP prime) packets
        pub ensop1: bool @ 1,
    }
}

impl Default for Control1 {
    fn default() -> Self {
        Self(Self::RESET)
    }
}

impl Register for Control1 {
    const ADDRESS: u8 = CONTROL_1_REGISTER;
    const WRITEABLE: bool = true;
    const RESET: u8 = 0b0000_0000;
}


bitfield! {
    #[derive(Format)]
    pub struct Control2(pub u8): FromStorage, IntoStorage {
        /// 00: Don’t go into the DISABLE state after one cycle of toggle
        /// 01: Wait between toggle cycles for tDIS time of 40 ms
        /// 10: Wait between toggle cycles for tDIS time of 80 ms
        /// 11: Wait between toggle cycles for tDIS time of 160 ms
        pub tog_save_pwr: u8 @ 6..=7,
        /// 1: When TOGGLE=1 only Rd values will cause the TOGGLE state
        ///  machine to stop toggling and trigger the I_TOGGLE interrupt.
        /// 0: When TOGGLE=1, Rd and Ra values will cause the TOGGLE
        ///  state machine to stop toggling.
        pub rog_rd_only: bool @ 5,
        /// 1: Enable Wake Detection functionality if the power state is correct
        /// 0: Disable Wake Detection functionality
        pub wake_end: bool @ 3,
        /// 11: Enable SRC polling functionality if TOGGLE=1
        /// 10: Enable SNK polling functionality if TOGGLE=1
        /// 01: Enable DRP polling functionality if TOGGLE=1
        /// 00: Do Not Use
        pub mode: u8 @ 1..=2,
        /// 1: Enable DRP, SNK or SRC Toggle autonomous functionality
        /// 0: Disable DRP, SNK and SRC Toggle functionality
        pub toggle: bool @ 0,
    }
}

impl Default for Control2 {
    fn default() -> Self {
        Self(Self::RESET)
    }
}

impl Register for Control2 {
    const ADDRESS: u8 = CONTROL_2_REGISTER;
    const WRITEABLE: bool = true;
    const RESET: u8 = 0b0000_0010;
}

bitfield! {
    #[derive(Format)]
    pub struct Control3(pub u8): FromStorage, IntoStorage {
        /// 1: Send a hard reset packet (highest priority)
        /// *0: Don’t send a soft reset packet
        pub send_hard_reset: bool [write_only] @ 6,

        /// 1: BIST mode. Receive FIFO is cleared immediately after
        ///  sending GoodCRC response.
        /// 0: Normal operation, All packets are treated as usual
        pub bist_tmode: bool @ 5,

        /// 1: Enable automatic hard reset packet if soft reset fail
        /// *0: Disable automatic hard reset packet if soft reset fail
        pub auto_hardreset: bool @ 4,
        /// 1: Enable automatic soft reset packet if retries fail
        /// *0: Disable automatic soft reset packet if retries fail
        pub auto_softreset: bool @ 3,
        /// 11: Three retries of packet (four total packets sent)
        /// 10: Two retries of packet (three total packets sent)
        /// 01: One retry of packet (two total packets sent)
        /// 00: No retries (similar to disabling auto retry)
        pub n_retries: u8 @ 1..=2,
        /// 1: Enable automatic packet retries if GoodCRC is not received
        /// 0: Disable automatic packet retries if GoodCRC not received
        pub auto_retry: bool @ 0,
    }
}

impl Default for Control3 {
    fn default() -> Self {
        Self(Self::RESET)
    }
}

impl Register for Control3 {
    const ADDRESS: u8 = CONTROL_3_REGISTER;
    const WRITEABLE: bool = true;
    const RESET: u8 = 0b0000_0110;
}

bitfield! {
    #[derive(Format)]
    pub struct Mask(pub u8): FromStorage, IntoStorage {
        /// 1: Mask I_VBUSOK interrupt bit
        /// *0: Do not mask
        pub m_vbusok: bool @ 7,
        /// 1: Mask interrupt for a transition in CC bus activity
        /// *0: Do not mask
        pub m_activity: bool @ 6, 
        /// 1: Mask I_COMP_CHNG interrupt for change is the value of
        ///  COMP, the measure comparator
        /// *0: Do not mask
        pub m_comp_chng: bool @ 5,
        /// 1: Mask interrupt from CRC_CHK bit
        /// *0: Do not mask
        pub m_crc_chk: bool @ 4,
        /// 1: Mask the I_ALERT interrupt bit
        /// *0: Do not mask
        pub m_alert: bool @ 3,
        /// 1: Mask the I_WAKE interrupt bit
        /// *0: Do not mask
        pub m_wake: bool @ 2,
        /// 1: Mask the I_COLLISION interrupt bit
        /// *0: Do not mask
        pub m_collision: bool @ 1,
        /// 1: Mask a change in host requested current level
        /// *0: Do not mask
        pub m_bc_lvl: bool @ 0,
    }
}

impl Default for Mask {
    fn default() -> Self {
        Self(Self::RESET)
    }
}

impl Register for Mask {
    const ADDRESS: u8 = MASK_REGISTER;
    const WRITEABLE: bool = true;
    const RESET: u8 = 0b0000_0000;
}


bitfield! {
    #[derive(Format)]
    pub struct Power(pub u8): FromStorage, IntoStorage {
        /// PWR[3]: Enable internal oscillator
        pub internal_oscillator: bool @ 3,
        /// PWR[2]: Measure block powered
        pub measure_block: bool @ 2,
        /// PWR[1]: Receiver powered and current references for Measure block
        pub receiver: bool @ 1,
        /// PWR[0]: Bandgap and wake circuit
        pub bandgap_wake: bool @ 0,
    }
}

impl Default for Power {
    fn default() -> Self {
        Self(Self::RESET)
    }
}

impl Register for Power {
    const ADDRESS: u8 = POWER_REGISTER;
    const WRITEABLE: bool = true;
    const RESET: u8 = 0b0000_0001;
}

bitfield! {
    #[derive(Format, Default)]
    pub struct Reset(pub u8): FromStorage, IntoStorage {
        /// 1: Reset just the PD logic for both
        ///  the PD transmitter and receiver.
        pub pd_reset: bool @ 1,

        /// 1: Reset the FUSB302B including the I2C
        ///  registers to their default values.
        pub sw_reset: bool @ 0,
    }
}

impl Register for Reset {
    const ADDRESS: u8 = RESET_REGISTER;
    const WRITEABLE: bool = true; // W/C
    const RESET: u8 = 0b0000_0000;
}

bitfield! {
    #[derive(Format)]
    pub struct OCPReg(pub u8): FromStorage, IntoStorage {
        /// 1: OCP range between 100−800 mA (max_range = 800 mA)
        /// 0: OCP range between 10−80 mA (max_range = 80 mA)
        pub ocp_range: bool @ 3,

        /// 111: max_range (see bit definition above for OCP_RANGE)
        /// 110: 7 × max_range / 8
        /// 101: 6 × max_range / 8
        /// 100: 5 × max_range / 8
        /// 011: 4 × max_range / 8
        /// 010: 3 × max_range / 8
        /// 001: 2 × max_range / 8
        /// 000: max_range / 8
        pub ocp_cur: u8 @ 0..=2,
    }
}

impl Default for OCPReg {
    fn default() -> Self {
        Self(Self::RESET)
    }
}

impl Register for OCPReg {
    const ADDRESS: u8 = OCP_REGISTER;
    const WRITEABLE: bool = true; // W/C
    const RESET: u8 = 0b0000_1111;
}


bitfield! {
    #[derive(Format, Default)]
    pub struct MaskA(pub u8): FromStorage, IntoStorage {
        /// 1: Mask the I_OCP_TEMP interrupt
        pub m_ocp_temp: bool @ 7,
        /// 1: Mask the I_TOGDONE interrupt
        pub m_togdone: bool @ 6,
        /// 1: Mask the I_SOFTFAIL interrupt
        pub m_softfail: bool @ 5,
        /// 1: Mask the I_RETRYFAIL interrupt
        pub m_retryfail: bool @ 4,
        /// 1: Mask the I_HARDSENT interrupt
        pub m_hardsent: bool @ 3,
        /// 1: Mask the I_TXSENT interrupt
        pub m_txsent: bool @ 2,
        /// 1: Mask the I_SOFTRST interrupt
        pub m_softrst: bool @ 1,
        /// 1: Mask the I_HARDRST interrupt
        pub m_hardrst: bool @ 0,
    }
}

impl Register for MaskA {
    const ADDRESS: u8 = MASK_A_REGISTER;
    const WRITEABLE: bool = true;
    const RESET: u8 = 0b0000_0000;
}


bitfield! {
    #[derive(Format, Default)]
    pub struct MaskB(u8): FromStorage, IntoStorage {
        /// 1: Mask the I_GCRCSENT interrupt
        pub m_gcrcsent: bool @ 0,
    }
}

impl Register for MaskB {
    const ADDRESS: u8 = MASK_B_REGISTER;
    const WRITEABLE: bool = true; 
    const RESET: u8 = 0b0000_0000;
}

bitfield! {
    #[derive(Format, Default)]
    pub struct Control4(pub u8): FromStorage, IntoStorage {
        /// 1: In auto Rd only Toggle mode, stop Toggle
        ///  at Audio accessory (Ra on both CC) ,
        pub tog_exit_aud: bool @ 0,
    }
}

impl Register for Control4 {
    const ADDRESS: u8 = CONTROL_4_REGISTER;
    const WRITEABLE: bool = true; 
    const RESET: u8 = 0b0000_0000;
}

bitfield! {
    #[derive(Format, Default)]
    pub struct Status0A(pub u8): FromStorage, IntoStorage {
        /// 1: All soft reset packets with retries have failed to get
        ///  a GoodCRC acknowledge. This status is cleared when
        ///  a START_TX, TXON or SEND_HARD_RESET is executed.
        pub softfail: bool [read_only] @ 5,
        /// 1: All packet retries have failed to get a GoodCRC acknowledge.
        ///  This status is cleared when a START_TX, TXON or
        ///  SEND_HARD_RESET is executed.
        pub retryfail: bool [read_only] @ 4,
        /// Internal power state when logic internals needs to control the
        /// power state. POWER3 corresponds to PWR3 bit and POWER2
        /// corresponds to PWR2 bit. The power state is the higher of both
        /// PWR[3:0] and {POWER3, POWER2, PWR[1:0]} so that if one is
        /// 03 and the other is F then the internal power state is F.
        pub power: u8 [read_only] @ 2..=3,
        /// 1: One of the packets received was a soft reset packet
        pub softrst: bool [read_only] @ 1,
        /// 1: Hard Reset PD ordered set has been received
        pub hardrst: bool [read_only] @ 0,
    }
}

impl Register for Status0A {
    const ADDRESS: u8 = STATUS_0_A_REGISTER;
    const WRITEABLE: bool = false; 
    const RESET: u8 = 0b0000_0000;
}

bitfield! {
    #[derive(Format, Default)]
    pub struct Status1A(pub u8): FromStorage, IntoStorage {
        /// 000: Toggle logic running (processor has previously written TOGGLE=1)
        /// 001: Toggle functionality has settled to SRCon CC1 (STOP_SRC1 state)
        /// 010: Toggle functionality has settled to SRCon CC2 (STOP_SRC2 state)
        /// 101: Toggle functionality has settled to SNKon CC1 (STOP_SNK1 state)
        /// 110: Toggle functionality has settled to SNKon CC2 (STOP_SNK2 state)
        /// 111: Toggle functionality has detected AudioAccessory with vRa
        /// on both CC1 and CC2 (settles to STOP_SRC1 state)
        /// Otherwise: Not defined (do not interpret)
        pub togss: u8 [read_only] @ 3..=5,
        /// 1: Indicates the last packet placed in the RxFIFO is type
        ///  SOP”_DEBUG (SOP double prime debug).
        pub rxsop2db: bool [read_only] @ 2,
        /// 1: Indicates the last packet placed in the RxFIFO is type
        ///  SOP’_DEBUG (SOP prime debug).
        pub rxsop1db: bool [read_only] @ 1,
        /// 1: Indicates the last packet placed in the RxFIFO is type SOP.
        pub rxsop: bool [read_only] @ 0,
    }
}

impl Register for Status1A {
    const ADDRESS: u8 = STATUS_1_A_REGISTER;
    const WRITEABLE: bool = false; 
    const RESET: u8 = 0b0000_0000;
}

bitfield! {
    #[derive(Format, Default)]
    pub struct InterruptA(pub u8): FromStorage, IntoStorage {
        /// 1: Interrupt from either a OCP event on one of
        ///  the VCONN switches or an over-temperature event.
        pub i_ocp_temp: bool @ 7,
        /// 1: Interrupt indicating the TOGGLE functionality
        ///  was terminated because a device was detected.
        pub i_togdone: bool @ 6,
        /// 1: Interrupt from automatic soft reset
        ///  packets with retries have failed.
        pub i_softfail: bool @ 5,
        /// 1: Interrupt from automatic packet retries have failed.
        pub i_retryfail: bool @ 4,
        /// 1: Interrupt from successfully sending a hard reset ordered set.
        pub i_hardsent: bool @ 3,
        /// 1: Interrupt to alert that we sent a packet that was
        ///  acknowledged with a GoodCRC response packet.
        pub i_txsent: bool @ 2,
        /// 1: Received a soft reset packet.
        pub i_softrst: bool @ 1,
        /// 1: Received a hard reset ordered set.
        pub i_hardrst: bool @ 0,
    }
}

impl Register for InterruptA {
    const ADDRESS: u8 = INTERRUPT_A_REGISTER;
    const WRITEABLE: bool = false; // R/C
    const RESET: u8 = 0b0000_0000;
}

bitfield! {
    #[derive(Format, Default)]
    pub struct InterruptB(pub u8): FromStorage, IntoStorage {
        /// 1: Sent a GoodCRC acknowledge packet in response to
        ///  an incoming packet that has the correct CRC value.
        pub i_gcrcsent: bool @ 0,
    }
}

impl Register for InterruptB {
    const ADDRESS: u8 = INTERRUPT_B_REGISTER;
    const WRITEABLE: bool = false; // R/C
    const RESET: u8 = 0b0000_0000;
}

bitfield! {
    #[derive(Format, Default)]
    pub struct Status0(pub u8): FromStorage, IntoStorage {
        /// 1: Interrupt occurs when VBUS transitions through vVBUSthr.
        ///
        /// This bit typically is used to recognize port partner during startup.
        pub vbusok: bool [read_only] @ 7,
        /// 1: Transitions are detected on the active CC* line. This bit goes
        ///  high after a minimum of 3 CC transitions, and goes low with
        ///  no Transitions.
        /// *0: Inactive
        pub activity: bool [read_only] @ 6,
        /// 1: Measured CC* input is higher than
        ///  reference level driven from the MDAC.
        /// *0: Measured CC* input is lower than
        ///  reference level driven from the MDAC.
        pub comp: bool [read_only] @ 5,
        /// 1: Indicates the last received packet had the correct CRC. This
        ///  bit remains set until the SOP of the next packet.
        /// 0: Packet received for an enabled SOP* and CRC for the
        ///  enabled packet received was incorrect.
        pub crc_chk: bool [read_only] @ 4,
        /// 1: Alert software an error condition has
        ///  occurred. An alert is caused by:
        ///   TX_FULL: the transmit FIFO is full
        ///   RX_FULL: the receive FIFO is full
        ///  See Status1 bits
        pub alert: bool [read_only] @ 3,
        /// 1: Voltage on CC indicated a device attempting to attach.
        /// *0: WAKE either not enabled (WAKE_EN=0) or no device attached.
        pub wake: bool [read_only] @ 2,
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
        pub bc_lvl: u8 [read_only] @ 0..=1,
    }
}

impl Register for Status0 {
    const ADDRESS: u8 = STATUS0_REGISTER;
    const WRITEABLE: bool = false; // R
    const RESET: u8 = 0b0000_0000;
}


bitfield! {
    #[derive(Format, Default)]
    pub struct Status1(pub u8): FromStorage, IntoStorage {
        /// 1: Indicates the last packet placed in the
        ///  RxFIFO is type SOP” (SOP double prime).
        pub rxsop2: bool [read_only] @ 7,
        /// 1: Indicates the last packet placed in the
        ///  RxFIFO is type SOP’ (SOP prime).
        pub rxsop1: bool [read_only] @ 6,
        /// 1: The receive FIFO is empty.
        pub rx_empty: bool [read_only] @ 5,
        /// 1: The receive FIFO is full.
        pub rx_full: bool [read_only] @ 4,
        /// 1: The transmit FIFO is empty.
        pub tx_empty: bool [read_only] @ 3,
        /// 1: The transmit FIFO is full.
        pub tx_full: bool [read_only] @ 2,
        /// 1: Temperature of the device is too high.
        pub overtemp: bool [read_only] @ 1,
        /// 1: Indicates an over-current or short condition
        ///  has occurred on the VCONN switch.
        pub ocp: bool [read_only] @ 0,
    }
}

impl Register for Status1 {
    const ADDRESS: u8 = STATUS1_REGISTER;
    const WRITEABLE: bool = false; // R
    const RESET: u8 = 0b0010_1000;
}


bitfield! {
    #[derive(Format, Default)]
    pub struct Interrupt(pub u8): FromStorage, IntoStorage {
        /// 1: Interrupt occurs when VBUS transitions through 4.5 V. This bit
        ///  typically is used to recognize port partner during startup.
        pub i_vbusok: bool @ 7,
        /// 1: A change in the value of ACTIVITY
        ///  of the CC bus has occurred.
        pub i_activity: bool @ 6,
        /// 1: A change in the value of COMP has occurred.
        ///  Indicates se-lected CC line has tripped a
        ///  threshold programmed into the MDAC.
        pub i_comp_chng: bool @ 5,
        /// 1: The value of CRC_CHK newly valid.
        ///  I.e. The validity of the incoming packet has been checked.
        pub i_crc_chk: bool @ 4,
        /// 1: Alert software an error condition has occurred.
        ///  An alert is caused by:
        ///   TX_FULL: the transmit FIFO is full
        ///   RX_FULL: the receive FIFO is full
        ///  See Status1 bits.
        pub i_alert: bool @ 3,
        /// 1: Voltage on CC indicated a device attempting to attach.
        /// Software must then power up the clock and receiver blocks.
        pub i_wake: bool @ 2,
        /// 1: When a transmit was attempted, activity was detected on the
        ///  active CC line. Transmit is not done. The packet is received
        ///  normally.
        pub i_collision: bool @ 1,
        /// 1: A change in host requested current level has occurred.
        pub i_bc_lvl: bool @ 0,
    }
}

impl Register for Interrupt {
    const ADDRESS: u8 = INTERRUPT_REGISTER;
    const WRITEABLE: bool = false; // R/C
    const RESET: u8 = 0b0000_0000;
}


bitfield! {
    #[derive(Format, Default)]
    pub struct Fifo(pub u8): FromStorage, IntoStorage {
        /// Writing to this register writes a byte into the transmit FIFO.
        /// Reading from this register reads from the receive FIFO.
        /// Each byte is a coded token. Or a token followed by a fixed number
        /// of packed data byte (see token coding in Table 41).
        pub token: u8 @ 0..7,
    }
}

impl Register for Fifo {
    const ADDRESS: u8 = FIFO_REGISTER;
    const WRITEABLE: bool = true;
    const RESET: u8 = 0b0000_0000;
}

// TODO: FIFO tokens (p.g. 29)
