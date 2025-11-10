use proc_bitfield::bitfield;

bitfield! {
    /// ChargerStatus(): 0x74
    ///
    /// This register provides the status of direct activity between the fuel gauge and the charger.
    pub struct ChargerStatusRegister(pub u8): Debug, FromStorage, IntoStorage {
        /// Indicates that the gauge is waiting on GG_CHGCNTRL_ENABLE subcommand
        /// so that the gauge can control the charger in host mode. True when set.
        wait_cmd: bool [read_only] @ 7,
        /// Indicates I2C communication error between gauge and charger. True when set.
        err: bool [read_only] @ 6,
        /// Indicates host access to a charger register was denied. The fuel gauge allows
        /// access to all charger registers for which this bit would never be set.
        denied: bool [read_only] @ 5,
        /// Indicates that a write intended to the charger has failed. True when set.
        wfail: bool [read_only] @ 4,
        /// Indicates that the part number read from the charger in Chrgr_Rev_RegA() does not match
        /// the expected part number as programmed in the Product Number of the gauge data flash.
        authfail: bool [read_only] @ 3,
        /// Indicates when the gauge is in the initializing process of the charge state machine.
        init: bool [read_only] @ 2,
        /// Indicates that the SHIPMODE_ENABLE subcommand is active.
        ///
        /// Chrgr_OpCtrl_Reg7() [BATFET_DIS] is set after Shipmode Delay expires.
        shipmode: bool [read_only] @ 0,
    }
}

bitfield! {
    /// Chrgr_InCtrl_Reg0(): 0x75
    ///
    /// This function returns the hex value corresponding to the Input Source Control Register of the charger.
    pub struct InputSourceControlRegister(pub u8): Debug, FromStorage, IntoStorage {
        /// Enables Hi-Z mode.
        en_hiz: bool @ 7,
        /// VIN DPM setting. Offset is 3.88 V, Range is 3.88 V to 5.08 V
        /// VINDPM[3]: 640 mV
        /// VINDPM[2]: 320 mV
        /// VINDPM[1]: 160 mV
        /// VINDPM[0]: 80 mV
        vindpm3: bool @ 6,
        vindpm2:bool @ 5,
        vindpm1:bool @ 4,
        vindpm0:bool @ 3,
        /// bq2419x input current limits.
        ///
        /// The defaults are dependent on the specific charger device and
        /// state of D+/D– (bq24190), PSEL (bq24191/192/192I/193), OTG/IUSB.
        ///
        /// 000 = 100 mA
        /// 001 = 150 mA
        /// 010 = 500 mA
        /// 011 = 900 mA
        /// 100 = 1.2 A
        /// 101 = 1.5 A
        /// 110 = 2 A
        /// 111 = 3 A
        iinlim: u8 @ 0..=2,
    }
}

bitfield! {
    /// Chrgr_POR_Config_Reg1(): 0x76
    ///
    /// This function returns the hex value corresponding to
    /// the Power-On Configuration Register of the charger.
    pub struct PowerOnConfigurationRegister(pub u8): Debug, FromStorage, IntoStorage {
        /// The write access of this bit is blocked by the fuel gauge.
        ///
        /// The firmware must use fuel gauge reset to perform this function.
        ///
        /// 0 = Keep current register setting
        /// 1 = Reset register to charger reset state
        reg_reset: bool [read_only] @ 7,
        /// bq2419x charger watchdog bit.
        ///
        /// The fuel gauge sets this bit to reset the watchdog timer every 15
        /// seconds during charging. Before the fuel gauge sets this bit, Voltage(),
        /// AverageCurrent(), and Temperature() are checked to ensure the safety of
        /// continuous charging. This bit is auto-cleared by the charger and always
        /// reads 0. Default is 0.
        wdt_reset: bool [read_only] @ 6,
        /// The write access of this bit is blocked by the fuel gauge. The firmware must have
        /// MAC commands to perform charge enable (CHG_ENABLE), charge disable (CHG_DISABLE),
        /// and OTG enable functions similar to the bq2419x control of the CE bit.
        ///
        /// 00 = Disable charge
        /// 01 = Charger battery
        /// 10 or 11 = Enable OTG
        chg_config: u8 [read_only] @ 4..=5,
        /// Offset is 3.0 V, Range is 3.0 V to 3.7 V
        ///
        /// SYS_MIN[2] = 0.4 V
        /// SYS_MIN[1] = 0.2 V
        /// SYS_MIN[0] = 0.1 V
        sys_min:u8 @ 1..=3,
        /// bq2419x USB OTG current limit
        ///
        /// 0 = 500 mA
        /// 1 = 1.3 A
        boost_lim: bool @ 0,
    }
}

bitfield! {
    /// Chrgr_Current_Reg2(): 0x77
    ///
    /// This function returns the hex value corresponding to the Current Control Register of the charger.
    pub struct CurrentControlRegister(pub u8): Debug, FromStorage, IntoStorage {
        /// ICHRG[5] = Charge current is 2048 mA.
        /// ICHRG[4] = Charge current is 1024 mA.
        /// ICHRG[3] = Charge current is 512 mA.
        /// ICHRG[2] = Charge current is 256 mA.
        /// ICHRG[1] = Charge current is 128 mA.
        /// ICHRG[0] = Charge current is 64 mA.
        icgh: u8 [read_only] @ 2..=7,
    }
}

bitfield! {
    /// Chrgr_PreTerm_Reg3(): 0x78
    ///
    /// This function returns the hex value corresponding to the
    /// Pre-Charge/Termination Current Control Register of the charger.
    ///
    /// NOTE: The fuel gauge disables the charger current termination
    /// and uses the coulomb counter for termination detection.
    /// Offset = 128 mA, Range = 128 to 2048 mA, Default = 256 mA
    pub struct PreChargeTerminationCurrentControlRegister(pub u8): Debug, FromStorage, IntoStorage {
        /// IPRECHG[3] = Precharge current is 1024 mA.
        /// IPRECHG[2] = Precharge current is 512 mA.
        /// IPRECHG[1] = Precharge current is 256 mA.
        /// IPRECHG[0] = Precharge current is 128 mA.
        iprech: u8 @ 4..=7,
        /// ITERM[3] = Termination current limit is 1024 mA.
        /// ITERM[2] = Termination current limit is 512 mA.
        /// ITERM[1] = Termination current limit is 256 mA.
        /// ITERM[0] = Termination current limit is 128 mA.
        iterm:u8 [read_only] @ 0..=3,
    }
}

bitfield! {
    /// Chrgr_Voltage_Reg4(): 0x79
    ///
    /// This function returns the hex value corresponding
    /// to the Voltage Control Register of the charger.
    pub struct VoltageControlRegister(pub u8): Debug, FromStorage, IntoStorage {
        /// VREG[5] = Charge voltage is 512 mV.
        /// VREG[4] = Charge voltage is 256 mV.
        /// VREG[3] = Charge voltage is 128 mV.
        /// VREG[2] = Charge voltage is 64 mV.
        /// VREG[1] = Charge voltage is 32 mV.
        /// VREG[0] = Charge voltage is 16 mV.
        vreg: u8 [read_only] @ 2..=7,
        /// Battery low voltage (transition from precharge to fast charge)
        /// 0 = 2.8 V
        /// 1 = 3.0 V
        batlowv: bool @ 1,
        /// Battery recharge threshold
        /// 0 = 100 mV
        /// 1 = 300 mV
        vrechg: bool [read_only] @ 0,
    }
}

bitfield! {
    /// Chrgr_TermTimer_Reg5(): 0x7A
    ///
    /// This function returns the hex value corresponding
    /// to the Voltage Control Register of the charger.
    pub struct TermTimerControlRegister(pub u8): Debug, FromStorage, IntoStorage {
        /// The fuel gauge disables the termination detection to take over
        /// this function using the coulomb counter for higher performance.
        en_term: bool [read_only] @ 7,
        /// Not applicable given that the gauge disables the charger termination function.
        term_stat: bool @ 6,
        /// The gauge refreshes the charger watchdog timeout at least every 15 seconds, even during SLEEP mode.
        ///
        /// 00 = Disable timer
        /// 01 = 40 seconds
        /// 10 = 80 seconds
        /// 11 = 160 seconds
        watch_dog: u8 [read_only] @ 4..=5,
        /// 0 = Disable timer
        /// 1 = Enable timer
        en_timer: bool @ 3,
        /// Fast charge timer (2× during VINDPM, IINDPM, Thermal Regulation)
        ///
        /// 00 = 5 hrs
        /// 01 = 8 hrs
        /// 10 = 12 hrs
        /// 11 = 20 hrs
        chg_timer: u8 @ 1..=2,
        /// This bit is a function of bq24193 and not used with bq27531-G1.
        jeita_iset: bool [read_only] @ 0,
    }
}

bitfield! {
    /// Chrgr_IRThermal_Reg6(): 0x7B
    ///
    /// This function returns the hex value corresponding to the IR
    /// Compensation/Thermal Regulation Control Register of the charger.
    pub struct IRCompensationThermalRegulationControl(pub u8): Debug, FromStorage, IntoStorage {
        /// BAT_COMP[2] = Battery IR compensation resistor setting is 40 mΩ.
        /// BAT_COMP[1] = Battery IR compensation resistor setting is 20 mΩ.
        /// BAT_COMP[0] = Battery IR compensation resistor setting is 10 mΩ.
        bat_comp: u8 @5..=7,
        /// VCLAMP[2] = Battery IR compensation voltage clamp (above regulation voltage) is 64 mV.
        /// VCLAMP[1] = Battery IR compensation voltage clamp (above regulation voltage) is 32 mV.
        /// VCLAMP[0] = Battery IR compensation voltage clamp (above regulation voltage) is 16 mV.
        vclamp:u8 @ 2..=4,
        /// TREG[1:0] = Thermal regulation threshold
        /// 00 = 60°C
        /// 01 = 80°C
        /// 10 = 100°C
        /// 11 = 120°C
        treg: u8 @ 0..=1,
    }
}

bitfield! {
    /// Chrgr_OpCtrl_Reg7(): 0x7C
    ///
    /// This function returns the hex value corresponding to
    /// the Misc Operation Control Register of the charger.
    pub struct MiscOperationControlRegister (pub u8): Debug, FromStorage, IntoStorage {
        /// D+/D– detection
        ///
        /// 0 = Not in D+/D– detection
        /// 1 = Force D+/D– detection
        dpdm_en: bool @ 7,
        ///Run safety timer in half-clock rate during DPM and thermal regulation.
        ///
        /// 0 = Disable 2× extended safety timer
        /// 1 = Enable 2× extended safety timer
        tmr2x_en: bool @ 6,
        /// BATFET (Q4) enable
        ///
        /// 0 = Enable Q4
        /// 1 = Turn off Q4
        batfet_dis: bool [read_only] @ 5,
        /// This bit is a function of bq24193 and not used with bq27531-G1.
        jeita_vset: bool [read_only] @ 4,
        /// INT_MASK[1] = Interrupt on Chrgr_Fault_Reg9() CHRG_FAULT[n] bit
        ///  0 = Disable INT
        ///  1 = INT enabled
        ///
        /// INT_MASK[0] =Interrupt on Chrgr_Fault_Reg9() BAT_FAULT bit
        ///  0 = Disable INT
        ///  1 = INT enabled
        int_mask: u8 @ 0..=1,
    }
}

bitfield! {
    ///Chrgr_Status_Reg8(): 0x7D
    ///
    /// This function returns the hex value corresponding
    /// to the System Status Register of the charger.
    pub struct SystemStatusRegister (pub u8): Debug, FromStorage, IntoStorage {
        /// VBUS status
        ///
        /// 00 = Unknown (no input or DPDM detection incomplete)
        /// 01 = USB host
        /// 10 = Adapter port (from DPDM detection or PSEL detection)
        /// 11 = OTG
        vbus_stat: u8 [read_only] @ 6..=7,
        /// Charger status
        ///
        /// 00 = Not charging
        /// 01 = Precharge
        /// 10 = Fast charging
        /// 11 = Charge done
        chrg_stat: u8 [read_only] @ 4..= 5,
        /// Dynamic power management (DPM) status
        ///
        /// 0 = Not DPM
        /// 1 = VINDPM or IINDPM
        dpm_stat: bool [read_only] @ 3,
        /// Power good status
        ///
        /// 0 = Power not good
        /// 1 = Power good
        pg_stat: bool [read_only] @ 2,
        /// Thermal status
        ///
        /// 0 = Normal
        /// 1 = TREG
        term_stat: bool [read_only] @ 1,
        /// Minimum system voltage regulation status
        ///
        /// 0 = Not in VSYSMIN regulation (BAT > VSYSMIN)
        /// 1 = In VSYSMIN regulation, battery is too low.
        vsys_stat: bool [read_only] @ 0,
    }
}

bitfield! {
    /// Chrgr_Fault_Reg9(): 0x7E
    ///
    /// This function returns the hex value corresponding
    /// to the Fault Register of the charger.
    pub struct FaultRegister (pub u8): Debug, FromStorage, IntoStorage {
        /// bq2419x watchdog fault
        ///
        /// 0 = Normal
        /// 1 = Watchdog timer expiration
        watchdog_fault: bool [read_only] @ 7,
        /// OTG mode fault
        ///
        /// 0 = Normal
        /// 1 = VBUS overloaded in OTG or VBUS OVP or battery is too low (any conditions that cannot start boost function)
        otg_fault: bool [read_only] @ 6,
        /// Charger fault
        ///
        /// 00 = Normal
        /// 01 = Input fault (OVP or bad source)
        /// 10 = Thermal shutdown
        /// 11 = Charge timer expiration
        chrg_fault:u8 [read_only] @ 4..=5,
        /// Battery fault
        ///
        /// 0 = Normal
        /// 1 = System OVP
        bat_fault: bool [read_only] @ 3,
        /// Thermistor fault detected at bq2419x charger.
        ///
        /// The thermistor is disabled at the bq2419x charger
        /// when used with the bq27531-G1 fuel gauge.
        ntc_fault:u8 [read_only] @ 0..=2,
    }
}

bitfield! {
    /// Chrgr_Rev_RegA(): 0x7F
    ///
    /// This function returns the hex value corresponding to the
    /// Vendor/Part/Revision Status Register of the charger.
    pub struct VendorPartRevisionStatusRegister (pub u8): Debug, FromStorage, IntoStorage {
        /// Part number for the bq2419x charger
        /// 000 = bq24190
        /// 001 = bq24191
        /// 010 = bq24192 (The BMU FW must target bq24192.)
        /// 011 = bq24192I
        pn: u8 [read_only] @ 3..=5,
        /// Temperature sensing profile at the bq2419x charger
        /// 0 = Cold or hot window (bq24190, bq24192, bq24192I)
        /// 1 = JEITA profile (bq24193)
        ts_profile: bool [read_only] @ 2,
        /// bq2419x device revision
        rev: u8 [read_only] @ 0..=1,
    }
}
