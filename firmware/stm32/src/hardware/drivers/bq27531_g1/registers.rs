use proc_bitfield::bitfield;

bitfield! {
    pub struct OperationConfigurationHigh (pub u8) {
        /// No-load rate of compensation is applied to the reserve capacity calculation. True when set.
        pub rescap: bool @ 7,
        /// System down interrupt bit.
        ///
        /// The SOC_INT pulses 1 ms when the battery voltage gets below SysDown
        /// Set Volt Threshold or gets above SysDown Clear Volt Threshold.
        ///
        /// True when set.
        pub int_sysdwn: bool @ 6,
        /// Battery removal interrupt bit.
        ///
        /// The SOC_INT pulses 1 ms when the battery removal interrupt is enabled.
        ///
        /// True when set.
        pub int_brem: bool @ 5,
        /// Configure the current wake function, see Table 6-7.
        pub iwake: bool @ 2,
        /// Configure the current wake function, see Table 6-7.
        pub rsns1: bool @ 1,
        /// Configure the current wake function, see Table 6-7.
        pub rsns0: bool @ 0,
    }
}

bitfield! {
    pub struct OperationConfigurationLow (pub u8) {
        ///Indication of the measurement of the OCV during the initialization.
        ///
        /// The SOC_INT pulses during the first measurement if this bit is set.
        ///
        /// True when set.
        pub int_focv: bool @ 7,
        /// Enables cell profile selection feature.
        ///
        /// True when set.
        pub idselen: bool @ 6,
        /// The fuel gauge can enter SLEEP mode, if operating conditions allow.
        ///
        /// True when set.
        pub sleep: bool @ 5,
        /// SOC interrupt polarity is active-low.
        ///
        /// True when cleared.
        pub soci_pol: bool @ 3,
        /// OCV command interrupt enable bit.
        ///
        /// True when cleared.
        pub int_ocvcmd: bool @ 2,
        /// Charge fault interrupt enable bit.
        ///
        /// True when set.
        pub int_chgflt: bool @ 1,
        /// Selects external thermistor for Temperature() measurements.
        ///
        /// True when set.
        pub temps: bool @ 0,
    }
}

bitfield! {
    pub struct OperationConfigurationB (pub u8) {
        /// Enables the temperature write.
        ///
        /// The temperature could be written by the host.
        ///
        /// True when set.
        pub wrtemp: bool @ 7,
        /// Battery insertion detection enable.
        ///
        /// When the battery insertion detection is disabled, the gauge
        /// relies on the host command to set the Flags() [BAT_DET] bit.
        ///
        /// True when set.
        pub bie: bool @ 6,
        /// Battery low interrupt enable.
        ///
        /// True when set.
        pub bl_int: bool @ 5,
        /// The ADC ground select control.
        ///
        /// The VSS (pin D1) is selected as ground reference when the bit is clear.
        ///
        /// Pin A1 is selected when the bit is set.
        pub gndsel: bool @ 4,
        /// Fast convergence enable.
        ///
        /// Configures algorithm to use fast convergence method.
        ///
        /// Default is 1 and is the recommended setting for all applications.
        ///
        /// True when set.
        pub fce: bool @ 3,
        /// Enables interrupt for any changes in charge profile.
        ///
        /// True when set.
        pub int_chgprof: bool @ 2,
        /// Enables Ra Step up/down to Min/Max Res Factor before disabling Ra updates.
        pub rfactstep: bool @ 1,
    }
}

bitfield! {
    pub struct OperationConfigurationC (pub u8) {
        /// Allows SOC to change due to temperature change during
        /// relaxation when SOC smoothing algorithm is enabled.
        ///
        /// True when set.
        pub relax_rc_jump_ok: bool @ 5,
        /// Enables SOC smoothing algorithm.
        ///
        /// True when set.
        pub smooth_en:bool @ 4,
        /// Voltage consistency enable.
        ///
        /// Default is 1 and is the recommended setting for all applications.
        ///
        /// True when set.
        pub vconsen: bool @ 3,
        /// Enables compensation for the passed charge
        /// missed when waking from SLEEP mode.
        pub sip_wk_cgh: bool @ 2,
        /// Configures options for determination of Delta Voltage which is defined as the
        /// maximum difference in Voltage() during normal load and short load spikes.
        ///
        /// Delta Voltage is a used as a compensation factor for calculating for
        /// RemainingCapacity() under pulsed loads.
        ///
        ///  00 = Standard DeltaV. Average variance from steady state
        ///   voltage that determines end-of-discharge voltage.
        ///  01 = No Averaging. The last instantaneous change in Voltage() from
        ///   steady state determines the end-of-discharge voltage.
        ///  10 = Use the value in Min Delta Voltage.
        ///  11 = Not used.
        pub delta_v_opt1: bool @ 1,
        pub delta_v_opt0: bool @ 1,
    }
}
