
use proc_bitfield::bitfield;

const POWER_MANAGEMENT_1: u8 = 0x01;
const POWER_MANAGEMENT_2: u8 = 0x02;
const POWER_MANAGEMENT_3: u8 = 0x03;

// General Audio Controls
const AUDIO_INTERFACE: u8 = 0x04;
const COMPANDING: u8 = 0x05;
const CLOCK_CONTROL_1: u8 = 0x06;
const CLOCK_CONTROL_2: u8 = 0x07;
const GPIO: u8 = 0x08;
const JACK_DETECT_1: u8 = 0x09;
const DAC_CONTROL: u8 = 0x06;
const LEFT_DAC_VOLUME: u8 = 0x0B;
const RIGHT_DAC_VOLUME: u8 = 0x0C;
const JACK_DETECT_2: u8 = 0x0D;
const ADC_CONTROL: u8 = 0x0E;
const LEFT_ADC_VOLUME: u8 = 0x0F;
const RIGHT_ADC_VOLUME: u8 = 0x10;

// Equalizer
const EQ1_HIGH_CUTOFF: u8 = 0x12;
const EQ2_PEAK_1: u8 = 0x13;
const EQ3_PEAK_2: u8 = 0x14;
const EQ4_PEAK_3: u8 = 0x15;
const EQ5_LOW_CUTOFF: u8 = 0x16;

// DAC Limiter
const DAC_LIMITER_1: u8 = 0x18;
const DAC_LIMITER_2: u8 = 0x19;

// Notch filter
const NOTCH_FILTER_1: u8 = 0x1B;
const NOTCH_FILTER_2: u8 = 0x1C;
const NOTCH_FILTER_3: u8 = 0x1D;
const NOTCH_FILTER_4: u8 = 0x1E;

// ALC and Noise Gate Controls
const ALC_CONTROL_1: u8 = 0x20;
const ALC_CONTROL_2: u8 = 0x21;
const ALC_CONTROL_3: u8 = 0x22;
const NOISE_GATE: u8 = 0x23;

// Phase Locked Loop
const PLL_N: u8 = 0x24;
const PLL_K_1: u8 = 0x25;
const PLL_K_2: u8 = 0x26;
const PLL_K_3: u8 = 0x27;

// Miscellaneous
const CONTROL_3D: u8 = 0x29;
const RIGHT_SPEAKER_SUBMIXER: u8 = 0x2B;
const INPUT_CONTROL: u8 = 0x2C;
const LEFT_INPUT_PGA_GAIN: u8 = 0x2D;
const RIGHT_INPUT_PGA_GAIN: u8 = 0x2E;
const LEFT_ADC_BOOST: u8 = 0x2F;
const RIGHT_ADC_BOOST: u8 = 0x30;
const OUTPUT_CONTROL: u8 = 0x31;
const LEFT_MIXER: u8 = 0x32;
const RIGHT_MIXER: u8 = 0x33;
const LHP_VOLUME: u8 = 0x34;
const RHP_VOLUME: u8 = 0x35;
const LSPKOUT_VOLUME: u8 = 0x36;
const RSPKOUT_VOLUME: u8 = 0x37;
const AUX2_MIXER: u8 = 0x38;
const AUX1_MIXER: u8 = 0x39;
const POWER_MANAGEMENT_4: u8 = 0x3A;

// PCM Time Slot and ADCOUT Impedance Option Control
const LEFT_TIME_SLOT: u8 = 0x3B;
const MISC: u8 = 0x3C;
const RIGHT_TIME_SLOT: u8 = 0x3D;

// Silicon Revision and Device ID
const DEVICE_REVISION_NUMBER: u8 = 0x3E;
const DEVICE_ID: u8 = 0x3F;
const DAC_DITHER: u8 = 0x41;
const ALC_ENHANCEMENT_1: u8 = 0x46;
const ALC_ENHANCEMENT_2: u8 = 0x47;
const MISC_CONTROLS: u8 = 0x49;
const INPUT_TIE_OFF_DIRECT_MANUAL_CONTROL: u8 = 0x4A;
const POWER_REDUCTION_AND_OUTPUT_TIE_OFF_DIRECT_MANUAL_CONTROL: u8 = 0x4B;
const AGC_PEAK_TO_PEAK_READOUT: u8 = 0x4C;
const AGC_PEAK_DETECTOR_READOUT: u8 = 0x4D;
const AUTOMUTE_CONTROL_AND_STATUS_READOUT: u8 = 0x4E;
const OUTPUT_TIE_OFF_DIRECT_MANUAL_CONTROLS: u8 = 0x4F;

bitfield! {
    #[derive(Default)]
    pub struct PowerManagement1(pub u16): FromStorage, IntoStorage {
        /// Power control for internal tie-off buffer used in 1.5X boost conditions
        /// 0 = internal buffer unpowered
        /// 1 = enabled
        pub dcbufen: bool @ 8,
        /// Power control for AUX1 MIXER supporting AUXOUT1 analog output
        /// 0 = unpowered
        /// 1 = enabled
        pub aux1mxen: bool @ 7,
        /// Power control for AUX2 MIXER supporting AUXOUT2 analog output
        /// 0 = unpowered
        /// 1 = enabled
        pub aux2mxen: bool @ 6,
        /// Power control for internal PLL
        /// 0 = unpowered
        /// 1 = enabled
        pub pllen: bool @ 5,
        /// Power control for microphone bias buffer amplifier (MICBIAS output, pin#32)
        /// 0 = unpowered and MICBIAS pin in high-Z condition
        /// 1 = enabled
        pub micbiasen: bool @ 4,
        /// Power control for internal analog bias buffers
        /// 0 = unpowered
        /// 1 = enabled
        pub abiasen: bool @ 3,
        /// Power control for internal tie-off buffer used in non-boost mode (-1.0x gain)
        /// conditions
        /// 0 = internal buffer unpowered
        /// 1 = enabled
        pub iobufen: bool @ 2,
        /// Select impedance of reference string used to establish VREF for internal bias buffers
        /// 00 = off (input to internal bias buffer in high-Z floating condition)
        /// 01 = 80kΩ nominal impedance at VREF pin
        /// 10 = 300kΩ nominal impedance at VREF pin
        /// 11 = 3kΩ nominal impedance at VREF pin
        pub refimp: u8 @ 0..=2

    }
}

bitfield! {
    #[derive(Default)]
    pub struct PowerManagement2(pub u16): FromStorage, IntoStorage {
        pub rhpen: bool @ 8,
        pub lphen: bool @ 7,
        pub sleep: bool @ 6,
        pub rbsten: bool @ 5,
        pub lbsten: bool @ 4,
        pub rpgaen: bool @ 3,
        pub lpgaen: bool @ 2,
        pub radcen: bool @ 1,
        pub ladcen: bool @ 0
    }
}


bitfield! {
    #[derive(Default)]
    pub struct PowerManagement3(pub u16): FromStorage, IntoStorage {
        /// AUXOUT1 analog output power control, pin#21
        /// 0 = AUXOUT1 output driver OFF
        /// 1 = enabled
        pub auxout1en: bool @ 8,
        /// AUXOUT2 analog output power control, pin#22
        //// 0 = AUXOUT2 output driver OFF
        /// 1 = enabled
        pub auxout2en: bool @ 7,
        /// LSPKOUT left speaker driver power control, pin#25
        /// 0 = LSPKOUT output driver OFF
        /// 1 = enabled
        pub lspken: bool @ 6,
        /// RSPKOUT left speaker driver power control, pin#23
        /// 0 = RSPKOUT output driver OFF
        /// 1 = enabled
        pub rspken: bool @ 5,
        /// Right main mixer power control, RMAIN MIXER internal stage
        /// 0 = RMAIN MIXER stage OFF
        /// 1 = enabled
        pub rmixen: bool @ 3,
        /// Left main mixer power control, LMAIN MIXER internal stage
        /// 0 = LMAIN MIXER stage OFF
        /// 1 = enabled
        pub lmixen: bool @ 2,
        /// Right channel digital-to-analog converter, RDAC, power control
        /// 0 = RDAC stage OFF
        /// 1 = enabled
        pub rdacen: bool @ 1,
        /// Left channel digital-to-analog converter, LDAC, power control
        /// 0 = LDAC stage OFF
        /// 1 = enabled
        pub ldacen: bool @ 0
    }
}

bitfield! {
    pub struct AudioInterface(pub u16): FromStorage, IntoStorage {
        /// Bit clock phase inversion option for BCLK, pin#8
        /// 0 = normal phase
        /// 1 = input logic sense inverted
        pub bclkp: bool @ 8,
        /// Phase control for I2S audio data bus interface
        /// 0 = normal phase operation
        /// 1 = inverted phase operation
        /// PCMA and PCMB left/right word order control
        /// 0 = MSB is valid on 2nd rising edge of BCLK after rising edge of FS
        /// 1 = MSB is valid on 1st rising edge of BCLK after rising edge of FS
        pub lrp: bool @ 7,
        /// Word length (24-bits default) of audio data stream
        /// 00 = 16-bit word length
        /// 01 = 20-bit word length
        /// 10 = 24-bit word length
        /// 11 = 32-bit word length
        pub wlen: u8 @ 5..=6,
        /// Audio interface data format (default setting is I2S)
        /// 00 = right justified
        /// 01 = left justified
        /// 10 = standard I2S format
        /// 11 = PCMA or PCMB audio data format option
        pub aifmt: u8 @ 3..=4,
        /// DAC audio data left-right ordering
        /// 0 = left DAC data in left phase of LRP
        /// 1 = left DAC data in right phase of LRP (left-right reversed)
        pub dacphs: bool @ 2,
        /// ADC audio data left-right ordering
        /// 0 = left ADC data is output in left phase of LRP
        /// 1 = left ADC data is output in right phase of LRP (left-right reversed)
        pub adcphs: bool @ 1,
        /// Mono operation enable
        /// 0 = normal stereo mode of operation
        /// 1 = mono mode with audio data in left phase of LRP
        pub mono: bool @ 0,
    }
}

impl Default for AudioInterface {
    fn default() -> Self {
        Self(0x050)
    }
}

bitfield! {
    #[derive(Default)]
    pub struct Companding(pub u16): FromStorage, IntoStorage {
        /// 8-bit word enable for companding mode of operation
        /// 0 = normal operation (no companding)
        /// 1 = 8-bit operation for companding mode
        pub cmb8: bool @ 5,
        /// DAC companding mode control
        /// 00 = off (normal linear operation)
        /// 01 = reserved
        /// 10 = u-law companding
        /// 11 = A-law companding
        pub daccm: u8 @ 3..=4,
        /// ADC companding mode control
        /// 00 = off (normal linear operation)
        /// 01 = reserved
        /// 10 = u-law companding
        /// 11 = A-law companding
        pub adccm: u8 @ 1..=2,
        /// DAC audio data input option to route directly to ADC data stream
        /// 0 = no passthrough, normal operation
        /// 1 = ADC output data stream routed to DAC input data path
        pub addap: bool @ 0,
    }
}

bitfield! {
    pub struct ClockControl1(pub u16): FromStorage, IntoStorage {
        /// master clock source selection control
        /// 0 = MCLK, pin#11 used as master clock
        /// 1 = internal PLL oscillator output used as master clock
        pub clkm: bool @ 8,
        /// Scaling of master clock source for internal 256fs rate ( divide by 2 = default)
        /// 000 = divide by 1
        /// 001 = divide by 1.5
        /// 010 = divide by 2
        /// 011 = divide by 3
        /// 100 = divide by 4
        /// 101 = divide by 6
        /// 110 = divide by 8
        /// 111 = divide by 12
        pub mclksel: bool @ 5..=7,
        /// Scaling of output frequency at BCLK pin#8 when chip is in master mode
        /// 000 = divide by 1
        /// 001 = divide by 2
        /// 010 = divide by 4
        /// 011 = divide by 8
        /// 100 = divide by 16
        /// 101 = divide by 32
        /// 110 = reserved
        /// 111 = reserved
        pub bclksel: bool @ 2..=4,
        /// Enables chip master mode to drive FS and BCLK outputs
        /// 0 = FS and BCLK are inputs
        /// 1 = FS and BCLK are driven as outputs by internally generated clocks
        pub clkioen: bool @ 1,
    }
}

impl Default for ClockControl1 {
    fn default() -> Self {
        Self(0x140)
    }
}


bitfield! {
    #[derive(Default)]
    pub struct ClockControl2(pub u16): FromStorage, IntoStorage {
        /// 4-wire control interface enable
        pub w4spien: bool @ 8,
        /// Audio data sample rate indication (48kHz default).
        /// 
        /// Sets up scaling for internal filter coefficients, but does not affect in
        /// any way the actual device sample rate. Should be set to value most closely
        /// matching the actual sample rate determined by 256fs internal node.
        /// 
        /// 000 = 48kHz
        /// 001 = 32kHz
        /// 010 = 24kHz
        /// 011 = 16kHz
        /// 100 = 12kHz
        /// 101 = 8kHz
        /// 110 = reserved
        /// 111 = reserved
        pub smplr: u8 @ 2..=4,
        /// Slow timer clock enable. Starts internal timer clock derived by dividing master clock.
        /// 0 = disabled
        /// 1 = enabled
        pub sclken: bool @ 0,
    }
}



bitfield! {
    #[derive(Default)]
    pub struct Gpio(pub u16): FromStorage, IntoStorage {
        /// Clock divisor applied to PLL clock for output from a GPIO pin
        /// 00 = divide by 1
        /// 01 = divide by 2
        /// 10 = divide by 3
        /// 11 = divide by 4
        pub gpio1pll: u8 @ 4..=5,
        /// GPIO1 polarity inversion control
        /// 0 = normal logic sense of GPIO signal
        /// 1 = inverted logic sense of GPIO signal
        pub gpio1pl: bool @ 3,
        /// CSB/GPIO1 function select (input default)
        /// 000 = use as input subject to MODE pin#18 input logic level
        /// 001 = reserved
        /// 010 = Temperature OK status output ( logic 0 = thermal shutdown)
        /// 011 = DAC automute condition (logic 1 = one or both DACs automuted)
        /// 100 = output divided PLL clock
        /// 101 = PLL locked condition (logic 1 = PLL locked)
        /// 110 = output set to logic 1 condition
        /// 111 = output set to logic 0 condition
        pub gpio1sel: u8 @ 0..=2,
    }
}


bitfield! {
    #[derive(Default)]
    pub struct JackDetect1(pub u16): FromStorage, IntoStorage {
        /// Automatically enable internal bias amplifiers on jack detection state as sensed through
        /// GPIO pin associated to jack detection function
        /// Bit 7 = logic 1: enable bias amplifiers on jack at logic 0 level
        /// Bit 8 = logic 1: enable bias amplifiers on jack at logic 1 level
        pub jckmiden: u8 @ 7..=8,
        // Jack detection feature enable
        /// 0 = disabled
        /// 1 = enable jack detection associated functionality
        pub jacden: bool @ 6,
        /// Select jack detect pin (GPIO1 default)
        /// 00 = GPIO1 is used for jack detection feature
        /// 01 = GPIO2 is used for jack detection feature
        /// 10 = GPIO3 is used for jack detection feature
        /// 11 = reserved
        pub jckdio: u8  @ 4..=5,
    }
}

bitfield! {
    #[derive(Default)]
    pub struct DACControl(pub u16): FromStorage, IntoStorage {
        /// Softmute feature control for DACs
        /// 0 = disabled
        /// 1 = enabled
        pub softmt: bool @ 6,
        /// DAC oversampling rate selection (64X default)
        /// 0 = 64x oversampling
        /// 1 = 128x oversampling
        pub dacos: bool @ 3,
        /// DAC automute function enable
        /// 0 = disabled
        /// 1 = enabled
        pub automt: bool @ 2,
        /// DAC right channel output polarity control
        /// 0 = normal polarity
        /// 1 = inverted polarity
        pub rdacpl: bool @ 1,
        /// DAC left channel output polarity control
        /// 0 = normal polarity
        /// 1 = inverted polarity
        pub ldacpl: bool @ 0,
    }
}

bitfield! {
    pub struct LeftDACVolume(pub u16): FromStorage, IntoStorage {
        /// DAC volume update bit feature. Write-only bit for synchronized L/R DAC changes
        /// If logic = 0 on R11 write, new R11 value stored in temporary register
        /// If logic = 1 on R11 write, new R11 and pending R12 values become active
        pub ldacvu: bool @ 8,
        /// DAC left digital volume control (0dB default attenuation value). Expressed as an
        /// attenuation value in 0.5dB steps as follows:
        /// 0000 0000 = digital mute condition
        /// 0000 0001 = -127.0dB (highly attenuated)
        /// 0000 0010 = -126.5dB attenuation
        /// - all intermediate 0.5 step values through maximum –
        /// 1111 1110 = -0.5dB attenuation
        /// 1111 1111 = 0.0dB attenuation (no attenuation)
        pub ldacgain: u8 @ 0..=7,
    }
}

impl Default for LeftDACVolume {
    fn default() -> Self {
        Self(0x0FF)
    }
}

bitfield! {
    pub struct RightDACVolume(pub u16): FromStorage, IntoStorage {
        /// DAC volume update bit feature. Write-only bit for synchronized L/R DAC changes
        /// If logic = 0 on R12 write, new R12 value stored in temporary register
        /// If logic = 1 on R12 write, new R12 and pending R11 values become active
        pub rdacvu: bool @ 8,
        /// DAC left digital volume control (0dB default attenuation value). Expressed as an
        /// attenuation value in 0.5dB steps as follows:
        /// 0000 0000 = digital mute condition
        /// 0000 0001 = -127.0dB (highly attenuated)
        /// 0000 0010 = -126.5dB attenuation
        /// - all intermediate 0.5 step values through maximum –
        /// 1111 1110 = -0.5dB attenuation
        /// 1111 1111 = 0.0dB attenuation (no attenuation)
        pub rdacgain: u8 @ 0..=7,
    }
}

impl Default for RightDACVolume {
    fn default() -> Self {
        Self(0x0FF)
    }
}

bitfield! {
    #[derive(Default)]
    pub struct JackDetect2(pub u16): FromStorage, IntoStorage {
        /// Outputs drivers that are automatically enabled whenever the designated jack detection
        /// input is in the logic = 1 condition, and the jack detection feature is enabled
        /// Bit 4 = 1: enable Left and Right Headphone output drivers
        /// Bit 5 = 1: enable Left and Right Speaker output drivers
        /// Bit 6 = 1: enable AUXOUT2 output driver
        /// Bit 7 = 1: enable AUXOUT1 output driver
        pub jckdoen1: u8 @ 4..=7,
        /// Outputs drivers that are automatically enabled whenever the designated jack detection
        /// input is in the logic = 0 condition, and the jack detection feature is enabled
        /// Bit 0 = 1: enable Left and Right Headphone output drivers
        /// Bit 1 = 1: enable Left and Right Speaker output drivers
        /// Bit 2 = 1: enable AUXOUT2 output driver
        /// Bit 3 = 1: enable AUXOUT1 output driver
        pub jckdoen0: u8 @ 0..=3,
    }
}

bitfield! {
    pub struct ADCControl(pub u16): FromStorage, IntoStorage {
        /// High pass filter enable control for filter of ADC output data stream
        /// 0 = high pass filter disabled
        /// 1 = high pass filter enabled
        pub hpfen: bool @ 8,
        /// High pass filter mode selection
        /// 0 = normal audio mode, 1st order 3.7Hz high pass filter for DC blocking
        /// 1 = application specific mode, variable 2nd order high pass filter
        pub hpfam: bool @ 7,
        /// Application specific mode cutoff frequency selection
        ///  <See text and table for details>
        pub hpf: u8 @ 4..=6,
        /// ADC oversampling rate selection (64X default)
        /// 0 = 64x oversampling rate for reduced power
        /// 1 = 128x oversampling for better SNR
        pub adcos: bool @ 3,
        /// ADC right channel polarity control
        /// 0 = normal polarity
        /// 1 = sign of RADC output is inverted from normal polarity
        pub radcpl: bool @ 1,
        /// ADC left channel polarity control
        /// 0 = normal polarity
        /// 1 = sign of LADC output is inverted from normal polarity
        pub ladcpl: bool @ 0,
    }
}

impl Default for ADCControl {
    fn default() -> Self {
        Self(0x100)
    }
}

bitfield! {
    pub struct LeftADCVolume(pub u16): FromStorage, IntoStorage {
        /// ADC volume update bit feature. Write-only bit for synchronized L/R ADC changes
        /// If logic = 0 on R15 write, new R15 value stored in temporary register
        /// If logic = 1 on R15 write, new R15 and pending R16 values become active
        pub ladcvu: bool @ 8,
        /// ADC right digital volume control (0dB default attenuation value). Expressed as an
        /// attenuation value in 0.5dB steps as follows:
        /// 0000 0000 = digital mute condition
        /// 0000 0001 = -127.0dB (highly attenuated)
        /// 0000 0010 = -126.5dB attenuation
        /// - all intermediate 0.5 step values through maximum volume –
        /// 1111 1110 = -0.5dB attenuation
        /// 1111 1111 = 0.0dB attenuation (no attenuation)
        pub ladcgain: u8 @ 0..=7.
    }
}

impl Default for LeftADCVolume {
    fn default() -> Self {
        Self(0x0FF)
    }
}

bitfield! {
    pub struct RightADCVolume(pub u16): FromStorage, IntoStorage {
        /// ADC volume update bit feature. Write-only bit for synchronized L/R ADC changes
        /// If logic = 0 on R16 write, new R16 value stored in temporary register
        /// If logic = 1 on R16 write, new R16 and pending R15 values become active
        pub radcvu: bool @ 8,
        /// ADC left digital volume control (0dB default attenuation value). Expressed as an
        /// attenuation value in 0.5dB steps as follows:
        /// 0000 0000 = digital mute condition
        /// 0000 0001 = -127.0dB (highly attenuated)
        /// 0000 0010 = -126.5dB attenuation
        /// - all intermediate 0.5 step values through maximum volume –
        /// 1111 1110 = -0.5dB attenuation
        /// 1111 1111 = 0.0dB attenuation (no attenuation)
        pub radcgain: u8 @ 0..=7.
    }
}

impl Default for RightADCVolume {
    fn default() -> Self {
        Self(0x0FF)
    }
}

bitfield! {
    pub struct EQ1HighCutoff(pub u16): FromStorage, IntoStorage {
        /// Equalizer and 3D audio processing block assignment.
        /// 0 = block operates on digital stream from ADC
        /// 1 = block operates on digital stream to DAC (default on reset)
        pub eqm: bool @ 8,
        /// Equalizer band 1 high pass -3dB cut-off frequency selection
        /// 00 = 80Hz
        /// 01 = 105Hz (default)
        /// 10 = 135Hz
        /// 11 = 175Hz
        pub eq1cf: u8 @ 5..=6,
        /// EQ Band 1 digital gain control. Expressed as a gain or attenuation in 1dB steps
        /// 01100 = 0.0dB default unity gain value
        /// 00000 = +12dB
        /// 00001 = +11dB
        /// - all intermediate 1.0dB step values through minimum gain –
        /// 11000 = -12dB
        /// 11001 and larger values are reserved
        pub eq1gc: u8 @ 0..=4,
    }
}

impl Default for EQ1HighCutoff {
    fn default() -> Self {
        Self(0x12C)
    }
}

bitfield! {
    pub struct EQ2Peak1(pub u16): FromStorage, IntoStorage {
        /// Equalizer Band 2 bandwidth selection
        /// 0 = narrow band characteristic (default)
        /// 1 = wide band characteristic
        pub eq2bw: bool @ 8,
        /// Equalizer Band 2 center frequency selection
        /// 00 = 230Hz
        /// 01 = 300Hz (default)
        /// 10 = 385Hz
        /// 11 = 500Hz
        pub eq2cf: u8 @ 5..=6,
        /// EQ Band 2 digital gain control. Expressed as a gain or attenuation in 1dB steps
        /// 01100 = 0.0dB default unity gain value
        /// 00000 = +12dB
        /// 00001 = +11dB
        /// - all intermediate 1.0dB step values through minimum gain –
        pub eq2gc: u8 @ 0..=4,
    }
}

impl Default for EQ2Peak1 {
    fn default() -> Self {
        Self(0x02C)
    }
}

bitfield! {
    pub struct EQ3Peak2(pub u16): FromStorage, IntoStorage {
        /// Equalizer Band 3 bandwidth selection
        /// 0 = narrow band characteristic (default)
        /// 1 = wide band characteristic
        pub eq3bw: bool @ 8,
        /// Equalizer Band 3 center frequency selection
        /// 00 = 650Hz
        /// 01 = 850Hz (default)
        /// 10 = 1.1kHz
        /// 11 = 1.4kHz
        pub eq3cf: u8 @ 5..=6,
        /// EQ Band 3 digital gain control. Expressed as a gain or attenuation in 1dB steps
        /// 01100 = 0.0dB default unity gain value
        /// 00000 = +12dB
        /// 00001 = +11dB
        /// - all intermediate 1.0dB step values through minimum gain –
        /// 11000 = -12dB
        /// 11001 and larger values are reserved
        pub eq3gc: u8 @ 0..=4,
    }
}

impl Default for EQ3Peak2 {
    fn default() -> Self {
        Self(0x02C)
    }
}

bitfield! {
    pub struct EQ4Peak3(pub u16): FromStorage, IntoStorage {
        /// Equalizer Band 4 bandwidth selection
        /// 0 = narrow band characteristic (default)
        /// 1 = wide band characteristic
        pub eq4bw: bool @ 8,
        /// Equalizer Band 4 center frequency selection
        /// 00 = 1.8kHz
        /// 01 = 2.4kHz (default)
        /// 10 = 3.2kHz
        /// 11 = 4.1kHz
        pub eq4cf: u8 @ 5..=6,
        /// EQ Band 4 digital gain control. Expressed as a gain or attenuation in 1dB steps
        /// 01100 = 0.0dB default unity gain value
        /// 00000 = +12dB
        /// 00001 = +11dB
        /// - all intermediate 1.0dB step values through minimum gain –
        /// 11000 = -12dB
        /// 11001 and larger values are reserved
        pub eq4gc: u8 @ 0..=4,
    }
}

impl Default for EQ4Peak3 {
    fn default() -> Self {
        Self(0x02C)
    }
}

bitfield! {
    pub struct EQ5LowCutoff(pub u16): FromStorage, IntoStorage {
        /// Equalizer Band 5 low pass -3dB cut-off frequency selection
        /// 00 = 5.3kHz
        /// 01 = 6.9kHz (default)
        /// 10 = 9.0kHz
        /// 11 = 11.7kHz
        pub eq5cf: u8 @ 5..=6,
        /// EQ Band 5 digital gain control. Expressed as a gain or attenuation in 1dB steps
        /// 01100 = 0.0dB default unity gain value
        /// 00000 = +12dB
        /// 00001 = +11dB
        /// - all intermediate 1.0dB step values through minimum gain –
        /// 11000 = -12dB
        /// 11001 and larger values are reserved
        pub eq5gc: u8 @ 0..=4,
    }
}

impl Default for EQ5LowCutoff {
    fn default() -> Self {
        Self(0x02C)
    }
}

bitfield! {
    pub struct DACLimiter1(pub u16): FromStorage, IntoStorage {
        /// DAC digital limiter control bit
        /// 0 = disabled
        /// 1 = enabled
        pub daclimen: bool @ 8,
        /// DAC limiter decay time. Proportional to actual DAC sample rate. Duration doubles
        /// with each binary bit value. Values given here are for 44.1kHz sample rate
        /// 0000 = 0.544ms
        /// 0001 = 1.09ms
        /// 0010 = 2.18ms
        /// 0011 = 4.36ms (default)
        /// 0100 = 8.72ms
        /// 0101 = 17.4ms
        /// 0110 = 34.8ms
        /// 0111 = 69.6ms
        /// 1000 = 139ms
        /// 1001 = 278ms
        /// 1010 = 566ms
        /// 1011 through 1111 = 1130ms
        pub daclimdcy: u8 @ 4..=7,
        /// DAC limiter attack time. Proportional to actual DAC sample rate. Duration doubles
        /// with each binary bit value. Values given here are for 44.1kHz sample rate
        /// 0000 = 68.0us (microseconds)
        /// 0001 = 136us
        /// 0010 = 272us (default)
        /// 0011 = 544us
        /// 0100 = 1.09ms (milliseconds)
        /// 0101 = 2.18ms
        /// 0110 = 4.36ms
        /// 0111 = 8.72ms
        /// 1000 = 17.4ms
        /// 1001 = 34.8ms
        /// 1010 = 69.6ms
        /// 1011 through 1111 = 139ms
        pub daclimatk: u8 @ 0..=3,
    }
}

impl Default for DACLimiter1 {
    fn default() -> Self {
        Self(0x032)
    }
}

bitfield! {
    #[derive(Default)]
    pub struct DACLimiter2(pub u16): FromStorage, IntoStorage {
        /// DAC limiter threshold in relation to full scale output level (0.0dB = full scale)
        /// 000 = -1.0dB
        /// 001 = -2.0dB
        /// 010 = -3.0dB
        /// 011 = -4.0dB
        /// 100 = -5.0dB
        /// 101 through 111 = -6.0dB
        pub daclimthl: u8 @ 4..6,
        /// DAC limiter maximum automatic gain boost in limiter mode. If R24 limiter mode is
        /// disabled, specified gain value will be applied in addition to other gain values in the
        /// signal path.
        /// 0000 = 0.0dB (default)
        /// 0001 = +1.0dB
        /// - Gain value increases in 1.0dB steps for each binary value –
        /// 1100 = +12dB (maximum allowed boost value)
        /// 1101 through 1111 = reserved
        pub daclimbst: u8 @ 0..=3,
    }
}

bitfield! {
    #[derive(Default)]
    pub struct NotchFilter1(pub u16): FromStorage, IntoStorage {
        /// Update bit feature for simultaneous change of all notch filter parameters. Write-only
        /// bit. Logic 1 on R27 register write operation causes new R27 value and any pending
        /// value in R28, R29, or R30 to go into effect. Logic 0 on R27 register write causes new
        /// value to be pending an update bit event on R27, R28, R29, or R30.
        pub nfcu1: bool @ 8,
        /// Notch filter control bit
        /// 0 = disabled
        /// 1 = enabled
        pub nfcen: bool @ 7,
        /// Notch filter A0 coefficient most significant bits. See text and table for details.
        pub nfca0: u8 @ 0..=6,
    }
}

bitfield! {
    #[derive(Default)]
    pub struct NotchFilter2(pub u16): FromStorage, IntoStorage {
        /// Update bit feature for simultaneous change of all notch filter parameters. Write-only
        /// bit. Logic 1 on R28 register write operation causes new R28 value and any pending
        /// value in R27, R29, or R30 to go into effect. Logic 0 on R28 register write causes new
        /// value to be pending an update bit event on R27, R28, R29, or R30.
        pub nfcu2: bool @ 8,
        /// Notch filter A0 coefficient least significant bits. See text and table for details.
        pub nfca0: u8 @ 0..=6,
    }
}

bitfield! {
    #[derive(Default)]
    pub struct NotchFilter3(pub u16): FromStorage, IntoStorage {
        /// Update bit feature for simultaneous change of all notch filter parameters. Write-only
        /// bit. Logic 1 on R29 register write operation causes new R29 value and any pending
        /// value in R27, R28, or R30 to go into effect. Logic 0 on R29 register write causes new
        /// value to be pending an update bit event on R27, R28, R29, or R30.
        pub nfcu3: bool @ 8,
        /// Notch filter A1 coefficient most significant bits. See text and table for details.
        pub ncfa1: u8: 0..=6,
    }
}

bitfield! {
    #[derive(Default)]
    pub struct NotchFilter4(pub u16): FromStorage, IntoStorage {
        /// Update bit feature for simultaneous change of all notch filter parameters. Write-only
        /// bit. Logic 1 on R30 register write operation causes new R30 value and any pending
        /// value in R27, R28, or R29 to go into effect. Logic 0 on R30 register write causes new
        /// value to be pending an update bit event on R27, R28, R29, or R30.
        pub nfcu4: bool @ 8,
        /// Notch filter A1 coefficient least significant bits. See text and table for details.
        pub nfca1: u8 @ 0..=6,
    }
}

bitfield! {
    pub struct ACLControl1(pub u16): FromStorage, IntoStorage {
        /// Automatic Level Control function control bits
        /// 00 = right and left ALCs disabled
        /// 01 = only right channel ALC enabled
        /// 10 = only left channel ALC enabled
        /// 11 = both right and left channel ALCs enabled
        pub alcen: u8 @ 7..=8,
        /// Set maximum gain limit for PGA volume setting changes under ALC control
        /// 111 = +35.25dB (default)
        /// 110 = +29.25dB
        /// 101 = +23.25dB
        /// 100 = +17.25dB
        /// 011 = +11.25dB
        /// 010 = +5.25dB
        /// 001 = -0.75dB
        /// 000 = -6.75dB
        pub alcmxgain: u8 @ 3..=5,
        /// Set minimum gain value limit for PGA volume setting changes under ALC control
        /// 000 = -12dB (default)
        /// 001 = -6.0dB
        /// 010 = 0.0dB
        /// 011 = +6.0dB
        /// 100 = +12dB
        /// 101 = +18dB
        /// 110 = +24dB
        /// 111 = +30dB
        pub aclmngain: u8 @ 0..=2,
    }
}

impl Default for ACLControl1 {
    fn default() -> Self {
        Self(0x038 )
    }
}

bitfield! {
    pub struct ACLControl2(pub u16): FromStorage, IntoStorage {
        /// Hold time before ALC automated gain increase
        /// 0000 = 0.00ms (default)
        /// 0001 = 2.00ms
        /// 0010 = 4.00ms
        /// - time value doubles with each bit value increment –
        /// 1001 = 512ms
        /// 1010 through 1111 = 1000ms
        pub alcht: u8 @ 4..=7,
        /// ALC target level at ADC output
        /// 1111 = -1.5dB below full scale (FS)
        /// 1110 = -1.5dB FS (same value as 1111)
        /// 1101 = -3.0dB FS
        /// 1100 = -4.5dB FS
        /// 1011 = -6.0dB FS (default)
        /// - target level varies 1.5dB per binary step throughout control range –
        /// 0001 = -21.0dB FS
        /// 0000 = -22.5dB FS (lowest possible target signal level)
        pub alcsl: u8 @ 0..=3,
    }
}

impl Default for ACLControl2 {
    fn default() -> Self {
        Self(0x00B)
    }
}

bitfield! {
    pub struct ACLControl3(pub u16): FromStorage, IntoStorage {
        /// ALC mode control setting
        /// 0 = normal ALC operation
        /// 1 = Limiter Mode operation
        pub alcm: bool @ 8,
        /// ALC decay time duration per step of gain change for gain increase of 0.75dB of PGA
        /// gain. Total response time can be estimated by the total number of steps necessary to
        /// compensate for a given magnitude change in the signal. For example, a 6dB decrease
        /// in the signal would require eight ALC steps to compensate.
        /// Step size for each mode is given by:
        /// |= Normal Mode =|=             Limiter Mode              =|
        /// | 0000 = 500us  |  0000 = 125us                           |
        /// | 0001 = 1.0ms  |  0001 = 250us                           |
        /// | 0010 = 2.0ms  |  (default) 0010 = 500us (default)       |
        /// | ---- time value doubles with each binary bit value -----|
        /// | 1000 = 128ms  |  1000 = 32ms                            |
        /// | 1001 = 256ms  |  1001 = 64ms                            |
        /// | 1010 through  |  1111 = 512ms 1010 through 1111 = 128ms |
        pub alcdcy: u8 @ 4..=7,
        /// ALC attack time duration per step of gain change for gain decrease of 0.75dB of PGA
        /// gain. Total response time can be estimated by the total number of steps necessary to
        /// compensate for a given magnitude change in the signal. For example, a 6dB increase
        /// in the signal would require eight ALC steps to compensate.
        /// Step size for each mode is given by:
        /// |= Normal Mode  =|= Limiter Mode =|
        /// | 0000 = 125us  =|= 0000 = 31us   |
        /// | 0001 = 250us  =|= 0001 = 62us   |
        /// | 0010 = 500us  =|= (default) 0010 = 124us (default) |
        /// | ------- time value doubles with each binary bit value -------- |
        /// | 1000 = 26.5ms =|= 1000 = 7.95ms |
        /// | 1001 = 53.0ms =|= 1001 = 15.9ms |
        /// | 1010 through 1111 = 128ms 1010 through 1111 = 31.7ms |
        pub alcatk: bool @ 0..=3,
    }
}

impl Default for ACLControl3 {
    fn default() -> Self {
        Self(0x032)
    }
}

bitfield! {
    pub struct NoiseGate(pub u16): FromStorage, IntoStorage {
        /// ALC noise gate function control bit
        /// 0 = disabled
        /// 1 = enabled
        pub alcnen: bool @ 3,
        /// ALC noise gate threshold level
        /// 000 = -39dB (default)
        /// 001 = -45dB
        /// 010 = -51dB
        /// 011 = -57dB
        /// 100 = -63dB
        /// 101 = -69dB
        /// 110 = -75dB
        /// 111 = -81dB
        pub alcnth: u8 @ 0..=2,
    }
}

impl Default for NoiseGate {
    fn default() -> Self {
        Self(0x010)
    }
}

bitfield! {
    pub struct PllN(pub u16): FromStorage, IntoStorage {
        /// Control bit for divide by 2 pre-scale of MCLK path to PLL clock input
        /// 0 = MCLK divide by 1 (default)
        /// 1 = MCLK divide by 2
        pub pllmclk: bool @ 4,
        /// Integer portion of PLL input/output frequency ratio divider. Decimal value should be
        /// constrained to 6, 7, 8, 9, 10, 11, or 12. Default decimal value is 8. See text for
        /// details.
        pub plln: u8 @ 0..=3,
    }
}

impl Default for PllN {
    fn default() -> Self {
        Self(0x008 )
    }
}

bitfield! {
    pub struct PllK1(pub u16): FromStorage, IntoStorage {
        /// High order bits of fractional portion of PLL
        /// input/output frequency ratio divider.
        /// 
        /// See text for details.
        pub pllk: u8 @ 0..=5, // PLLK[23:18]
    }
}

impl Default for PllK1 {
    fn default() -> Self {
        Self(0x00C)
    }
}

bitfield! {
    pub struct PllK2(pub u16): FromStorage, IntoStorage {
        /// Middle order bits of fractional portion of PLL input/output frequency ratio divider.
        ///
        /// See text for details.
        pub pllk: u16 @ 0..=8, // PLLK[17:9] 
    }
}

impl Default for PllK2 {
    fn default() -> Self {
        Self(0x093)
    }
}

bitfield! {
    pub struct PllK3(pub u16): FromStorage, IntoStorage {
        /// Low order bits of fractional portion of PLL
        /// input/output frequency ratio divider.
        /// 
        /// See text for details.
        pub pllk: u16 @ 0..=8,
    }
}

impl Default for PllK3 {
    fn default() -> Self {
        Self(0x0E9)
    }
}

bitfield! {
    #[derive(Default)]
    pub struct Control3D(pub u16): FromStorage, IntoStorage {
        /// 3D Stereo Enhancement effect depth control
        /// 0000 = 0.0% effect (disabled, default)
        /// 0001 = 6.67% effect
        /// 0010 = 13.3% effect
        /// - effect depth varies by 6.67% per binary bit value –
        /// 1110 = 93.3% effect
        /// 1111 = 100% effect (maximum effect)
        pub depth3d: u8 @ 0..=4,
    }
}

bitfield! {
    #[derive(Default)]
    pub struct RightSpeakerSubmixer(pub u16): FromStorage, IntoStorage {
        /// Mutes the RMIX speaker signal gain stage output in the right speaker submixer
        /// 0 = gain stage output enabled
        /// 1 = gain stage output muted
        pub rmixmut: bool @ 5,
        /// Right speaker submixer bypass control
        /// 0 = right speaker amplifier directly connected to RMIX speaker signal gain stage
        /// 1 = right speaker amplifier connected to submixer output (inverts RMIX for BTL)
        pub rsubbyp: bool @ 4,
        /// RAUXIN to Right Speaker Submixer input gain control
        /// 000 = -15dB (default)
        /// 001 = -12dB
        /// 010 = -9.0dB
        /// 011 = -6.0dB
        /// 100 = -3.0dB
        /// 101 = 0.0dB
        /// 110 = +3.0dB
        /// 111 = +6.0dB
        pub rauxrsubg: u8 @ 1..=3,
        /// RAUXIN to Right Speaker Submixer mute control
        /// 0 = RAUXIN path to submixer is muted
        /// 1 = RAUXIN path to submixer is enabled
        pub rauxsmut: bool @ 0,
    }
}

bitfield! {
    pub struct InputControl(pub u16): FromStorage, IntoStorage {
        /// Microphone bias voltage selection control. Values change slightly with R40
        /// MISBIAS mode selection control. Open circuit voltage on MICBIAS pin#32 is
        /// shown as follows as a fraction of the VDDA pin#31 supply voltage.
        /// Normal Mode | Low Noise Mode
        /// 00 = 0.9x   | 00 = 0.85x
        /// 01 = 0.65x  | 01 = 0.60x
        /// 10 = 0.75x  | 10 = 0.70x
        /// 11 = 0.50x  | 11 = 0.50x
        pub micbiasv: u8 @ 7..=8,
        /// RLIN right line input path control to right PGA positive input
        /// 0 = RLIN not connected to PGA positive input (default)
        /// 1 = RLIN connected to PGA positive input
        pub rlinrpga: bool @ 6,
        /// RMICN right microphone negative input to right PGA negative input path control
        /// 0 = RMICN not connected to PGA negative input (default)
        /// 1 = RMICN connected to PGA negative input
        pub rmicnrpga: bool @ 5,
        /// RMICP right microphone positive input to right PGA positive input enable
        /// 0 = RMICP not connected to PGA positive input (default)
        /// 1 = RMICP connected to PGA positive input
        pub rmicrrpga: bool @ 4,
        /// LLIN right line input path control to left PGA positive input
        /// 0 = LLIN not connected to PGA positive input (default)
        /// 1 = LLIN connected to PGA positive input
        pub llinlpga: bool @ 2,
        /// LMICN left microphone negative input to left PGA negative input path control
        /// 0 = LMICN not connected to PGA negative input (default)
        /// 1 = LMICN connected to PGA negative input
        pub lmicnlpga: bool @ 1,
        /// LMICP left microphone positive input to left PGA positive input enable
        /// 0 = LMICP not connected to PGA positive input (default)
        /// 1 = LMICP connected to PGA positive input
        pub lmicpgpga: bool @ 0,

    }
}

impl Default for InputControl {
    fn default() -> Self {
        Self(0x033)
    }
}

bitfield! {
    pub struct LeftInputPGAGain(pub u16): FromStorage, IntoStorage {
        /// PGA volume update bit feature. Write-only bit for synchronized L/R PGA changes
        /// If logic = 0 on R45 write, new R45 value stored in temporary register
        /// If logic = 1 on R45 write, new R45 and pending R46 values become active
        pub lpgua: bool @ 8,
        /// Left channel input zero cross detection enable
        /// 0 = gain changes to PGA register happen immediately (default)
        /// 1 = gain changes to PGA happen pending zero crossing logic
        pub lpgazc: bool @ 7,
        /// Left channel mute PGA mute control
        /// 0 = PGA not muted, normal operation (default)
        /// 1 = PGA in muted condition not connected to LADC Mix/Boost stage
        pub lpgamt: bool @ 6,
        /// Left channel input PGA volume control setting. Setting becomes active when allowed
        /// by zero crossing and/or update bit features.
        /// 01 0000 = 0.0dB default setting
        /// 00 0000 = -12dB
        /// 00 0001 = -11.25dB
        /// - volume changes in 0.75dB steps per binary bit value –
        /// 11 1110 = +34.50dB
        /// 11 1111 = +35.25dB
        pub lpgagain: u8 @ 0..=5,
    }
}

impl Default for LeftInputPGAGain {
    fn default() -> Self {
        Self(0x010)
    }
}

bitfield! {
    pub struct RightInputPGAGain(pub u16): FromStorage, IntoStorage {
        /// PGA volume update bit feature. Write-only bit for synchronized L/R PGA changes
        /// If logic = 0 on R46 write, new R46 value stored in temporary register
        /// If logic = 1 on R46 write, new R46 and pending R45 values become active
        pub rpgau: bool @ 8,
        /// Right channel input zero cross detection enable
        /// 0 = gain changes to PGA register happen immediately
        /// 1 = gain changes to PGA happen pending zero crossing logic
        pub rpgazc: bool @ 7,
        /// Right channel mute PGA mute control
        /// 0 = PGA not muted, normal operation (default)
        /// 1 = PGA in muted condition not connected to RADC Mix/Boost stage
        pub rpgamt: bool @ 6,
        /// Right channel input PGA volume control setting. Setting becomes active when
        /// allowed by zero crossing and/or update bit features.
        /// 01 0000 = 0.0dB default setting
        /// 00 0000 = -12dB
        /// 00 0001 = -11.25dB
        /// - volume changes in 0.75dB steps per binary bit value –
        /// 11 1110 = +34.50dB
        /// 11 1111 = +35.25dB
        pub rpgagain: u8 @ 0..=5,
    }
}

impl Default for RightInputPGAGain {
    fn default() -> Self {
        Self(0x010)
    }
}

bitfield! {
    pub struct LeftADCBoost(pub u16): FromStorage, IntoStorage {
        /// Left channel PGA boost control
        /// 0 = no gain between PGA output and LPGA Mix/Boost stage input
        /// 1 = +20dB gain between PGA output and LPGA Mix/Boost stage input
        pub lpgabst: bool @ 8,
        /// Gain value between LLIN line input and LPGA Mix/Boost stage input
        /// 000 = path disconnected (default)
        /// 001 = -12dB
        /// 010 = -9.0dB
        /// 011 = -6.0dB
        /// 100 = -3.0dB
        /// 101 = 0.0dB
        /// 110 = +3.0dB
        /// 111 = +6.0dB
        pub lpgabstgaun: u8 @ 4..=6,
        /// Gain value between LAUXIN auxiliary input and LPGA Mix/Boost stage input
        /// 000 = path disconnected (default)
        /// 001 = -12dB
        /// 010 = -9.0dB
        /// 011 = -6.0dB
        /// 100 = -3.0dB
        /// 101 = 0.0dB
        /// 110 = +3.0dB
        /// 111 = +6.0dB
        pub lauxbstegain: u8 @ 0..=2,
    }
}

impl Default for LeftADCBoost {
    fn default() -> Self {
        Self(0x100)
    }
}

bitfield! {
    pub struct RightADCBoost(pub u16): FromStorage, IntoStorage {
        /// Right channel PGA boost control
        /// 0 = no gain between PGA output and RPGA Mix/Boost stage input
        /// 1 = +20dB gain between PGA output and RPGA Mix/Boost stage input
        pub rpgabst: bool @ 8,
        /// Gain value between RLIN line input and RPGA Mix/Boost stage input
        /// 000 = path disconnected (default)
        /// 001 = -12dB
        /// 010 = -9.0dB
        /// 011 = -6.0dB
        /// 100 = -3.0dB
        /// 101 = 0.0dB
        /// 110 = +3.0dB
        /// 111 = +6.0dB
        pub rpgabstgain: u8 @ 4..=6,
        /// Gain value between RAUXIN auxiliary input and RPGA Mix/Boost stage input
        /// 000 = path disconnected (default)
        /// 001 = -12dB
        /// 010 = -9.0dB
        /// 011 = -6.0dB
        /// 100 = -3.0dB
        /// 101 = 0.0dB
        /// 110 = +3.0dB
        /// 111 = +6.0dB
        pub rauxbstgain: u8 @ 0..=2,
    }
}

impl Default for RightADCBoost {
    fn default() -> Self {
        Self(0x100)
    }
}

bitfield! {
    pub struct OutputControl(pub u16): FromStorage, IntoStorage {
        /// Left DAC output to RMIX right output mixer cross-coupling path control
        /// 0 = path disconnected (default)
        /// 1 = path connected
        pub ldacrmx: bool @ 6,
        /// Right DAC output to LMIX left output mixer cross-coupling path control
        /// 0 = path disconnected (default)
        /// 1 = path connected
        pub rdaclmx: bool @ 5,
        /// AUXOUT1 gain boost control
        /// 0 = preferred setting for 3.6V and lower operation, -1.0x gain (default)
        /// 1 = required setting for greater than 3.6V operation, +1.5x gain
        pub aux1bst: bool @ 4,
        /// AUXOUT2 gain boost control
        /// 0 = preferred setting for 3.6V and lower operation, -1.0x gain (default)
        /// 1 = required setting for greater than 3.6V operation, +1.5x gain
        pub aux2bst: bool @ 3,
        /// LSPKOUT and RSPKOUT speaker amplifier gain boost control
        /// 0 = preferred setting for 3.6V and lower operation, -1.0x gain (default)
        /// 1 = required setting for greater than 3.6V operation, +1.5x gain
        pub spkbst: bool @ 2,
        /// Thermal shutdown enable protects chip from thermal destruction on overload
        /// 0 = disable thermal shutdown (engineering purposes, only)
        /// 1 = enable (default) strongly recommended for normal operation
        pub tsen: bool @ 1,
        /// Output resistance control option for tie-off of unused or disabled outputs. Unused
        /// outputs tie to internal voltage reference for reduced pops and clicks.
        /// 0 = nominal tie-off impedance value of 1kΩ (default)
        /// 1 = nominal tie-off impedance value of 30kΩ
        pub aoutimp: bool @ 0,
    }
}

impl Default for OutputControl {
    fn default() -> Self {
        Self(0x002)
    }
}

bitfield! {
    pub struct LeftMixer(pub u16): FromStorage, IntoStorage {
        /// Gain value between LAUXIN auxiliary input and input to LMAIN left output mixer
        /// 000 = -15dB (default)
        /// 001 = -12dB
        /// 010 = -9.0dB
        /// 011 = -6.0dB
        /// 100 = -3.0dB
        /// 101 = 0.0dB
        /// 110 = +3.0dB
        /// 111 = +6.0dB
        pub lauxmxgain: u8 @ 6..=8,
        /// LAUXIN input to LMAIN left output mixer path control
        /// 0 = LAUXIN not connected to LMAIN left output mixer (default)
        /// 1 = LAUXIN connected to LMAIN left output mixer
        pub lauxlmx: bool @ 5,
        /// Gain value for bypass from LADC Mix/Boost output to LMAIN left output mixer.
        /// 000 = -15dB (default)
        /// 001 = -12dB
        /// 010 = -9.0dB
        /// 011 = -6.0dB
        /// 100 = -3.0dB
        /// 101 = 0.0dB
        /// 110 = +3.0dB
        /// 111 = +6.0dB
        pub lbypmxgain: u8 @ 2..=3,
        /// Left bypass path control from LADC Mix/Boost output to LMAIN left output mixer
        ///0 = path not connected
        /// 1 = bypass path connected
        pub lbyplmx: bool @ 1,
        /// Left DAC output to LMIX left output mixer path control
        /// 0 = path disconnected (default)
        /// 1 = path connected
        pub ldaclmx: bool @ 0,
    }
}

impl Default for LeftMixer {
    fn default() -> Self {
        Self(0x001)
    }
}

bitfield! {
    pub struct RightMixer(pub u16): FromStorage, IntoStorage {
        /// Gain value between LAUXIN auxiliary input and input to LMAIN left output mixer
        /// 000 = -15dB (default)
        /// 001 = -12dB
        /// 010 = -9.0dB
        /// 011 = -6.0dB
        /// 100 = -3.0dB
        /// 101 = 0.0dB
        /// 110 = +3.0dB
        /// 111 = +6.0dB
        pub rauxmxgain: u8 @ 6..=8,
        /// RAUXIN input to RMAIN right output mixer path control
        /// 0 = RAUXIN not connected to RMAIN right output mixer (default)
        /// 1 = RAUXIN connected to RMAIN right output mixer
        pub rauxrmx: bool @ 5,
        /// Gain value for bypass from LADC Mix/Boost output to LMAIN left output mixer.
        /// 000 = -15dB (default)
        /// 001 = -12dB
        /// 010 = -9.0dB
        /// 011 = -6.0dB
        /// 100 = -3.0dB
        /// 101 = 0.0dB
        /// 110 = +3.0dB
        /// 111 = +6.0dB
        pub rbypmxgain: u8 @ 2..=4,
        /// Right bypass path control from RADC Mix/Boost output to RMAIN r output mixer
        /// 0 = path not connected
        /// 1 = bypass path connected
        pub rbyprmx: bool @ 1,
        /// Right DAC output to RMIX right output mixer path control
        /// 0 = path disconnected (default)
        /// 1 = path connected
        pub rdacrmx: bool @ 0,
    }
}

impl Default for RightMixer {
    fn default() -> Self {
        Self(0x001)
    }
}

bitfield! {
    pub struct LHPVolume(pub u16): FromStorage, IntoStorage {
        /// Headphone output volume update bit feature. Write-only bit for synchronized
        /// changes of left and right headphone amplifier output settings
        /// If logic = 0 on R52 write, new R52 value stored in temporary register
        /// If logic = 1 on R52 write, new R52 and pending R53 values become active
        pub lhpvu: bool @ 8,
        /// Left channel input zero cross detection enable
        /// 0 = gain changes to left headphone happen immediately (default)
        /// 1 = gain changes to left headphone happen pending zero crossing logic
        pub lhpzc: bool @ 7,
        /// Left headphone output mute control
        /// 0 = headphone output not muted, normal operation (default)
        /// 1 = headphone in muted condition not connected to LMIX output stage
        pub lhpmute: bool @ 6,
        /// Left channel headphone output volume control setting. Setting becomes active when
        /// allowed by zero crossing and/or update bit features.
        /// 11 1001 = 0.0dB default setting
        /// 00 0000 = -57dB
        /// 00 0001 = -56dB
        /// - volume changes in 1.0dB steps per binary bit value –
        /// 11 1110 = +5.0dB
        /// 11 1111 = +6.0dB
        pub lhpgain: u8 @ 0..=5,
    }
}

impl Default for LHPVolume {
    fn default() -> Self {
        Self(0x039)
    }
}

bitfield! {
    pub struct RHPVolume(pub u16): FromStorage, IntoStorage {
        /// Headphone output volume update bit feature. Write-only bit for synchronized
        /// changes of left and right headphone amplifier output settings
        /// If logic = 0 on R53 write, new R53 value stored in temporary register
        /// If logic = 1 on R53 write, new R53 and pending R52 values become active
        pub rhpvu: bool @ 8,
        /// Right channel input zero cross detection enable
        /// 0 = gain changes to right headphone happen immediately (default)
        /// 1 = gain changes to right headphone happen pending zero crossing logic
        pub rhpzc: bool @ 7,
        /// Right headphone output mute control
        /// 0 = headphone output not muted, normal operation (default)
        /// 1 = headphone in muted condition not connected to RMIX output stage
        pub rhpmute: bool @ 6,
        /// Right channel headphone output volume control setting. Setting becomes active when
        /// allowed by zero crossing and/or update bit features.
        /// 11 1001 = 0.0dB default setting
        /// 00 0000 = -57dB
        /// 00 0001 = -56dB
        /// - volume changes in 1.0dB steps per binary bit value –
        /// 11 1110 = +5.0dB
        /// 11 1111 = +6.0dB
        pub rhpgain: u8 @ 0..=5,
    }
}

impl Default for RHPVolume {
    fn default() -> Self {
        Self(0x039)
    }
}

bitfield! {
    pub struct LSPKOutVolume(pub u16): FromStorage, IntoStorage {
        /// Loudspeaker output volume update bit feature. Write-only bit for synchronized
        /// changes of left and right headphone amplifier output settings
        /// If logic = 0 on R54 write, new R54 value stored in temporary register
        /// If logic = 1 on R54 write, new R54 and pending R55 values become active
        pub lspkvu: bool @ 8,
        /// Left loudspeaker LSPKOUT output zero cross detection enable
        /// 0 = gain changes to left loudspeaker happen immediately (default)
        /// 1 = gain changes to left loudspeaker happen pending zero crossing logic
        pub lspkzc: bool @ 7,
        /// Right loudspeaker LSPKOUT output mute control
        /// 0 = loudspeaker output not muted, normal operation (default)
        /// 1 = loudspeaker in muted condition
        pub lspkmute: bool @ 6,
        /// Left loudspeaker output volume control setting. Setting becomes active when allowed
        /// by zero crossing and/or update bit features.
        /// 11 1001 = 0.0dB default setting
        /// 00 0000 = -57dB
        /// 00 0001 = -56dB
        /// - volume changes in 1.0dB steps per binary bit value –
        /// 11 1110 = +5.0dB
        /// 11 1111 = +6.0dB
        pub lspkgain: u8 @ 0..=5,
    }
}

impl Default for LSPKOutVolume {
    fn default() -> Self {
        Self(0x039)
    }
}

bitfield! {
    pub struct RSPKOutVolume(pub u16): FromStorage, IntoStorage {
        /// Loudspeaker output volume update bit feature. Write-only bit for synchronized
        /// changes of left and right headphone amplifier output settings
        /// If logic = 0 on R55 write, new R55 value stored in temporary register
        /// If logic = 1 on R55 write, new R55 and pending R54 values become active
        pub rspkvu: bool @ 8,
        /// Right loudspeaker RSPKOUT output zero cross detection enable
        /// 0 = gain changes to right loudspeaker happen immediately (default)
        /// 1 = gain changes to right loudspeaker happen pending zero crossing logic
        pub rspkzc: bool @ 7,
        /// Right loudspeaker RSPKOUT output mute control
        /// 0 = loudspeaker output not muted, normal operation (default)
        /// 1 = loudspeaker in muted condition
        pub rspkmute: bool @ 6,
        /// Right loudspeaker output volume control setting. Setting becomes active when
        /// allowed by zero crossing and/or update bit features.
        /// 11 1001 = 0.0dB default setting
        /// 00 0000 = -57dB
        /// 00 0001 = -56dB
        /// - volume changes in 1.0dB steps per binary bit value –
        /// 11 1110 = +5.0dB
        /// 11 1111 = +6.0dB
        pub rspkgain: u8 @ 0..=5,
    }
}

impl Default for RSPKOutVolume {
    fn default() -> Self {
        Self(0x039)
    }
}

bitfield! {
    pub struct AUX2Mixer(pub u16): FromStorage, IntoStorage {
        /// AUXOUT2 output mute control
        /// 0 = output not muted, normal operation (default)
        /// 1 = output in muted condition
        pub auxout2mt: bool @ 6,
        /// AUX1 Mixer output to AUX2 MIXER input path control
        /// 0 = path not connected
        /// 1 = path connected
        pub aux1mix_2: bool @ 3,
        /// Left LADC Mix/Boost output LINMIX path control to AUX2 MIXER input
        /// 0 = path not connected
        /// 1 = path connected
        pub ldacaux: bool @ 2,
        /// Left LMAIN MIXER output to AUX2 MIXER input path control
        /// 0 = path not connected
        /// 1 = path connected
        pub lmixaux2: bool @ 1,
        /// Left DAC output to AUX2 MIXER input path control
        /// 0 = path not connected
        /// 1 = path connected
        pub ldacaux2: bool @ 0,
    }
}

impl Default for AUX2Mixer {
    fn default() -> Self {
        Self(0x001)
    }
}

bitfield! {
    pub struct AUX1Mixer(pub u16): FromStorage, IntoStorage {
        /// AUXOUT1 output mute control
        /// 0 = output not muted, normal operation (default)
        /// 1 = output in muted condition
        pub auxiut1mt: bool @ 6,
        /// AUXOUT1 6dB attenuation enable
        /// 0 = output signal at normal gain value (default)
        /// 1 = output signal attenuated by 6.0dB
        pub aux1half: bool @ 5,
        /// Left LMAIN MIXER output to AUX1 MIXER input path control
        /// 0 = path not connected
        /// 1 = path connected
        pub lmixaux1: bool @ 4,
        /// Left DAC output to AUX1 MIXER input path control
        /// 0 = path not connected
        /// 1 = path connected
        pub ldacaux1: bool @ 3,
        /// Right RADC Mix/Boost output RINMIX path control to AUX1 MIXER input
        /// 0 = path not connected
        /// 1 = path connected
        pub radcaux1: bool @ 2,
        /// Right RMIX output to AUX1 MIXER input path control
        /// 0 = path not connected
        /// 1 = path connected
        pub rmixaux1: bool @ 1,
        /// Right DAC output to AUX1 MIXER input path control
        /// 0 = path not connected
        /// 1 = path connected
        pub rdacaux1: bool @ 0,
    }
}

impl Default for AUX1Mixer {
    fn default() -> Self {
        Self(0x001)
    }
}

bitfield! {
    #[derive(Default)]
    pub struct PowerManagement4(pub u16): FromStorage, IntoStorage {
        /// Reduce DAC supply current 50% in low power operating mode
        /// 0 = normal supply current operation (default)
        /// 1 = 50% reduced supply current mode
        pub lpdac: bool @ 8,
        /// Reduce ADC Mix/Boost amplifier supply current 50% in low power operating mode
        /// 0 = normal supply current operation (default)
        /// 1 = 50% reduced supply current mode
        pub lpipbst: bool @ 7,
        /// Reduce ADC supply current 50% in low power operating mode
        /// 0 = normal supply current operation (default)
        /// 1 = 50% reduced supply current mode
        pub lpadc: bool @ 6,
        /// Reduce loudspeaker amplifier supply current 50% in low power operating mode
        /// 0 = normal supply current operation (default)
        /// 1 = 50% reduced supply current mode
        pub lpspkd: bool @ 5,
        /// Microphone bias optional low noise mode configuration control
        /// 0 = normal configuration with low-Z micbias output impedance
        /// 1 = low noise configuration with 200-ohm micbias output impedance
        pub micbiasm: bool @ 4,
        /// Regulator voltage control power reduction options
        /// 00 = normal 1.80Vdc operation (default)
        /// 01 = 1.61Vdc operation
        /// 10 = 1.40 Vdc operation
        /// 11 = 1.218 Vdc operation
        pub regvolt: u8 @ 2..=3,
        /// Master bias current power reduction options
        /// 00 = normal operation (default)
        /// 01 = 25% reduced bias current from default
        /// 10 = 14% reduced bias current from default
        /// 11 = 25% reduced bias current from default
        pub ibadj: u8 @ 0..=1,

    }
}

bitfield! {
    #[derive(Default)]
    pub struct LeftTimeSlot(pub u16): FromStorage, IntoStorage {
        /// Left channel PCM time slot start count: LSB portion of total number of bit times to
        /// wait from frame sync before clocking audio channel data. LSB portion is combined
        /// with MSB from R60 to get total number of bit times to wait.
        pub ltslot: u16 @ 0..=8,
    }
}

bitfield! {
    pub struct Misc(pub u16): FromStorage, IntoStorage {
        /// Time slot function enable for PCM mode.
        pub pcmtsen: bool @ 8,
        /// Tri state ADC out after second half of LSB enable
        pub tri: bool @ 7,
        /// 8-bit word length enable
        pub pcm8bit: bool @ 6,
        /// ADCOUT output driver
        /// 1 = enabled (default)
        /// 0 = disabled (driver in high-z state)
        pub puden: bool @ 5,
        /// ADCOUT passive resistor pull-up or passive pull-down enable
        /// 0 = no passive pull-up or pull-down on ADCOUT pin
        /// 1 = passive pull-up resistor on ADCOUT pin if PUDPS = 1
        /// 1 = passive pull-down resistor on ADCOUT pin if PUDPS = 0
        pub pudpe: bool @ 4,
        /// ADCOUT passive resistor pull-up or pull-down selection
        /// 0 = passive pull-down resistor applied to ADCOUT pin if PUDPE = 1
        /// 1 = passive pull-down resistor applied to ADCOUT pin if PUDPE = 1
        pub pudps: bool @ 3,
        /// Right channel PCM time slot start count: MSB portion of total number of bit times to
        /// wait from frame sync before clocking audio channel data. MSB is combined with
        /// LSB portion from R61 to get total number of bit times to wait.
        pub rtslot: bool @ 1, // RTSLOT[9]
        /// Left channel PCM time slot start count: MSB portion of total number of bit times to
        /// wait from frame sync before clocking audio channel data. MSB is combined with
        /// LSB portion from R59 to get total number of bit times to wait.
        pub ltslot: bool @ 0, // LTSLOT[9]
    }
}

impl Default for Misc {
    fn default() -> Self {
        Self(0x020)
    }
}

bitfield! {
    #[derive(Default)]
    pub struct RightTimeSlot(pub u16): FromStorage, IntoStorage {
        /// Right channel PCM time slot start count: LSB portion of total number of bit times to
        /// wait from frame sync before clocking audio channel data. LSB portion is combined
        /// with MSB from R60 to get total number of bit times to wait.
        pub rtslot: u16 @ 0..=8,
    }
}

bitfield! {
    pub struct DeviceRevisionNumber(pub u16): FromStorage, IntoStorage {
        /// Device Revision Number for readback over control interface = read-only value
        pub rev: u8 @ 0..=7,
    }
}

impl Default for DeviceRevisionNumber {
    fn default() -> Self {
        Self(0x07F) // RevA code
    }
}

bitfield! {
    pub struct DeviceID(pub u16): FromStorage, IntoStorage {
        /// 0x01A Device ID equivalent to control bus address = read-only value
        pub id: u16 @ 0..=8,
    }
}

bitfield! {
    pub struct DACDither(pub u16): FromStorage, IntoStorage {
        /// Dither added to DAC modulator to eliminate all non-random noise
        /// 0 0000 = dither off
        /// 1 0001 = nominal optimal dither
        /// 1 1111 = maximum dither
        pub mod_dither: u8 @ 5..=8,
        /// Dither added to DAC analog output to eliminate all non-random noise
        /// 0000 = dither off
        /// 0100 = nominal optimal dither
        /// 1111 = maximum dither
        pub analog_dither: u8 @ 0..=4,
    }
}

impl Default for DACDither {
    fn default() -> Self {
        Self(0x114)
    }
}

bitfield! {
    #[derive(Default)]
    pub struct ALCEnhancement1(pub u16): FromStorage, IntoStorage {
        /// Selects one of two tables used to set the target level for the ALC
        /// 0 = default recommended target level table spanning -1.5dB through -22.5dB FS
        /// 1 = optional ALC target level table spanning -6.0dB through -28.5dB FS
        pub alctblsel: bool @ 8,
        /// Choose peak or peak-to-peak value for ALC threshold logic
        /// 0 = use rectified peak detector output value
        /// 1 = use peak-to-peak detector output value
        pub alcpksel: bool @ 7,
        /// Choose peak or peak-to-peak value for Noise Gate threshold logic
        /// 0 = use rectified peak detector output value
        /// 1 = use peak-to-peak detector output value
        pub alcngset: bool @ 6,
        /// Real time readout of instantaneous gain value used by left channel PGA
        pub alcgainl: u8 @ 0..=5,
    }
}

bitfield! {
    #[derive(Default)]
    pub struct ALCEnhancement2(pub u16): FromStorage, IntoStorage {
        /// Enable control for ALC fast peak limiter function
        /// 0 = enabled (default)
        /// 1 = disabled
        pub pklimena: bool @ 8,
        /// Real time readout of instantaneous gain value used by right channel PGA
        pub alcgainr: u8 @ 0..=5,
    }
}

bitfield! {
    #[derive(Debug)]
    pub struct MiscControls(pub u16): FromStorage, IntoStorage {
        /// Set SPI control bus mode regardless of state of Mode pin
        /// 0 = normal operation (default)
        /// 1 = force SPI 4-wire mode regardless of state of Mode pin
        pub w4spiena: bool @ 8,
        /// Short frame sync detection period value
        /// 00 = trigger if frame time less than 255 MCLK edges
        /// 01 = trigger if frame time less than 253 MCLK edges
        /// 10 = trigger if frame time less than 254 MCLK edges
        /// 11 = trigger if frame time less than 255 MCLK edges
        pub fserrval: u8 @ 6..=7,
        /// Enable DSP state flush on short frame sync event
        /// 0 = ignore short frame sync events (default)
        /// 1 = set DSP state to initial conditions on short frame sync event
        pub fserflsh: bool @ 5,
        /// Enable control for short frame cycle detection logic
        /// 0 = short frame cycle detection logic enabled
        /// 1 = short frame cycle detection logic disabled
        pub fserrena: bool @ 4,
        /// Enable control to delay use of notch filter output when filter is enabled
        /// 0 = delay using notch filter output 512 sample times after notch enabled (default)
        /// 1 = use notch filter output immediately after notch filter is enabled
        pub notchdly: bool @ 3,
        /// Enable control to mute DAC limiter output when softmute is enabled
        /// 0 = DAC limiter output may not move to exactly zero during Softmute (default)
        /// 1 = DAC limiter output muted to exactly zero during softmute
        pub dacinmute: bool @ 2,
        /// Enable control to use PLL output when PLL is not in phase locked condition
        /// 0 = PLL VCO output disabled when PLL is in unlocked condition (default)
        /// 1 = PLL VCO output used as-is when PLL is in unlocked condition
        pub plllockbp: bool @ 1,
        /// Set DAC to use 256x oversampling rate (best at lower sample rates)
        /// 0 = Use oversampling rate as determined by Register 0x0A[3] (default)
        /// 1 = Set DAC to 256x oversampling rate regardless of Register 0x0A[3]
        pub dacosr256: bool @ 0,
    }
}

bitfield! {
    #[derive(Default)]
    pub struct InputTieOffDirectManualControl(pub u16): FromStorage, IntoStorage {
        /// Enable direct control over input tie-off resistor switching
        /// 0 = ignore Register 0x4A bits to control input tie-off resistor switching
        /// 1 = use Register 0x4A bits to override automatic tie-off resistor switching
        pub maninena: bool @ 8,
        /// If MANUINEN = 1, use this bit to control right aux input tie-off resistor switch
        /// 0 = Tie-off resistor switch for RAUXIN input is forced open
        /// 1 = Tie-off resistor switch for RAUXIN input is forced closed
        pub manraux: bool @ 7,
        /// If MANUINEN = 1, use this bit to control right line input tie-off resistor switch
        /// 0 = Tie-off resistor switch for RLIN input is forced open
        /// 1 = Tie-off resistor switch for RLIN input is forced closed
        pub manrlin: bool @ 6,
        /// If MANUINEN = 1, use this bit to control right PGA inverting input tie-off switch
        /// 0 = Tie-off resistor switch for RMICN input is forced open
        /// 1 = Tie-off resistor switch for RMICN input is forced closed
        pub manrmicn: bool @ 5,
        /// If MANUINEN =1, use this bit to control right PGA non-inverting input tie-off switch
        /// 0 = Tie-off resistor switch for RMICP input is forced open
        /// 1 = Tie-off resistor switch for RMICP input is forced closed
        pub manrmicp: bool @ 4,
        /// If MANUINEN = 1, use this bit to control left aux input tie-off resistor switch
        /// 0 = Tie-off resistor switch for LAUXIN input is forced open
        /// 1 = Tie-off resistor switch for RAUXIN input is forced closed
        pub manlaux: bool @ 3,
        /// If MANUINEN = 1, use this bit to control left line input tie-off resistor switch
        /// 0 = Tie-off resistor switch for LLIN input is forced open
        /// 1 = Tie-off resistor switch for LLIN input is forced closed
        pub manllin: bool @ 2,
        /// If MANUINEN = 1, use this bit to control left PGA inverting input tie-off switch
        /// 0 = Tie-off resistor switch for LMICN input is forced open
        /// 1 = Tie-off resistor switch for LMINN input is forced closed
        pub manlmicn: bool @ 1,
        /// If MANUINEN = 1, use this bit to control left PGA non-inverting input tie-off switch
        /// 0 = Tie-off resistor switch for LMICP input is forced open
        /// 1 = Tie-off resistor switch for LMICP input is forced closed
        pub manlminp: bool @ 0,
    }
}

bitfield! {
    #[derive(Default)]
    pub struct ReductionAndOutputTieOffDirectManualControl(pub u16): FromStorage, IntoStorage {
        /// Reduce bias current to left and right input MIX/BOOST stage
        /// 0 = normal bias current
        /// 1 = bias current reduced by 50% for reduced power and bandwidth
        pub ibthalfi: bool @ 8,
        /// Increase bias current to left and right input MIX/BOOST stage
        /// 0 = normal bias current
        /// 1 = bias current increased by 500 microamps
        pub ibt500up: bool @ 6,
        /// Decrease bias current to left and right input MIX/BOOST stage
        /// 0 = normal bias current
        /// 1 = bias current reduced by 250 microamps
        pub ibt250dn: bool @ 5,
        /// Direct manual control to turn on bypass switch around input tie-off buffer amplifier
        /// 0 = normal automatic operation of bypass switch
        /// 1 = bypass switch in closed position when input buffer amplifier is disabled
        pub maninbbp: bool @ 4,
        /// Direct manual control to turn on switch to ground at input tie-off buffer amp output
        /// 0 = normal automatic operation of switch to ground
        /// 1 = switch to ground in in closed position when input buffer amplifier is disabled
        pub maninpad: bool @ 3,
        /// Direct manual control of switch for Vref 600k-ohm resistor to ground
        /// 0 = switch to ground controlled by Register 0x01 setting
        /// 1 = switch to ground in the closed positioin
        pub manvrefh: bool @ 2,
        /// Direct manual control for switch for Vref 160k-ohm resistor to ground
        /// 0 = switch to ground controlled by Register 0x01 setting
        /// 1 = switch to ground in the closed position
        pub manvrefm: bool @ 1,
        /// Direct manual control for switch for Vref 6k-ohm resistor to ground
        /// 0 = switch to ground controlled by Register 0x01 setting
        /// 1 = switch to ground in the closed position
        pub manvrefl: bool @ 0,
    }
}

bitfield! {
    pub struct AGCPeakToPeakReadout(pub u16): FromStorage, IntoStorage {
        /// Read-only register which outputs the instantaneous value contained in the peak-to-
        /// peak amplitude register used by the ALC for signal level dependent logic. Value is
        /// highest of left or right input when both inputs are under ALC control.
        pub p2pval: u16 @ 0..=8,
    }
}

bitfield! {
    pub struct AGCPeakDetectorReadout(pub u16): FromStorage, IntoStorage {
        /// Read-only register which outputs the instantaneous value contained in the peak
        /// detector amplitude register used by the ALC for signal level dependent logic. Value
        /// is highest of left or right input when both inputs are under ALC control.
        pub peakval: u16 @ 0..=8,
    }
}

bitfield! {
    #[derive(Default)]
    pub struct AutomuteControlAndStatusReadout(pub u16): FromStorage, IntoStorage {
        /// Select observation point used by DAC output automute feature
        /// 0 = automute operates on data at the input to the DAC digital attenuator (default)
        /// 1 = automute operates on data at the DACIN input pin
        pub amutectl: bool @ 5,
        /// Read-only status bit of high voltage detection circuit monitoring VDDSPK voltage
        /// 0 = voltage on VDDSPK pin measured at approximately 4.0Vdc or less
        /// 1 = voltage on VDDSPK pin measured at approximately 4.0Vdc or greater
        pub hvdet: bool @ 4,
        /// Read-only status bit of logic controlling the noise gate function
        /// 0 = signal is greater than the noise gate threshold and ALC gain can change
        /// 1 = signal is less than the noise gate threshold and ALC gain is held constant
        pub nsgate: bool @ 3,
        /// Read-only status bit of analog mute function applied to DAC channels
        /// 0 = not in the automute condition
        /// 1 = in automute condition
        pub anamute: bool @ 2,
        /// Read-only status bit of digital mute function of the left channel DAC
        /// 0 = digital gain value is greater than zero
        /// 1 = digital gain is zero either by direct setting or operation of softmute function
        pub digmutel: bool @ 1,
        /// Read-only status bit of digital mute function of the left channel DAC
        /// 0 = digital gain value is greater than zero
        /// 1 = digital gain is zero either by direct setting or operation of softmute function
        pub digmuter: bool @ 0,
    }
}

bitfield! {
    #[derive(Default)]
    pub struct OutputTieOffDirectManualControls(pub u16): FromStorage, IntoStorage {
        /// Enable direct control over output tie-off resistor switching
        /// 0 = ignore Register 0x4F bits to control input tie-off resistor/buffer switching
        /// 1 = use Register 0x4F bits to override automatic tie-off resistor/buffer switching
        pub manouten: bool @ 8,
        /// If MANUOUTEN = 1, use this bit to control bypass switch around 1.5x boosted
        /// output tie-off buffer amplifier
        /// 0 = normal automatic operation of bypass switch
        /// 1 = bypass switch in closed position when output buffer amplifier is disabled.
        pub shrtbufh: bool @ 7,
        /// If MANUOUTEN = 1, use this bit to control bypass switch around 1.0x non-boosted
        /// output tie-off buffer amplifier
        /// 0 = normal automatic operation of bypass switch
        /// 1 = bypass switch in closed position when output buffer amplifier is disabled
        pub shrtbufl: bool @ 6,
        /// If MANUOUTEN = 1, use this bit to control left speaker output tie-off resistor switch
        /// 0 = tie-off resistor switch for LSPKOUT speaker output is forced open
        /// 1 = tie-off resistor switch for LSPKOUT speaker output is forced closed
        pub shrtlspk: bool @ 5,
        /// If MANUOUTEN = 1, use this bit to control left speaker output tie-off resistor switch
        /// 0 = tie-off resistor switch for RSPKOUT speaker output is forced open
        /// 1 = tie-off resistor switch for RSPKOUT speaker output is forced closed
        pub shrtrspk: bool @ 4,
        /// If MANUOUTEN = 1, use this bit to control Auxout1 output tie-off resistor switch
        /// 0 = tie-off resistor switch for AUXOUT1 output is forced open
        /// 1 = tie-off resistor switch for AUXOUT1 output is forced closed
        pub shrtaux1: bool @ 3,
        /// If MANUOUTEN = 1, use this bit to control Auxout2 output tie-off resistor switch
        /// 0 = tie-off resistor switch for AUXOUT2 output is forced open
        /// 1 = tie-off resistor switch for AUXOUT2 output is forced closed
        pub shrtaux2: bool @ 2,
        /// If MANUOUTEN = 1, use this bit to control left headphone output tie-off switch
        /// 0 = tie-off resistor switch for LHP output is forced open
        /// 1 = tie-off resistor switch for LHP output is forced closed
        pub shrtlhp: bool @ 1,
        /// If MANUOUTEN = 1, use this bit to control right headphone output tie-off switch
        /// 0 = tie-off resistor switch for RHP output is forced open
        /// 1 = tie-off resistor switch for RHP output is forced closed
        pub shrtrhp: bool @ 0,
    }
}

