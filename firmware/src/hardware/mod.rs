pub mod is42s16160j_7; // SDRAM
pub mod stm6601; // power button manager
pub mod tca8418; // Keypad matrix
pub mod bq27531_g1;
pub mod fusb302b;
pub mod keypad;
pub mod sd_card;
pub mod internal_storage;

use defmt::trace;
use embassy_usb::driver::Driver;

use core::{
    cell::RefCell,
    default::Default,
    option::Option::Some
};

use assign_resources::assign_resources;

use embassy_stm32::{
    bind_interrupts, exti::ExtiInput, fmc::Fmc, gpio::{
        self, Input, Level, Output, OutputType, Pull, Speed
    }, i2c::{self, I2c}, mode, peripherals, spi::{
        self
    }, time::{
        khz, mhz, Hertz
    }, timer::{self, low_level::CountingMode, simple_pwm::{PwmPin, SimplePwm}, GeneralInstance4Channel}, usart::{
        self,
        Uart
    }, Config, Peri, Peripherals
};

use embassy_time::{
    Delay
};

use embassy_sync::{
    blocking_mutex::{
        NoopMutex,
        raw::{
            RawMutex,
            NoopRawMutex
        }
    },
    mutex::Mutex
};

// Provides types and interfaces common to display and graphics operations.
use embedded_graphics::{
    prelude::RgbColor,
    // Provides the necessary functions to draw on the display
    draw_target::DrawTarget,
    // Provides colors from the Rgb565 color space.
    //
    // Rgb565 fits a single pixel into the
    // 8080 parallel 16-bit data-bus.
    pixelcolor::Rgb565,
};

// Provides support for displays that support
// MIPI commands over SPI or 8080 parallal.
use mipidsi::{
    interface::{
        Generic16BitBus,
        ParallelInterface
    },
    models::{
        ILI9341Rgb565
    },
    options::ColorOrder,
    Builder
};


// blocking SDMMC over SPI support.
use embedded_sdmmc::{SdCard};

// Async SDMMC over SPI support.
use sdspi::SdSpi;

use embassy_embedded_hal::{
    SetConfig,
    shared_bus::asynch::{
        self,
        spi::SpiDeviceWithConfig,
    }
};

use crate::hardware::{bq27531_g1::Bq27531, fusb302b::Fusb302b, keypad::Keypad, sd_card::InitError, tca8418::Tca8418};
use crate::hardware::stm6601::Stm6601;

assign_resources! {
    // For debug logging via Black Magic Probe's alternate pins 9/7.
    // see: https://black-magic.org/knowledge/pinouts.html
    uart_debug: UartDebugResources {
        peri: UART5 = UartDebugPeri,
        rx_pin: PD2,
        tx_pin: PC12,
        rx_dma: DMA1_CH0,
        tx_dma: DMA1_CH1,
    }

    /// For TRS MIDI in and out/thru connections.
    uart_midi: UartMIDIResources {
        peri: USART1 = UartMIDIPeri,
        rx_pin: PA10,
        tx_pin: PA9,
        rx_dma: DMA1_CH2,
        tx_dma: DMA1_CH3,
    }

    /// For Bluetooth and WiFi connections.
    uart_rf: UartRFResources {
        peri: UART4 = UartRFPeri,
        rx_pin: PA1,
        tx_pin: PA0,
        rx_dma: DMA1_CH4,
        tx_dma: DMA1_CH5,
    }

    /// NOR Flash chip.
    flash: FlashResources {
        peri: QUADSPI = FlashPeri,
        cs: PD6,
        sck: PF10,
        d0: PD11,
        d1: PD12,
        d2: PE2,
        d3: PD13,
        dma: BDMA_CH0 = FlashDma,
    }

    /// Primary I2S interface, to NAU88C22YG audio codec.
    codec: CodecResources {
        peri: SAI1 = CodecPeri,
        mclk: PG7, // SAI1_MCLK_A
        adcdat: PE3, // SAI1_SD_B
        fs: PE4, // SAI1_FS_A
        sck: PE5, // SAI1_SCK_A,
        dacdat: PE6, //  SAI1_SD_A

        mic_l: PC8, // HIGH = mic-level input
        mic_r: PC7, // HIGH = mic-level input
    }

    /// Wireless I2S interface, nRF5340 Bluetooth and nRF7002 WiFi.
    rf_audio: RFAudioResources {
        peri: SAI4 = RFAudioPeri,
        dacdat: PF6, //SAI4_SD_B
        mclk: PF7, // SAI4_MCLK_B
        sck: PF8, // SAI4_SCK_B
        fs: PF9, // SAI4_FS_B
        adcdat: PC1, //SAT1_SD_A
    }

    /// For USB storage, USB audio interface, and DFU.
    usb: UsbResources {
        peri: USB_OTG_HS = UsbPeri,
        dm: PB14,
        dp: PB15,
        vbus: PB13, // Supplied with 3.3v when in device mode and USB cable connected.
    }

    /// SDRAM and 16-bit parallel display
    fmc: FMCResources {
        peri: FMC = FMCPeri,

        a0: PF0, // A0 / Dispaly D/CX(SCL)
        a1: PF1,
        a2: PF2,
        a3: PF3,
        a4: PF4,
        a5: PF5,
        a6: PF12,
        a7: PF13,
        a8: PF14,
        a9: PF15,
        a10: PG0,
        a11: PG1,
        a12: PG2,

        d0: PD14,
        d1: PD15,
        d2: PD0,
        d3: PD1,
        d4: PE7,
        d5: PE8,
        d6: PE9,
        d7: PE10,
        d8: PE11,
        d9: PE12,
        d10: PE13,
        d11: PE14,
        d12: PE15,
        d13: PD8,
        d14: PD9,
        d15: PD10,

        nbl0: PE0, // DQML
        nbl1: PE1, // DQMH

        ba0: PG4,
        ba1: PG5,

        sdclk: PG8, // SDRAM CLK
        sdncas: PG15, // SDRAM CAS
        sdnwe: PC0, // SDRAM WE
        sdne0: PC4, // NE0 / SDRAM CS
        sdne1: PD7, // NE1 / Display CS
        sdcke0: PC5, // SDRAM CKE
        sdnras: PF11, // SDRAM RAS
        
        noe: PD4, // NOW / Display RD
        nwe: PD5, // NWE / Display WRX(D/CX)
    }

    // Micro-SD card and internal SD-format NAND flash.
    spi_storage: SPIStorageResources {
        peri: SPI1 = SPIStoragePeri,
        clk: PA5, // SCK
        poci: PA6, // MISO
        pico: PA7, // MOSI

        rx_dma: DMA2_CH0,
        tx_dma: DMA2_CH1,

        sd_cs: PA4, // Micro-SD card
        xtsdg_cs: PC13, // SD-protocol NAND flash
    }

    /// Power management resources.
    power: PowerResources {
        int: PA2, // Raises int when power button pressed
        int_exit: EXTI2,
        psel: PA8, // Charger power source selection input. High indicates a USB host source and Low indicates an adapter source.
        pwr_hold: PC6, // Hold HIGH to keep power enabled
        pbout: PG3, // Current power button state
    }

    led: LEDResources {
        dat: PA3,
        tim: TIM5,
        dma: DMA2_CH4,
    }

    usb_pd: USBPDResources {
        int: PG6,
        int_exti: EXTI6,
    }

    fuel_gauge: FuelGaugeResources {
        int: PB1,
        int_exti: EXTI1,
    }

    /// NAU88C22YG Audio Codec, FM SI4703-C19-GMR RX / SI4710-B30-GMR TX
    i2c1: I2C1Resources {
        peri: I2C1 = I2C1Peri,
        scl: PB6,
        sda: PB7,

        tx_dma: DMA1_CH6,
        rx_dma: DMA1_CH7,

        fm_rx_rst: PA12,
        rm_rx_int: PA11,
        fm_tx_rst: PA15,
    }
    
    /// BQ24193 battery charger, BQ27531YZFR-G1 fuel gauge, FUSB302B USB-PD.
    i2c2: I2C2Resources {
        peri: I2C2 = I2C2Peri,
        scl: PB10,
        sda: PB11,

        tx_dma: DMA2_CH2,
        rx_dma: DMA2_CH3,       
    }

    /// FT6206 Capacitive Touch, TCA8418RTWR Keypad, ADS7128IRTER Velocity, DA7280 Haptics, MMA8653FCR1 9DOF
    i2c4: I2C4Resources {
        peri: I2C4 = I2C4Peri,
        scl: PB8,
        sda: PB9,

        tx_dma: BDMA_CH1,
        rx_dma: BDMA_CH2,
    }

    i2c4_interrupts: I2C4Interrupts {
        touch_int: PC3,
        keypad_int: PB0,
        keypad_exti: EXTI0,
        velocity_int: PG13,
        int1_9dof: PB5,
    }

    display: DisplayResources {
        reset: PC11,
        te: PB4,
        backlight_ctrl: PC9, // PWM
        backlight_tim: TIM3,
        touch_rst: PC2,
    }
}

pub mod preamble {
    pub use crate::hardware;
    pub use super::{
        AssignedResources,
        UartDebugResources,
        UartMIDIResources,
        UartRFResources,
        FlashResources,
        CodecResources,
        RFAudioResources,
        UsbResources,
        FMCResources,
        SPIStorageResources,
        PowerResources,
        LEDResources,
        FuelGaugeResources,
        USBPDResources,
        I2C1Resources,
        I2C2Resources,
        I2C4Resources,
        I2C4Interrupts,
        DisplayResources,
    };
}

bind_interrupts!(pub struct Irqs {
    USART1 => usart::InterruptHandler<peripherals::USART1>;
    UART4 => usart::InterruptHandler<peripherals::UART4>;
    UART5 => usart::InterruptHandler<peripherals::UART5>;

    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;

    I2C2_EV => i2c::EventInterruptHandler<peripherals::I2C2>;
    I2C2_ER => i2c::ErrorInterruptHandler<peripherals::I2C2>;

    I2C4_EV => i2c::EventInterruptHandler<peripherals::I2C4>;
    I2C4_ER => i2c::ErrorInterruptHandler<peripherals::I2C4>;

    OTG_HS => embassy_stm32::usb::InterruptHandler<peripherals::USB_OTG_HS>;
});

/// Initializes the Embassy STM32 HAL by configuring
/// the RCC, PLL, voltage, and clock matrix.
pub fn init() -> Peripherals {
    let mut config = Config::default();
    {
        use embassy_stm32::rcc::*;
        
        config.rcc.hsi = Some(HSIPrescaler::DIV1);
        config.rcc.hse = Some(Hse {
            freq: Hertz::mhz(25), // 25Mhz crystal
            mode: HseMode::Oscillator,
        });
        config.rcc.csi = false;
        
        config.rcc.hsi48 = Some(Hsi48Config{
            sync_from_usb: true, // correct?
        });
        
        config.rcc.sys = Sysclk::PLL1_P;
        
        config.rcc.pll1 = Some(Pll{
            source: PllSource::HSE,
            prediv: PllPreDiv::DIV5,
            mul: PllMul::MUL192,
            divp: Some(PllDiv::DIV2),
            divq: Some(PllDiv::DIV7),
            divr: Some(PllDiv::DIV2),
        });
        
        config.rcc.pll2 = Some(Pll{
            source: PllSource::HSE,
            prediv: PllPreDiv::DIV2,
            mul:  PllMul::MUL12,
            divp: Some(PllDiv::DIV2),
            divq: Some(PllDiv::DIV2),
            divr: Some(PllDiv::DIV2),
        });
        
        config.rcc.pll3 = None; /* Some(Pll{
            source: PllSource::HSE,
            prediv: PllPreDiv::DIV32,
            mul:  PllMul::MUL129,
            divp: Some(PllDiv::DIV2),
            divq: Some(PllDiv::DIV2),
            divr: Some(PllDiv::DIV2),
        }); */

        config.rcc.d1c_pre = AHBPrescaler::DIV1; // D1CPRE - 480MHz
        config.rcc.ahb_pre = AHBPrescaler::DIV2; // HPRE 140Mhz (todo: double check)
        config.rcc.apb1_pre = APBPrescaler::DIV2; // D2PPRE1 120Mhz
        config.rcc.apb2_pre = APBPrescaler::DIV2; // D2PPRE2 120Mhz
        config.rcc.apb3_pre = APBPrescaler::DIV2; // D1PPRE 120Mhz
        config.rcc.apb4_pre = APBPrescaler::DIV2; // D3PPRE 120Mhz
        
        config.rcc.timer_prescaler = TimerPrescaler::DefaultX2; // TODO: double check
        
        config.rcc.voltage_scale = VoltageScale::Scale0; // 
        
        config.rcc.ls = LsConfig {
            rtc: RtcClockSource::LSI, // TODO: should be LSE ocillator?
            lsi: true,
            lse: None,/*Some(LseConfig{
                frequency: Hertz(32_768),
                mode: LseMode::Oscillator(LseDrive::Low),
            }),*/
        };

        // config.rcc.mux = mux::ClockMux {
        config.rcc.mux.rtcsel = RtcClockSource::LSI;
        config.rcc.mux.fmcsel = mux::Fmcsel::HCLK3;
        config.rcc.mux.persel = mux::Persel::HSI;
        config.rcc.mux.quadspisel = mux::Fmcsel::HCLK3;
        config.rcc.mux.sdmmcsel = mux::Sdmmcsel::PLL1_Q; // Unused
        config.rcc.mux.fdcansel = mux::Fdcansel::PLL1_Q; // Unused
        config.rcc.mux.sai1sel = mux::Saisel::PLL1_Q;
        config.rcc.mux.sai23sel = mux::Saisel::PLL1_Q; // Unused
        config.rcc.mux.spdifrxsel = mux::Spdifrxsel::PLL1_Q;
        config.rcc.mux.spi123sel = mux::Saisel::PLL1_Q;
        config.rcc.mux.spi45sel = mux::Spi45sel::PCLK2; // Unused
        config.rcc.mux.cecsel = mux::Cecsel::LSI;
        config.rcc.mux.i2c1235sel = mux::I2c1235sel::PCLK1;
        config.rcc.mux.lptim1sel = mux::Lptim1sel::PCLK1; // Unused?
        config.rcc.mux.rngsel = mux::Rngsel::HSI48;
        config.rcc.mux.usart16910sel = mux::Usart16910sel::PCLK2;
        config.rcc.mux.usart234578sel = mux::Usart234578sel::PCLK1;
        config.rcc.mux.usbsel = mux::Usbsel::HSI48;
        config.rcc.mux.adcsel = mux::Adcsel::PLL2_P;
        config.rcc.mux.i2c4sel = mux::I2c4sel::PCLK4;
        config.rcc.mux.lptim2sel = mux::Lptim2sel::PCLK4; // Unused?
        config.rcc.mux.lpuart1sel = mux::Lpuartsel::PCLK4; // TODO: is this correct? unused anyways
        config.rcc.mux.spi6sel = mux::Spi6sel::PCLK4;
    }

    embassy_stm32::init(config)
}

pub fn get_uart_debug_blocking<'a>(r: UartDebugResources) -> Uart<'a, mode::Blocking> {
    let config = usart::Config::default();
    Uart::new_blocking(r.peri, r.rx_pin, r.tx_pin, config).unwrap()
}

pub fn get_uart_debug<'a>(r: UartDebugResources) -> Uart<'a, mode::Async> {
    let config = usart::Config::default();
    Uart::new(r.peri, r.rx_pin, r.tx_pin, Irqs, r.tx_dma, r.rx_dma, config).unwrap()
}

pub fn get_uart_midi_blocking<'a>(r: UartMIDIResources) -> Uart<'a, mode::Blocking> {
    let config = usart::Config::default();
    Uart::new_blocking(r.peri, r.rx_pin, r.tx_pin, config).unwrap()
}

pub fn get_uart_midi<'a>(r: UartMIDIResources) -> Uart<'a, mode::Async> {
    let config = usart::Config::default();
    Uart::new(r.peri, r.rx_pin, r.tx_pin, Irqs, r.tx_dma, r.rx_dma, config).unwrap()
}


pub fn get_uart_rf_blocking<'a>(r: UartRFResources) -> Uart<'a, mode::Blocking> {
    let config = usart::Config::default();
    Uart::new_blocking(r.peri, r.rx_pin, r.tx_pin, config).unwrap()
}

pub fn get_uart_rf<'a>(r: UartRFResources) -> Uart<'a, mode::Async> {
    let config = usart::Config::default();
    Uart::new(r.peri, r.rx_pin, r.tx_pin, Irqs, r.tx_dma, r.rx_dma, config).unwrap()
}

pub fn get_leds<'a>(r: LEDResources) {
    
}

// TODO: need to fix NSS pin
// pub fn get_flash_blocking<'a>(r: FlashResources) -> Ospi<'a, peripherals::QUADSPI1, mode::Blocking> {
//     let config = qspi::Config {
//         memory_size: qspi::enums::MemorySize::_512MiB,
//         address_size: qspi::enums::AddressSize::_16Bit,
//         //prescaler: 160, // AHB 160Mhz => SCK 1MHz
//         ..Default::default()
//     };

//     Qspi::new_blocking_quadspi(
//         r.peri,
//         r.sck,
//         r.d0,
//         r.d1,
//         r.d2,
//         r.d3,
//         r.cs,
//         config)
// }

// TODO: need to fix NSS pin
/*pub fn get_flash<'a>(r: FlashResources) -> Ospi<'a, peripherals::QUADSPI1, mode::Async> {
    let config = qspi::Config {
        memory_size: qspi::enums::MemorySize::_512MiB,
        address_size: qspi::enums::AddressSize::_16Bit,
        //prescaler: 160, // AHB 160Mhz => SCK 1MHz
        ..Default::default()
    };

    Qspi::new_bank1(
        r.peri,
        r.sck,
        r.d0,
        r.d1,
        r.d2,
        r.d3,
        r.cs, // tODO: optional?
        r.dma,
        config)
}*/

/// Returns a handle to the FMC peripheral configured for use with the ISSI is42s16160j-7 SDRAM.
pub fn get_sdram<'a>(r: FMCResources) -> stm32_fmc::Sdram<Fmc<'a, peripherals::FMC>, is42s16160j_7::Is42s16160j> {
    // Configured for the is42s16160j-7
    Fmc::sdram_a13bits_d16bits_4banks_bank1(
        r.peri,
        // A0-A11
        r.a0,
        r.a1,
        r.a2,
        r.a3,
        r.a4,
        r.a5,
        r.a6,
        r.a7,
        r.a8,
        r.a9,
        r.a10,
        r.a11,
        r.a12,
        // BA0-BA1
        r.ba0,
        r.ba1,
        // D0-D15
        r.d0,
        r.d1,
        r.d2,
        r.d3,
        r.d4,
        r.d5,
        r.d6,
        r.d7,
        r.d8,
        r.d9,
        r.d10,
        r.d11,
        r.d12,
        r.d13,
        r.d14,
        r.d15,
        // NBL0 - NBL1
        r.nbl0,
        r.nbl1,
        r.sdcke0,  // SDCKE0
        r.sdclk, // SDCLK
        r.sdncas, // SDNCAS
        r.sdne0, // SDNE1 (!CS)
        r.sdnras, // SDRAS
        r.sdnwe,  // SDNWE 

        // Supplies the timing and mode registry
        // parameters for the specific SDRAM IC.
        is42s16160j_7::Is42s16160j {},
    )
}

/// Returns a handle to 
pub fn get_display<'a>(
    r: FMCResources,
    display: DisplayResources,
    delay: &mut Delay,
) -> mipidsi::Display<
        ParallelInterface<
            Generic16BitBus<
                embassy_stm32::gpio::Output<'a>, // d0
                embassy_stm32::gpio::Output<'a>, // d1
                embassy_stm32::gpio::Output<'a>, // d2
                embassy_stm32::gpio::Output<'a>, // d3
                embassy_stm32::gpio::Output<'a>, // d4
                embassy_stm32::gpio::Output<'a>, // d5
                embassy_stm32::gpio::Output<'a>, // d6
                embassy_stm32::gpio::Output<'a>, // d7
                embassy_stm32::gpio::Output<'a>, // d8
                embassy_stm32::gpio::Output<'a>, // d9
                embassy_stm32::gpio::Output<'a>, // d10
                embassy_stm32::gpio::Output<'a>, // d11
                embassy_stm32::gpio::Output<'a>, // d12
                embassy_stm32::gpio::Output<'a>, // d13
                embassy_stm32::gpio::Output<'a>, // d14
                embassy_stm32::gpio::Output<'a>  // d15
            >,
            embassy_stm32::gpio::Output<'a>, // data/command
            embassy_stm32::gpio::Output<'a>  // write enable
        >,
        ILI9341Rgb565,
        embassy_stm32::gpio::Output<'a>>{
    // Digital reset signal for the display.
    //
    // Pull to VDD to make sure datasheet is satisfied.
    // let rst = display.reset.into_push_pull_output_in_state(gpio::PinState::High);
    let rst = Output::new(display.reset, Level::High, Speed::Low);
    
    // Write enable pin.
    //
    // Pull to VDD to make sure datasheet is satisfied.
    // let wr = r.nwe.into_push_pull_output_in_state(gpio::PinState::High);
    let wr = Output::new(r.nwe, Level::High, Speed::High);

    // Data/Command digital output
    // let dc = r.a0.into_push_pull_output();
    let dc = Output::new(r.a0, Level::High, Speed::High);

    // Configure the data pins as high-speed outputs.
    let d0 = Output::new(r.d0, Level::High, Speed::VeryHigh);
    let d1 = Output::new(r.d1, Level::High, Speed::VeryHigh);
    let d2 = Output::new(r.d2, Level::High, Speed::VeryHigh);
    let d3 = Output::new(r.d3, Level::High, Speed::VeryHigh);
    let d4 = Output::new(r.d4, Level::High, Speed::VeryHigh);
    let d5 = Output::new(r.d5, Level::High, Speed::VeryHigh);
    let d6 = Output::new(r.d6, Level::High, Speed::VeryHigh);
    let d7 = Output::new(r.d7, Level::High, Speed::VeryHigh);
    let d8 = Output::new(r.d8, Level::High, Speed::VeryHigh);
    let d9 = Output::new(r.d9, Level::High, Speed::VeryHigh);
    let d10 = Output::new(r.d10, Level::High, Speed::VeryHigh);
    let d11 = Output::new(r.d11, Level::High, Speed::VeryHigh);
    let d12 = Output::new(r.d12, Level::High, Speed::VeryHigh);
    let d13 = Output::new(r.d13, Level::High, Speed::VeryHigh);
    let d14 = Output::new(r.d14, Level::High, Speed::VeryHigh);
    let d15 = Output::new(r.d15, Level::High, Speed::VeryHigh);

    // Define the parallal bus for display communication.
    let bus = Generic16BitBus::new((
        d0, d1, d2, d3, d4, d5, d6, d7, d8,
        d9, d10, d11, d12, d13, d14, d15,
    ));

    // Create the parallel display interface using the previously
    // created bus and Data/Command and write enable pins.
    let di = ParallelInterface::new(bus, dc, wr);

    // Build the display interface using the parallal bus and interface.
    //
    // RB565 utilizies the entire 16-bit data bus
    // to transfer 1 pixel for every clock cycle.
    let mut display = Builder::new(ILI9341Rgb565, di)
        .display_size(240, 320)
        .reset_pin(rst)
        .color_order(ColorOrder::Bgr)
        .init(delay)
        .unwrap();

    // Clear the dispaly before returning the handle.
    display.clear(Rgb565::BLACK).unwrap();

    return display;
}

/// Returns the analog PWN channel that
/// drives the TPS92360 backlight driver.
// fn get_display_backlight<'a>(display: DisplayResources) -> SimplePwmChannel<'a, peripherals::TIM3> {
//     let backlight_pin = PwmPin::new(display.backlight_ctrl, OutputType::PushPull);

//     // Create the timer interface for the driving the PWM pin.
//     let mut pwm = SimplePwm::new(
//         display.backlight_tim,
//         None,
//         None, None, Some(backlight_pin),
//         khz(10),
//         Default::default());
    
//     let mut ch4 = pwm.ch4();
//     ch4.enable();

//     ch4
// }

/// Returns the STM6601 driver for managing the system power state.
pub fn get_stm6601<'a> (r: PowerResources) -> Stm6601<'a, Output<'a>, Input<'a>>{
    let int = ExtiInput::new(r.int, r.int_exit, Pull::Down);
    let ps_hold = Output::new(r.pwr_hold, Level::High, Speed::Low);
    let pb_state = Input::new(r.pbout, Pull::Down);

    Stm6601::new(int, ps_hold, pb_state)
}

/// Creates the I2C1 interface for communicating with the NAU88C22YG
/// Audio Codec, FM SI4703-C19-GMR RX / SI4710-B30-GMR TX.
pub fn get_i2c1<'a>(r: I2C1Resources) -> embassy_sync::blocking_mutex::Mutex<NoopRawMutex, RefCell<I2c<'a, embassy_stm32::mode::Async, embassy_stm32::i2c::Master>>> {
    let i2c = I2c::new(r.peri, r.scl, r.sda, Irqs, r.tx_dma, r.rx_dma, Default::default());
    NoopMutex::new(RefCell::new(i2c))
}

/// Creates the I2C2 interface for communicating with the BQ24193
/// battery charger, BQ27531YZFR-G1 fuel gauge, FUSB302B USB-PD.
pub fn get_i2c2<'a>(
    r: I2C2Resources
) -> I2c<'a, embassy_stm32::mode::Async, embassy_stm32::i2c::Master> {
    let mut config: i2c::Config = Default::default();

    // The frequency can probably be set faster, but needs to take
    // into account the timing requirements for 100kHz<freq<400kHz
    // on https://www.ti.com/lit/ds/symlink/bq27531-g1.pdf page 20.
    config.frequency = Hertz::khz(100);

    I2c::new(r.peri, r.scl, r.sda, Irqs, r.tx_dma, r.rx_dma, config)
}

/// Gets a handle to the BQ27531 fuel gauge on I2C2.
pub fn get_bq27531_g1_async<'a, 
    INT: gpio::Pin,
    M: RawMutex,
    BUS: embedded_hal_async::i2c::I2c,
    DELAY: embedded_hal::delay::DelayNs
> (
    int: Peri<'a, INT>,
    exti: Peri<'a, INT::ExtiChannel>,
    i2c_bus: &'a Mutex<M, BUS>,
    delay: DELAY,
) -> Bq27531<'a, asynch::i2c::I2cDevice<'a, M, BUS>, DELAY> {
    // Pulled up to compensate for missing pull-up resistor in board design.
    let int = ExtiInput::new(int, exti, Pull::Up);

    // Create the I2C device for the fuel gauge.
    let device = asynch::i2c::I2cDevice::new(i2c_bus);

    Bq27531::new(int, device, delay)
}

/// Gets a handle to the FUSB302B USB-PD controller on I2C2.
pub fn get_fusb302b_async<'a, 
    INT: gpio::Pin,
    M: RawMutex,
    BUS: embedded_hal_async::i2c::I2c,
> (
    int: Peri<'a, INT>,
    exti: Peri<'a, INT::ExtiChannel>,
    i2c_bus: &'a Mutex<M, BUS>,
) -> Fusb302b<'a, asynch::i2c::I2cDevice<'a, M, BUS>> {
    // Pulled up to compensate for missing pull-up resistor in board design.
    let int = ExtiInput::new(int, exti, Pull::Up);

    // Create the I2C device for the fuel gauge.
    let device = asynch::i2c::I2cDevice::new(i2c_bus);

    Fusb302b::new(int, device)
}


/// Creates the I2C4 interface for communicating with the FT6206 Capacitive
/// Touch, TCA8418RTWR Keypad, ADS7128IRTER GPIO Breakout for Velocity
/// Grid, DA7280 Haptics, and MMA8653FCR1 9DOF.
//embassy_sync::blocking_mutex::Mutex<NoopRawMutex, RefCell<I2c<'a, embassy_stm32::mode::Async, embassy_stm32::i2c::Master>>>
pub fn get_i2c4<'a>(r: I2C4Resources) -> embassy_stm32::i2c::I2c<'static, embassy_stm32::mode::Async, embassy_stm32::i2c::Master> {
    I2c::new(r.peri, r.scl, r.sda, Irqs, r.tx_dma, r.rx_dma, Default::default())
}

/// Gets a handle to the TCA8418 keypad decoder on I2C4.
pub fn get_tca8418_async<'a, 
    INT: gpio::Pin,
    M: RawMutex,
    //BUS: SetConfig<Config = i2c::Config> + embedded_hal_async::i2c::I2c,
    BUS: embedded_hal_async::i2c::I2c,
> (
    int: Peri<'a, INT>,
    exti: Peri<'a, INT::ExtiChannel>,
    i2c_bus: &'a Mutex<M, BUS>,
) -> Tca8418<'a, asynch::i2c::I2cDevice<'a, M, BUS>> {
    // Pulled up to compensate for missing pull-up resistor in board design.
    let int = ExtiInput::new(int, exti, Pull::Up);

    // Initialize the I2C config for the device.
    // let config: i2c::Config = Default::default();

    // Create the I2C device for the keypad decoder.
    // let device = asynch::i2c::I2cDeviceWithConfig::new(i2c_bus, config);
    let device = asynch::i2c::I2cDevice::new(i2c_bus);

    Tca8418::new(int, device)
}

/// Create the SPI1 peripheral interface for interacting
/// with the Micro SD card and onboard storage.
pub fn get_spi1<'a> (r: SPIStorageResources) -> (
    embassy_stm32::spi::Spi<'a, embassy_stm32::mode::Async>,
    embassy_stm32::gpio::Output<'a>,
    embassy_stm32::gpio::Output<'a>
) {
    // Initialize the output pins for SPI1 peripheral's chip selects.
    let sd_cs = Output::new(r.sd_cs, Level::High, Speed::Low);
    let xtsdg_cs = Output::new(r.xtsdg_cs, Level::High, Speed::Low);

    // Initialize the SPI1 peripheral.
    let mut spi_config = spi::Config::default();
    trace!("setting spi1 frequency to {}mhz", mhz(1));
    spi_config.frequency = mhz(1);

    (
        spi::Spi::new(r.peri, r.clk, r.pico, r.poci, r.rx_dma, r.tx_dma, spi_config),
        sd_cs,
        xtsdg_cs
    )
}

/// Uses the embedded-sdmmc crate which is a blocking API.
/// 
/// ```
/// let spi_bus = RefCell::new(spi1);
/// ```
pub fn get_sdcard_blocking<'a, DELAY: embedded_hal::delay::DelayNs + core::marker::Copy> (
    spi_bus: &'a RefCell<spi::Spi<'a, mode::Async> >,
    sd_cs: embassy_stm32::gpio::Output<'a>,
    delay: DELAY,
)  -> embedded_sdmmc::SdCard<
        embedded_hal_bus::spi::RefCellDevice<'a,
            embassy_stm32::spi::Spi<'a,
                embassy_stm32::mode::Async
            >,
            embassy_stm32::gpio::Output<'a>,
            DELAY
        >,
        DELAY
> {
    let sdmmc_spi = embedded_hal_bus::spi::RefCellDevice::new(
        spi_bus, sd_cs, delay).unwrap();
    
    SdCard::new(sdmmc_spi, delay)

    // Triggers card initialization and read bytes.
    // println!("SD card size is {} bytes", sdcard.num_bytes()?);
}

/// Uses the embedded-fatfs crate which is async.
pub async fn get_sdcard_async<'a,
    M: RawMutex,
    BUS: SetConfig<Config = spi::Config> + embedded_hal_async::spi::SpiBus,
    DELAY: embedded_hal_async::delay::DelayNs + Clone
> (
    spi_bus: &'a Mutex<M, BUS>,
    cs: gpio::Output<'a>,
    delay: DELAY,
) -> SdSpi<
        embassy_embedded_hal::shared_bus::asynch::spi::SpiDeviceWithConfig<'a,
            M, BUS, embassy_stm32::gpio::Output<'a>>,
        DELAY, aligned::A4
> {
    // Configure the SPI settings for the SD card.
    //
    // Before knowing the SD card's capabilities we need to start with a 400khz clock.
    let mut spi_config: spi::Config = spi::Config::default();
    spi_config.frequency = khz(400);

    // Create the SPI device for the SD card, using the SD card's CS pin.
    let spid = SpiDeviceWithConfig::new(spi_bus, cs, spi_config);
    
    // Initialize the SD-over-SPI wrapper.
    SdSpi::<_, _, aligned::A4>::new(spid, delay)
}

pub async fn get_sdcard_async2<'a,'b, 
    M: RawMutex,
    BUS: SetConfig<Config = spi::Config> + embedded_hal_async::spi::SpiBus,
    DELAY: embedded_hal_async::delay::DelayNs + Clone
> (
    spi_bus: &'a Mutex<M, BUS>,
    cs: gpio::Output<'a>,
    delay: DELAY,
) -> Result<sd_card::SdCard<'a, M, BUS, DELAY>, InitError> {
    // Configure the SPI settings for the SD card.
    //
    // Before knowing the SD card's capabilities we need to start with a 400khz clock.
    let mut spi_config: spi::Config = spi::Config::default();
    spi_config.frequency = khz(400);

    // Create the SPI device for the SD card, using the SD card's CS pin.
    let spid = SpiDeviceWithConfig::new(spi_bus, cs, spi_config);
    
    // Initialize the SD-over-SPI wrapper.
    let spi_sd: SdSpi<SpiDeviceWithConfig<'a, M, BUS, Output<'a>>, DELAY, aligned::A4> = SdSpi::new(spid, delay);
    
    sd_card::SdCard::init(spi_sd).await
}

pub fn get_internal_storage<'a,
    M: RawMutex,
    BUS: SetConfig<Config = spi::Config> + embedded_hal_async::spi::SpiBus,
    DELAY: embedded_hal_async::delay::DelayNs + Clone
> (
    spi_bus: &'a Mutex<M, BUS>,
    cs: gpio::Output<'a>,
    delay: DELAY,
) {
    
}

pub fn get_keypad<'a> (r: LEDResources) -> Keypad<'a, peripherals::TIM5> {
    // Configure the pin to PWM
    let pwm_pin = PwmPin::new(r.dat, OutputType::PushPull);

    // Obtain a PWM handler, configure the Timer and Frequency.
    // The prescaler and ARR are automatically set.
    // Given this system frequency and pwm frequency the max duty cycle will be 300.
    let mut pwm = SimplePwm::new(
        r.tim,
        None,
        None,
        None,
        Some(pwm_pin),
        // PWM_FREQ = 1 / data_transfer_time = 1 / 1.25us = 800kHz
        Hertz::khz(800),
        CountingMode::EdgeAlignedUp,
    );
    

    Keypad::<'a, peripherals::TIM5>::new(pwm, r.dma, timer::Channel::Ch4)
}

/// Constructs the USB high-speed driver.
pub fn get_usb_hs_driver<'a> (r: UsbResources, ep_out_buffer: &'a mut [u8; 256]) -> embassy_stm32::usb::Driver<'a, peripherals::USB_OTG_HS>{
    // Create the USB driver config.
    let mut config = embassy_stm32::usb::Config::default();

    // If your USB device is self-powered (can stay powered on if USB is unplugged), you need
    // to enable vbus_detection to comply with the USB spec. If you enable it, the board
    // has to support it or USB won't work at all. See docs on `vbus_detection` for details.
    config.vbus_detection = true;
    
    // Create the HS USB driver.
    embassy_stm32::usb::Driver::new_fs(r.peri, Irqs, r.dp, r.dm, ep_out_buffer, config)
}


pub async fn get_usb<'a>(r: UsbResources, ep_out_buffer: &'a mut [u8; 256]) {
    let driver = get_usb_hs_driver(r, ep_out_buffer);
}