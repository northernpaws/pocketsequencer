pub mod drivers;

pub mod codec;
pub mod display;
pub mod internal_storage;
pub mod keypad;
pub mod mpu;
pub mod sd_card;
pub mod usb;

use defmt::{info, trace};
use grounded::uninit::GroundedArrayCell;
use proc_bitfield::{Bitfield, SetBits};
use static_cell::{ConstStaticCell, StaticCell};
use stm32_fmc::FmcPeripheral;

use core::{cell::RefCell, default::Default, option::Option::Some};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};

use assign_resources::assign_resources;

use stm32_metapac::{self as pac};

use embassy_stm32::{
    Config, Peri, Peripherals, bind_interrupts,
    can::frame,
    exti::ExtiInput,
    fmc::Fmc,
    gpio::{self, Input, Level, Output, OutputType, Pull, Speed},
    i2c::{self, I2c},
    mode, peripherals,
    rcc::Pll,
    spi::{self},
    time::{Hertz, khz, mhz},
    timer::{
        self,
        low_level::CountingMode,
        simple_pwm::{PwmPin, SimplePwm},
    },
    usart::{self, Uart},
};

use embassy_time::Delay;

use embassy_sync::{
    blocking_mutex::{
        NoopMutex,
        raw::{NoopRawMutex, RawMutex},
    },
    mutex::Mutex,
};

// Provides types and interfaces common to display and graphics operations.
use embedded_graphics::{pixelcolor::Rgb565, prelude::RgbColor};

// Provides support for displays that support
// MIPI commands over SPI or 8080 parallal.
use mipidsi::{
    Builder,
    interface::{Generic16BitBus, ParallelInterface},
    models::ST7789,
    options::ColorOrder,
};

// blocking SDMMC over SPI support.
use embedded_sdmmc::SdCard;

// Async SDMMC over SPI support.
use sdspi::SdSpi;

use embassy_embedded_hal::{
    SetConfig,
    shared_bus::asynch::{self, spi::SpiDeviceWithConfig},
};

use embassy_executor::Spawner;

use crate::hardware::{
    display::Display,
    drivers::{
        bq27531_g1::Bq27531,
        fusb302b::Fusb302b,
        tca8418::{self, Tca8418},
    },
    keypad::Keypad,
    sd_card::InitError,
};
use crate::hardware::{drivers::stm6601::Stm6601, mpu::RegionAttributeSizeRegister};

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

        a0: PF0, // A0 / Display D/CX(SCL)
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

        ba0: PG4, // SDRAM
        ba1: PG5, // SDRAM

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

    keypad: KeypadResources {
        // LED
        dat: PA3,
        tim: TIM5,
        dma: DMA2_CH4,

        // Matric
        int: PB0,
        exti: EXTI0,
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
        velocity_int: PG13,
        int1_9dof: PB5,
    }

    display: DisplayResources {
        reset: PC11,
        te: PB4,
        te_exti: EXTI4,
        backlight_ctrl: PC9, // PWM
        backlight_tim: TIM3,
        touch_rst: PC2,
        dma: DMA2_CH5
    }
}

pub mod preamble {
    pub use super::{
        AssignedResources, CodecResources, DisplayResources, FMCResources, FlashResources,
        FuelGaugeResources, I2C1Resources, I2C2Resources, I2C4Interrupts, I2C4Resources,
        KeypadResources, PowerResources, RFAudioResources, SPIStorageResources, USBPDResources,
        UartDebugResources, UartMIDIResources, UartRFResources, UsbResources,
    };
    pub use crate::hardware;
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

    // config.rcc.pll1 = Some(Pll {
    //     source: embassy_stm32::rcc::PllSource::HSI, // 48
    //     prediv: embassy_stm32::rcc::PllPreDiv::DIV4,
    //     mul: embassy_stm32::rcc::PllMul::MUL10,
    //     divp: None,
    //     divq: Some(embassy_stm32::rcc::PllDiv::DIV12),
    //     divr: None,
    // });
    // config.rcc.mux.fmcsel = embassy_stm32::rcc::mux::Fmcsel::PLL1_Q;

    // default is 64Mhz, div 2 to 32MHz
    // config.rcc.d1c_pre = AHBPrescaler::DIV2;

    // TODO: re-enable 64mhz->480mhz clock
    {
        use embassy_stm32::rcc::*;

        config.rcc.hsi = Some(HSIPrescaler::DIV1);
        config.rcc.hse = Some(Hse {
            freq: Hertz::mhz(25), // 25Mhz crystal
            mode: HseMode::Oscillator,
        });
        config.rcc.csi = false;

        config.rcc.hsi48 = Some(Hsi48Config {
            sync_from_usb: true, // correct?
        });

        config.rcc.sys = Sysclk::PLL1_P;

        config.rcc.pll1 = Some(Pll {
            source: PllSource::HSE,
            prediv: PllPreDiv::DIV5,
            mul: PllMul::MUL192,
            divp: Some(PllDiv::DIV2),
            divq: Some(PllDiv::DIV7),
            divr: Some(PllDiv::DIV2),
        });

        config.rcc.pll2 = Some(Pll {
            source: PllSource::HSE,
            prediv: PllPreDiv::DIV2,
            mul: PllMul::MUL12,
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
            lse: None, /*Some(LseConfig{
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
/// //stm32_fmc::Sdram<Fmc<'a, peripherals::FMC>, is42s16160j_7::Is42s16160j>
pub fn get_sdram<'a>(r: FMCResources) -> &'a mut [u32] {
    let mut core_peri = cortex_m::Peripherals::take().unwrap();

    // taken from stm32h7xx-hal
    core_peri.SCB.enable_icache();
    // See Errata Sheet 2.2.1
    // core_peri.SCB.enable_dcache(&mut core_peri.CPUID);
    core_peri.DWT.enable_cycle_counter();

    // ----------------------------------------------------------
    // Configure MPU for external SDRAM
    // MPU config for SDRAM write-through

    // 256MB
    let sdram_size = 256 * 1024 * 1024;

    {
        let mpu = core_peri.MPU;
        let scb = &mut core_peri.SCB;
        let size = sdram_size;

        // Refer to ARM®v7-M Architecture Reference Manual ARM DDI 0403
        // Version E.b Section B3.5
        const MEMFAULTENA: u32 = 1 << 16;

        unsafe {
            /* Make sure outstanding transfers are done */
            cortex_m::asm::dmb();

            scb.shcsr.modify(|r| r & !MEMFAULTENA);

            /* Disable the MPU and clear the control register*/
            mpu.ctrl.write(0);
        }

        const REGION_NUMBER0: u32 = 0x00;
        const REGION_BASE_ADDRESS: u32 = 0xD000_0000;

        const REGION_FULL_ACCESS: u32 = 0x03;
        const REGION_CACHEABLE: u32 = 0x01;
        const REGION_WRITE_BACK: u32 = 0x01;
        const REGION_ENABLE: u32 = 0x01;

        assert_eq!(
            size & (size - 1),
            0,
            "SDRAM memory region size must be a power of 2"
        );
        assert_eq!(
            size & 0x1F,
            0,
            "SDRAM memory region size must be 32 bytes or more"
        );

        fn log2minus1(sz: u32) -> u32 {
            for i in 5..=31 {
                if sz == (1 << i) {
                    return i - 1;
                }
            }
            defmt::panic!("Unknown SDRAM memory region size!");
        }

        info!("SDRAM Memory Size 0x{:x}", log2minus1(size as u32));

        // Configure region 0
        //
        // Cacheable, outer and inner write-back, no write allocate. So
        // reads are cached, but writes always write all the way to SDRAM
        unsafe {
            mpu.rnr.write(REGION_NUMBER0);
            mpu.rbar.write(REGION_BASE_ADDRESS);
            mpu.rasr.write(
                (REGION_FULL_ACCESS << 24)
                    | (REGION_CACHEABLE << 17)
                    | (REGION_WRITE_BACK << 16)
                    | (log2minus1(size as u32) << 1)
                    | REGION_ENABLE,
            );
        }

        const MPU_ENABLE: u32 = 0x01;
        const MPU_DEFAULT_MMAP_FOR_PRIVILEGED: u32 = 0x04;

        // Enable
        unsafe {
            mpu.ctrl
                .modify(|r| r | MPU_DEFAULT_MMAP_FOR_PRIVILEGED | MPU_ENABLE);

            scb.shcsr.modify(|r| r | MEMFAULTENA);

            // Ensure MPU settings take effect
            cortex_m::asm::dsb();
            cortex_m::asm::isb();
        }
    }

    // Configured for the is42s16160j-7
    let mut sdram = Fmc::sdram_a13bits_d16bits_4banks_bank1(
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
        r.sdcke0, // SDCKE0
        r.sdclk,  // SDCLK
        r.sdncas, // SDNCAS
        r.sdne0,  // SDNE1 (!CS)
        r.sdnras, // SDRAS
        r.sdnwe,  // SDNWE
        // Supplies the timing and mode registry
        // parameters for the specific SDRAM IC.
        drivers::is42s16160j_7::Is42s16160j {},
    );

    let mut delay = Delay;

    let ram_slice: &mut [u32] = unsafe {
        // Initialise controller and SDRAM
        let ram_ptr: *mut u32 = sdram.init(&mut delay) as *mut _;

        // Convert raw pointer to slice
        core::slice::from_raw_parts_mut(ram_ptr, sdram_size / core::mem::size_of::<u32>())
    };

    // // ----------------------------------------------------------
    // // Use memory in SDRAM
    info!("RAM contents before writing: {:x}", ram_slice[..10]);

    ram_slice[0] = 1;
    ram_slice[1] = 2;
    ram_slice[2] = 3;
    ram_slice[3] = 4;

    info!("RAM contents after writing: {:x}", ram_slice[..10]);

    assert_eq!(ram_slice[0], 1);
    assert_eq!(ram_slice[1], 2);
    assert_eq!(ram_slice[2], 3);
    assert_eq!(ram_slice[3], 4);

    info!("Assertions succeeded.");

    ram_slice
}

use ili9341::{DisplaySize240x320, Ili9341, Orientation};

// /// Returns a handle to
// pub fn get_display2<'a>(
//     r: FMCResources,
//     display: DisplayResources,
//     mut delay: Delay,
// ) -> (
//     SimplePwm<'a, embassy_stm32::peripherals::TIM3>,
//     Ili9341<display_interface_parallel_gpio::PGPIO16BitInterface<
//         display_interface_parallel_gpio::Generic16BitBus<
//             embassy_stm32::gpio::Output<'a>,
//             embassy_stm32::gpio::Output<'a>,
//             embassy_stm32::gpio::Output<'a>,
//             embassy_stm32::gpio::Output<'a>,
//             embassy_stm32::gpio::Output<'a>,
//             embassy_stm32::gpio::Output<'a>,
//             embassy_stm32::gpio::Output<'a>,
//             embassy_stm32::gpio::Output<'a>,
//             embassy_stm32::gpio::Output<'a>,
//             embassy_stm32::gpio::Output<'a>,
//             embassy_stm32::gpio::Output<'a>,
//             embassy_stm32::gpio::Output<'a>,
//             embassy_stm32::gpio::Output<'a>,
//             embassy_stm32::gpio::Output<'a>,
//             embassy_stm32::gpio::Output<'a>,
//             embassy_stm32::gpio::Output<'a>>,
//             embassy_stm32::gpio::Output<'a>,
//             embassy_stm32::gpio::Output<'a>>,
//             embassy_stm32::gpio::Output<'a>>)

// {
//      let mut pwm: SimplePwm<'_, peripherals::TIM3> = SimplePwm::new(
//         display.backlight_tim,
//         None,
//         None,
//         None,
//         Some(PwmPin::new(display.backlight_ctrl, OutputType::PushPull)),
//         Hertz::khz(25),
//         CountingMode::EdgeAlignedUp,
//     );

//     // Digital reset signal for the display.
//     //
//     // Pull to VDD to make sure datasheet is satisfied.
//     // let rst = display.reset.into_push_pull_output_in_state(gpio::PinState::High);
//     let rst = Output::new(display.reset, Level::High, Speed::Low);

//     // Write enable pin.
//     //
//     // Pull to VDD to make sure datasheet is satisfied.
//     // let wr = r.nwe.into_push_pull_output_in_state(gpio::PinState::High);
//     let wr = Output::new(r.nwe, Level::High, Speed::High);

//     // Data/Command digital output
//     // let dc = r.a0.into_push_pull_output();
//     let dc = Output::new(r.a0, Level::Low, Speed::High);

//     // Read needs to be held high to be valid.
//     let rd = Output::new(r.noe, Level::High, Speed::High);

//     // Configure the data pins as high-speed outputs.
//     let d0 = Output::new(r.d0, Level::Low, Speed::VeryHigh);
//     let d1 = Output::new(r.d1, Level::Low, Speed::VeryHigh);
//     let d2 = Output::new(r.d2, Level::Low, Speed::VeryHigh);
//     let d3 = Output::new(r.d3, Level::Low, Speed::VeryHigh);
//     let d4 = Output::new(r.d4, Level::Low, Speed::VeryHigh);
//     let d5 = Output::new(r.d5, Level::Low, Speed::VeryHigh);
//     let d6 = Output::new(r.d6, Level::Low, Speed::VeryHigh);
//     let d7 = Output::new(r.d7, Level::Low, Speed::VeryHigh);
//     let d8 = Output::new(r.d8, Level::Low, Speed::VeryHigh);
//     let d9 = Output::new(r.d9, Level::Low, Speed::VeryHigh);
//     let d10 = Output::new(r.d10, Level::Low, Speed::VeryHigh);
//     let d11 = Output::new(r.d11, Level::Low, Speed::VeryHigh);
//     let d12 = Output::new(r.d12, Level::Low, Speed::VeryHigh);
//     let d13 = Output::new(r.d13, Level::Low, Speed::VeryHigh);
//     let d14 = Output::new(r.d14, Level::Low, Speed::VeryHigh);
//     let d15 = Output::new(r.d15, Level::Low, Speed::VeryHigh);

//     let bus = display_interface_parallel_gpio::Generic16BitBus::new((
//         d0, d1, d2, d3, d4, d5, d6, d7, d8,
//         d9, d10, d11, d12, d13, d14, d15,
//     ));

//     let mut interface = display_interface_parallel_gpio::PGPIO16BitInterface::new(bus, dc, wr);

//     let mut display = Ili9341::new(
//             interface,
//             rst,
//             &mut delay,
//             Orientation::Landscape,
//             DisplaySize240x320,
//         ).unwrap();

//     (pwm, display)
// }

// /// Returns a handle to
// pub fn get_display<'a>(
//     r: FMCResources,
//     display: DisplayResources,
//     mut delay: Delay,
// ) -> (
//     SimplePwm<'a, peripherals::TIM3>,
//     Display<'a, Delay>)
// {
//     let mut pwm: SimplePwm<'_, peripherals::TIM3> = SimplePwm::new(
//         display.backlight_tim,
//         None,
//         None,
//         None,
//         Some(PwmPin::new(display.backlight_ctrl, OutputType::PushPull)),
//         Hertz::khz(25),
//         CountingMode::EdgeAlignedUp,
//     );

//     // pwm.channel(timer::Channel::Ch4).enable();
//     // pwm.channel(timer::Channel::Ch4).set_duty_cycle_fully_on();

//     // Digital reset signal for the display.
//     //
//     // Pull to VDD to make sure datasheet is satisfied.
//     // let rst = display.reset.into_push_pull_output_in_state(gpio::PinState::High);
//     let mut rst = Output::new(display.reset, Level::High, Speed::Low);

//     // Write enable pin.
//     //
//     // Pull to VDD to make sure datasheet is satisfied.
//     // let wr = r.nwe.into_push_pull_output_in_state(gpio::PinState::High);
//     let wr = Output::new(r.nwe, Level::High, Speed::High);

//     // Data/Command digital output
//     // let dc = r.a0.into_push_pull_output();
//     let dc = Output::new(r.a0, Level::Low, Speed::High);

//     let rd = Output::new(r.noe, Level::High, Speed::High);

//     let te = Input::new(display.te, Pull::Down);

//     // Configure the data pins as high-speed outputs.
//     let d0 = Output::new(r.d0, Level::Low, Speed::VeryHigh);
//     let d1 = Output::new(r.d1, Level::Low, Speed::VeryHigh);
//     let d2 = Output::new(r.d2, Level::Low, Speed::VeryHigh);
//     let d3 = Output::new(r.d3, Level::Low, Speed::VeryHigh);
//     let d4 = Output::new(r.d4, Level::Low, Speed::VeryHigh);
//     let d5 = Output::new(r.d5, Level::Low, Speed::VeryHigh);
//     let d6 = Output::new(r.d6, Level::Low, Speed::VeryHigh);
//     let d7 = Output::new(r.d7, Level::Low, Speed::VeryHigh);
//     let d8 = Output::new(r.d8, Level::Low, Speed::VeryHigh);
//     let d9 = Output::new(r.d9, Level::Low, Speed::VeryHigh);
//     let d10 = Output::new(r.d10, Level::Low, Speed::VeryHigh);
//     let d11 = Output::new(r.d11, Level::Low, Speed::VeryHigh);
//     let d12 = Output::new(r.d12, Level::Low, Speed::VeryHigh);
//     let d13 = Output::new(r.d13, Level::Low, Speed::VeryHigh);
//     let d14 = Output::new(r.d14, Level::Low, Speed::VeryHigh);
//     let d15 = Output::new(r.d15, Level::Low, Speed::VeryHigh);

//     let fmc = Fmc::new_raw(r.peri);

//     (pwm, Display::new(rst, wr, dc, rd, d0, d1, d2, d3, d4, d5, d6, d7, d8, d9, d10, d11, d12, d13, d14, d15, delay))
// }

/// Returns a handle to
pub fn get_display_st7789<'a>(
    r: FMCResources,
    display: DisplayResources,
    mut delay: Delay,
) -> (
    SimplePwm<'a, peripherals::TIM3>,
    // timer::simple_pwm::SimplePwmChannel<'a, peripherals::TIM3>,
    mipidsi::Display<
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
                embassy_stm32::gpio::Output<'a>, // d15
            >,
            embassy_stm32::gpio::Output<'a>, // data/command
            embassy_stm32::gpio::Output<'a>, // write enable
        >,
        ST7789,
        embassy_stm32::gpio::Output<'a>,
    >,
) {
    let mut pwm: SimplePwm<'_, peripherals::TIM3> = SimplePwm::new(
        display.backlight_tim,
        None,
        None,
        None,
        Some(PwmPin::new(display.backlight_ctrl, OutputType::PushPull)),
        Hertz::khz(25),
        CountingMode::EdgeAlignedUp,
    );

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
    let dc = Output::new(r.a0, Level::Low, Speed::High);

    // Configure the data pins as high-speed outputs.
    let d0 = Output::new(r.d0, Level::Low, Speed::VeryHigh);
    let d1 = Output::new(r.d1, Level::Low, Speed::VeryHigh);
    let d2 = Output::new(r.d2, Level::Low, Speed::VeryHigh);
    let d3 = Output::new(r.d3, Level::Low, Speed::VeryHigh);
    let d4 = Output::new(r.d4, Level::Low, Speed::VeryHigh);
    let d5 = Output::new(r.d5, Level::Low, Speed::VeryHigh);
    let d6 = Output::new(r.d6, Level::Low, Speed::VeryHigh);
    let d7 = Output::new(r.d7, Level::Low, Speed::VeryHigh);
    let d8 = Output::new(r.d8, Level::Low, Speed::VeryHigh);
    let d9 = Output::new(r.d9, Level::Low, Speed::VeryHigh);
    let d10 = Output::new(r.d10, Level::Low, Speed::VeryHigh);
    let d11 = Output::new(r.d11, Level::Low, Speed::VeryHigh);
    let d12 = Output::new(r.d12, Level::Low, Speed::VeryHigh);
    let d13 = Output::new(r.d13, Level::Low, Speed::VeryHigh);
    let d14 = Output::new(r.d14, Level::Low, Speed::VeryHigh);
    let d15 = Output::new(r.d15, Level::Low, Speed::VeryHigh);

    // Define the parallal bus for display communication.
    let bus = Generic16BitBus::new((
        d0, d1, d2, d3, d4, d5, d6, d7, d8, d9, d10, d11, d12, d13, d14, d15,
    ));

    // Create the parallel display interface using the previously
    // created bus and Data/Command and write enable pins.
    let di = ParallelInterface::new(bus, dc, wr);

    // Build the display interface using the parallal bus and inte0rface.
    //
    // RB565 utilizies the entire 16-bit data bus
    // to transfer 1 pixel for every clock cycle.
    let mut display = Builder::new(ST7789, di)
        .display_size(240, 320)
        .reset_pin(rst)
        .color_order(ColorOrder::Bgr)
        .init(&mut delay)
        .unwrap();

    (pwm, display)
}

use embassy_stm32::fmc;

// fn set_as_af<T>(pin: Peri<'a, T>, af_type: AfType) {
//     let pin = unsafe { AnyPin::steal(pin.port()) };
//     let r = pin.block();
//     let n = pin._pin() as usize;

//     r.cr(n / 8).modify(|w| {
//         w.set_mode(n % 8, af_type.mode);
//         // note that we are writing the CNF field, which is exposed as both `cnf_in` and `cnf_out`
//         // in the PAC. the choice of `cnf_in` instead of `cnf_out` in this code is arbitrary and
//         // does not affect the result.
//         w.set_cnf_in(n % 8, vals::CnfIn::from_bits(af_type.cnf));
//     });

//     match af_type.pull {
//         Pull::Up => r.bsrr().write(|w| w.set_bs(n, true)),
//         Pull::Down => r.bsrr().write(|w| w.set_br(n, true)),
//         Pull::None => {}
//     }
// }

// pub async fn configure_fmc_pins<'a, T: fmc::Instance>(
//     // Peri<'a, impl $addr_signal<T>>
//     a0: Peri<'a, impl fmc::A0Pin<T>>, a1: Peri<'a, impl fmc::A1Pin<T>>, a2: Peri<'a, impl fmc::A2Pin<T>>, a3: Peri<'a, impl fmc::A3Pin<T>>,
//     a4: Peri<'a, impl fmc::A4Pin<T>>, a5: Peri<'a, impl fmc::A5Pin<T>>, a6: Peri<'a, impl fmc::A6Pin<T>>, a7: Peri<'a, impl fmc::A7Pin<T>>,
//     a8: Peri<'a, impl fmc::A8Pin<T>>, a9: Peri<'a, impl fmc::A9Pin<T>>, a10: Peri<'a, impl fmc::A10Pin<T>>, a11: Peri<'a, impl fmc::A11Pin<T>>, a12: Peri<'a, impl fmc::A12Pin<T>>,

//     ba0: Peri<'a, impl fmc::BA0Pin<T>>, ba1: Peri<'a, impl fmc::BA1Pin<T>>,

//     d0: Peri<'a, impl fmc::D0Pin<T>>, d1: Peri<'a, impl fmc::D1Pin<T>>, d2: Peri<'a, impl fmc::D2Pin<T>>,
//     d3: Peri<'a, impl fmc::D3Pin<T>>, d4: Peri<'a, impl fmc::D4Pin<T>>, d5: Peri<'a, impl fmc::D5Pin<T>>,
//     d6: Peri<'a, impl fmc::D6Pin<T>>, d7: Peri<'a, impl fmc::D7Pin<T>>, d8: Peri<'a, impl fmc::D8Pin<T>>,
//     d9: Peri<'a, impl fmc::D9Pin<T>>, d10: Peri<'a, impl fmc::D10Pin<T>>, d11: Peri<'a, impl fmc::D11Pin<T>>,
//     d12: Peri<'a, impl fmc::D12Pin<T>>, d13: Peri<'a, impl fmc::D13Pin<T>>, d14: Peri<'a, impl fmc::D14Pin<T>>, d15: Peri<'a, impl fmc::D15Pin<T>>,

//     nbl0: Peri<'a, impl fmc::NBL0Pin<T>>, nbl1: Peri<'a, impl fmc::NBL1Pin<T>>,

//     sdcke: Peri<'a, impl fmc::SDCKE0Pin<T>>, sdclk: Peri<'a, impl fmc::SDCLKPin<T>>,
//     sdncas: Peri<'a, impl fmc::SDNCASPin<T>>, sdne: Peri<'a, impl fmc::SDNE0Pin<T>>,
//     sdnras: Peri<'a, impl fmc::SDNRASPin<T>>, sdnwe: Peri<'a, impl fmc::SDNWEPin<T>>,
// ) {
//     sdnwe.af_num();

// }

/// "Fake" FMC peripheral to avoid the SDRAM package trying to initialize
/// the FMC before the dispaly SRAM has also been configured.
struct SDRamFmcPeripheral {
    source_clock_hz: u32,
}

impl SDRamFmcPeripheral {
    pub fn new(source_clock_hz: u32) -> Self {
        Self { source_clock_hz }
    }
}

unsafe impl FmcPeripheral for SDRamFmcPeripheral {
    // Still need to provide the registers because they're used to configure the SDRAM bank.
    const REGISTERS: *const () = pac::FMC.as_ptr() as *const _;

    fn enable(&mut self) {
        // no-op as we'll already have enabled the FMC controller.
    }

    fn memory_controller_enable(&mut self) {
        // no-op
    }

    fn source_clock_hz(&self) -> u32 {
        self.source_clock_hz
    }
}

static FRAME_BUFFER: ConstStaticCell<[Rgb565; display::FRAMEBUFFER_SIZE]> =
    ConstStaticCell::new([Rgb565::BLACK; display::FRAMEBUFFER_SIZE]);

static SCANLINE: ConstStaticCell<[Rgb565; display::WIDTH]> =
    ConstStaticCell::new([Rgb565::BLACK; display::WIDTH]);

// static mut FRAME_BUFFER: &'static mut [Rgb565; display::FRAMEBUFFER_SIZE] =
//     &mut [Rgb565::BLACK; display::FRAMEBUFFER_SIZE];

/// Configures the SDRAM and display which both use FMC access.
pub async fn get_memory_devices<
    'a,
    DELAY: embedded_hal_async::delay::DelayNs + embedded_hal::delay::DelayNs,
>(
    r: FMCResources,
    display: DisplayResources,
    mut delay: DELAY,
) -> Result<(&'a mut [u32], Display<'a, DELAY>), display::fmc::InitError> {
    let mut core_peri: cortex_m::Peripherals = cortex_m::Peripherals::take().unwrap();
    // taken from stm32h7xx-hal
    core_peri.SCB.enable_icache();
    // See Errata Sheet 2.2.1
    // core_peri.SCB.enable_dcache(&mut core_peri.CPUID);
    core_peri.DWT.enable_cycle_counter();

    const DISPLAY_BANK: display::fmc::FMCRegion = display::fmc::FMCRegion::Bank1;
    const SDRAM_SIZE: usize = 256 * 1024 * 1024;

    {
        let mpu = core_peri.MPU;
        let scb = &mut core_peri.SCB;

        // Refer to ARM®v7-M Architecture Reference Manual ARM DDI 0403
        // Version E.b Section B3.5
        const MEMFAULTENA: u32 = 1 << 16;

        unsafe {
            /* Make sure outstanding transfers are done */
            cortex_m::asm::dmb();

            scb.shcsr.modify(|r| r & !MEMFAULTENA);

            /* Disable the MPU and clear the control register*/
            mpu.ctrl.write(0);
        }

        {
            // Configure MPU region 1 for the SDRAM

            trace!("Configuring MPU region 1 for SDRAM...");

            // 256MB

            const REGION_NUMBER1: u32 = 0x01;
            const REGION_BASE_ADDRESS: u32 = 0xD000_0000;

            const REGION_FULL_ACCESS: u32 = 0x03;
            const REGION_CACHEABLE: u32 = 0x01;
            const REGION_WRITE_BACK: u32 = 0x01;
            const REGION_ENABLE: u32 = 0x01;

            assert_eq!(
                SDRAM_SIZE & (SDRAM_SIZE - 1),
                0,
                "SDRAM memory region size must be a power of 2"
            );
            assert_eq!(
                SDRAM_SIZE & 0x1F,
                0,
                "SDRAM memory region size must be 32 bytes or more"
            );

            fn log2minus1(sz: u32) -> u32 {
                for i in 5..=31 {
                    if sz == (1 << i) {
                        return i - 1;
                    }
                }
                defmt::panic!("Unknown SDRAM memory region size!");
            }

            info!("SDRAM Memory Size 0x{:x}", log2minus1(SDRAM_SIZE as u32));

            // Configure region 1
            //
            // Cacheable, outer and inner write-back, no write allocate. So
            // reads are cached, but writes always write all the way to SDRAM
            unsafe {
                mpu.rnr.write(REGION_NUMBER1);
                mpu.rbar.write(REGION_BASE_ADDRESS);
                mpu.rasr.write(
                    (REGION_FULL_ACCESS << 24)
                        | (REGION_CACHEABLE << 17)
                        | (REGION_WRITE_BACK << 16)
                        | (log2minus1(SDRAM_SIZE as u32) << 1)
                        | REGION_ENABLE,
                );
            }
        }

        {
            // Configure the MPU region 2 for the SRAM.
            trace!("Configuring MPU region 2 for display SRAM...");
            // While the display interface only uses around 4 bytes, we configure
            // the MPU region for the full SRAM bank access window of 64Mbytes.
            //
            // see: RM0433 Rev 8 P.g. 808
            let size = 64 * 1024 * 1024;

            const REGION_NUMBER: u32 = 0x02; // Configure MPU region 2
            const REGION_BASE_ADDRESS: u32 = DISPLAY_BANK.base_address(); // SRAM1 bank base address

            assert_eq!(
                size & (size - 1),
                0,
                "SRAM memory region size must be a power of 2"
            );
            assert_eq!(
                size & 0x1F,
                0,
                "SRAM memory region size must be 32 bytes or more"
            );

            fn log2minus1(sz: u32) -> u32 {
                for i in 5..=31 {
                    if sz == (1 << i) {
                        return i - 1;
                    }
                }
                defmt::panic!("Unknown SRAM memory region size!");
            }

            info!("SRAM Memory Size 0x{:x}", log2minus1(size as u32));

            let rasr = RegionAttributeSizeRegister::from_storage(0) //
                .with_size(log2minus1(size as u32) as u8)
                .with_access_permission(0b011) // full access
                // .with_memory_access_attribute_tex(0b001)
                .with_memory_access_attribute_c(false)
                .with_memory_access_attribute_b(false)
                .with_shareable(true)
                .with_enable(true); //

            // Configure region 0
            //
            // Cacheable, outer and inner write-back, no write allocate. So
            // reads are cached, but writes always write all the way to SDRAM
            unsafe {
                mpu.rnr.write(REGION_NUMBER);
                mpu.rbar.write(REGION_BASE_ADDRESS);
                mpu.rasr.write(rasr.into_storage());
                // mpu.rasr.write(
                //     (REGION_FULL_ACCESS << 24)
                //         | (REGION_CACHEABLE << 17)
                //         | (REGION_WRITE_BACK << 16)
                //         | (log2minus1(size as u32) << 1)
                //         | REGION_ENABLE,
                // );

                //0x03000033
            }
        }
        const MPU_ENABLE: u32 = 0x01;
        const MPU_DEFAULT_MMAP_FOR_PRIVILEGED: u32 = 0x04;

        // Enable
        unsafe {
            mpu.ctrl
                .modify(|r| r | MPU_DEFAULT_MMAP_FOR_PRIVILEGED | MPU_ENABLE);

            scb.shcsr.modify(|r| r | MEMFAULTENA);

            // Ensure MPU settings take effect
            cortex_m::asm::dsb();
            cortex_m::asm::isb();
        }
    }

    // NOTE: Kinda hacky, should be in build.rs script or similar. The metadata isn't really meant to be referenced at runtime.
    // TODO: yikes - this seems to have mate .rodata section grow expenentially..
    // let fmc_base = METADATA.peripherals.iter().find(|p| p.name == "FMC").unwrap().address as u32;

    // Convert the address into an FMC instance.
    //
    // This instance allows us to directly access the registers.
    let fmc_inst = unsafe { pac::fmc::Fmc::from_ptr(0x52004000 as _) };

    // Sets the correct alternate functions for all the FMC pins.
    //
    // Without this, the GPIO matrix won't connect the FMC anywhere...
    let mut fmc = Fmc::configure_sdram_a13bits_d16bits_4banks_bank1_and_parallel(
        r.peri, // A0-A11
        r.a0, r.a1, r.a2, r.a3, r.a4, r.a5, r.a6, r.a7, r.a8, r.a9, r.a10, r.a11, r.a12,
        // BA0-BA1
        r.ba0, r.ba1, // D0-D15
        r.d0, r.d1, r.d2, r.d3, r.d4, r.d5, r.d6, r.d7, r.d8, r.d9, r.d10, r.d11, r.d12, r.d13,
        r.d14, r.d15, // NBL0 - NBL1
        r.nbl0, r.nbl1, r.sdcke0, // SDCKE0
        r.sdclk,  // SDCLK
        r.sdncas, // SDNCAS
        r.sdne0,  // SDNE0 (!CS)
        r.sdne1,  // SDNE1 (!CS)
        r.sdnras, // SDRAS
        r.sdnwe,  // SDNWE
        r.noe,    // NOE
        r.nwe,    // NWE
    );

    // Enable the FMC peripheral for use with the RCC.
    trace!("Initializing Memory Controller...");
    fmc.enable();

    // let fmc = Fmc{ peri: PhantomData };
    let mut sdram = stm32_fmc::Sdram::new_unchecked(
        // NOTE: we
        SDRamFmcPeripheral::new(fmc.source_clock_hz()),
        stm32_fmc::SdramTargetBank::Bank1,
        // Supplies the timing and mode registry
        // parameters for the specific SDRAM IC.
        drivers::is42s16160j_7::Is42s16160j {},
    );

    // Get a Rust slice referencing the memory region mapped to the SDRAM.
    let ram_slice: &mut [u32] = unsafe {
        // Initialise controller and SDRAM
        let ram_ptr: *mut u32 = sdram.init(&mut delay) as *mut _;

        // Convert raw pointer to slice
        core::slice::from_raw_parts_mut(ram_ptr, SDRAM_SIZE / core::mem::size_of::<u32>())
    };

    // TODO: sdram..
    // let sdram = get_sdram(r);
    // Needs custom construction to avoid conflicts?
    //https://docs.rs/stm32-fmc/latest/stm32_fmc/struct.Sdram.html#method.new
    //https://docs.rs/stm32-fmc/latest/stm32_fmc/struct.Sdram.html#method.new_unchecked

    trace!("Creating FMC SRAM interface for LCD...");
    let mut interface = display::fmc::Fmc::new(
        fmc_inst,
        DISPLAY_BANK,
        display::fmc::Config::new_ili9341_parallel_i(display::fmc::MemoryDataWidth::Bits16),
        display::fmc::Timing::new_ili9341_parallel_i(),
    );

    trace!("Initializing display SRAM bank...");
    interface.init()?;

    // Enable the actual memory controller and memory mapping.
    trace!(
        "Enabling Memory Controller... fmc_ker_ck={}hz period={}ns",
        fmc.source_clock_hz(),
        1_000_000_000u32 / fmc.source_clock_hz()
    );
    fmc.memory_controller_enable(); // sets BCR1 FMCEN=1

    // Digital reset signal for the display.
    //
    // Pull to VDD to make sure datasheet is satisfied.
    // let rst = display.reset.into_push_pull_output_in_state(gpio::PinState::High);
    trace!("Initializing Display reset pin...");
    let rst = Output::new(display.reset, Level::High, Speed::Low);

    let te = ExtiInput::new(display.te, display.te_exti, Pull::Up);

    trace!("Initializing Display framebuffer...");
    let frame_buffer = FRAME_BUFFER.take();
    let scanline = SCANLINE.take();

    trace!("Initializing display driver...");
    let mut sram_display = Display::new(
        interface,
        rst,
        te,
        delay,
        display.dma.into(),
        frame_buffer,
        scanline,
    );
    sram_display.init().await;

    Ok((ram_slice, sram_display))
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
pub fn get_stm6601<'a>(r: PowerResources) -> Stm6601<'a, Output<'a>, Input<'a>> {
    let int = ExtiInput::new(r.int, r.int_exit, Pull::Down);
    let ps_hold = Output::new(r.pwr_hold, Level::High, Speed::Low);
    let pb_state = Input::new(r.pbout, Pull::Down);

    Stm6601::new(int, ps_hold, pb_state)
}

/// Creates the I2C1 interface for communicating with the NAU88C22YG
/// Audio Codec, FM SI4703-C19-GMR RX / SI4710-B30-GMR TX.
pub fn get_i2c1<'a>(
    r: I2C1Resources,
) -> embassy_sync::blocking_mutex::Mutex<
    NoopRawMutex,
    RefCell<I2c<'a, embassy_stm32::mode::Async, embassy_stm32::i2c::Master>>,
> {
    let i2c = I2c::new(
        r.peri,
        r.scl,
        r.sda,
        Irqs,
        r.tx_dma,
        r.rx_dma,
        Default::default(),
    );
    NoopMutex::new(RefCell::new(i2c))
}

/// Creates the I2C2 interface for communicating with the BQ24193
/// battery charger, BQ27531YZFR-G1 fuel gauge, FUSB302B USB-PD.
pub fn get_i2c2<'a>(
    r: I2C2Resources,
) -> I2c<'a, embassy_stm32::mode::Async, embassy_stm32::i2c::Master> {
    let mut config: i2c::Config = Default::default();

    // The frequency can probably be set faster, but needs to take
    // into account the timing requirements for 100kHz<freq<400kHz
    // on https://www.ti.com/lit/ds/symlink/bq27531-g1.pdf page 20.
    config.frequency = Hertz::khz(100);

    I2c::new(r.peri, r.scl, r.sda, Irqs, r.tx_dma, r.rx_dma, config)
}

/// Gets a handle to the BQ27531 fuel gauge on I2C2.
pub fn get_bq27531_g1_async<
    'a,
    INT: gpio::Pin,
    M: RawMutex,
    BUS: embedded_hal_async::i2c::I2c,
    DELAY: embedded_hal::delay::DelayNs,
>(
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
pub fn get_fusb302b_async<'a, INT: gpio::Pin, M: RawMutex, BUS: embedded_hal_async::i2c::I2c>(
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
pub fn get_i2c4<'a>(
    r: I2C4Resources,
) -> embassy_stm32::i2c::I2c<'static, embassy_stm32::mode::Async, embassy_stm32::i2c::Master> {
    I2c::new(
        r.peri,
        r.scl,
        r.sda,
        Irqs,
        r.tx_dma,
        r.rx_dma,
        Default::default(),
    )
}

/// Gets a handle to the TCA8418 keypad decoder on I2C4.
pub fn get_tca8418_async<
    'a,
    INT: gpio::Pin,
    M: RawMutex,
    //BUS: SetConfig<Config = i2c::Config> + embedded_hal_async::i2c::I2c,
    BUS: embedded_hal_async::i2c::I2c,
>(
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

    static DRIVER_CH: tca8418::KeyChannel = Channel::new();
    Tca8418::new(int, device, &DRIVER_CH)
}

/// Create the SPI1 peripheral interface for interacting
/// with the Micro SD card and onboard storage.
pub fn get_spi1<'a>(
    r: SPIStorageResources,
) -> (
    embassy_stm32::spi::Spi<'a, embassy_stm32::mode::Async>,
    embassy_stm32::gpio::Output<'a>,
    embassy_stm32::gpio::Output<'a>,
) {
    // Initialize the output pins for SPI1 peripheral's chip selects.
    let sd_cs = Output::new(r.sd_cs, Level::High, Speed::Low);
    let xtsdg_cs = Output::new(r.xtsdg_cs, Level::High, Speed::Low);

    // Initialize the SPI1 peripheral.
    let mut spi_config = spi::Config::default();
    trace!("setting spi1 frequency to {}mhz", mhz(1));
    spi_config.frequency = mhz(1);

    (
        spi::Spi::new(
            r.peri, r.clk, r.pico, r.poci, r.rx_dma, r.tx_dma, spi_config,
        ),
        sd_cs,
        xtsdg_cs,
    )
}

/// Uses the embedded-sdmmc crate which is a blocking API.
///
/// ```
/// let spi_bus = RefCell::new(spi1);
/// ```
pub fn get_sdcard_blocking<'a, DELAY: embedded_hal::delay::DelayNs + core::marker::Copy>(
    spi_bus: &'a RefCell<spi::Spi<'a, mode::Async>>,
    sd_cs: embassy_stm32::gpio::Output<'a>,
    delay: DELAY,
) -> embedded_sdmmc::SdCard<
    embedded_hal_bus::spi::RefCellDevice<
        'a,
        embassy_stm32::spi::Spi<'a, embassy_stm32::mode::Async>,
        embassy_stm32::gpio::Output<'a>,
        DELAY,
    >,
    DELAY,
> {
    let sdmmc_spi = embedded_hal_bus::spi::RefCellDevice::new(spi_bus, sd_cs, delay).unwrap();

    SdCard::new(sdmmc_spi, delay)

    // Triggers card initialization and read bytes.
    // println!("SD card size is {} bytes", sdcard.num_bytes()?);
}

/// Uses the embedded-fatfs crate which is async.
pub async fn get_sdcard_async<
    'a,
    M: RawMutex,
    BUS: SetConfig<Config = spi::Config> + embedded_hal_async::spi::SpiBus,
    DELAY: embedded_hal_async::delay::DelayNs + Clone,
>(
    spi_bus: &'a Mutex<M, BUS>,
    cs: gpio::Output<'a>,
    delay: DELAY,
) -> SdSpi<
    embassy_embedded_hal::shared_bus::asynch::spi::SpiDeviceWithConfig<
        'a,
        M,
        BUS,
        embassy_stm32::gpio::Output<'a>,
    >,
    DELAY,
    aligned::A4,
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

pub async fn get_sdcard_async2<
    'a,
    'b,
    M: RawMutex,
    BUS: SetConfig<Config = spi::Config> + embedded_hal_async::spi::SpiBus,
    DELAY: embedded_hal_async::delay::DelayNs + Clone,
>(
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
    let spi_sd: SdSpi<SpiDeviceWithConfig<'a, M, BUS, Output<'a>>, DELAY, aligned::A4> =
        SdSpi::new(spid, delay);

    sd_card::SdCard::init(spi_sd).await
}

pub fn get_internal_storage<
    'a,
    M: RawMutex,
    BUS: SetConfig<Config = spi::Config> + embedded_hal_async::spi::SpiBus,
    DELAY: embedded_hal_async::delay::DelayNs + Clone,
>(
    spi_bus: &'a Mutex<M, BUS>,
    cs: gpio::Output<'a>,
    delay: DELAY,
) {
}

pub async fn get_keypad<'a>(
    spawner: Spawner,
    r: KeypadResources,
    i2c_bus: &'static Mutex<
        CriticalSectionRawMutex,
        embassy_stm32::i2c::I2c<'static, mode::Async, embassy_stm32::i2c::Master>,
    >,
) -> Result<Keypad<'a>, keypad::InitError> {
    let tca8418 = get_tca8418_async(r.int, r.exti, i2c_bus);

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

    static NOTIFIER: keypad::KeypadNotifier = keypad::notifier();
    static BUTTON_CH: keypad::ButtonChannel = keypad::channel();

    Keypad::new(
        &NOTIFIER,
        &BUTTON_CH,
        tca8418,
        pwm,
        r.dma,
        timer::Channel::Ch4,
        spawner,
    )
    .await
}

/// Constructs the USB high-speed driver.
pub fn get_usb_hs_driver<'a>(
    r: UsbResources,
    ep_out_buffer: &'a mut [u8; 256],
) -> embassy_stm32::usb::Driver<'a, peripherals::USB_OTG_HS> {
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
