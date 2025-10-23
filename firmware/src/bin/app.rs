#![no_std]
#![no_main]

use core::cell::RefCell;

use defmt::*;

use embassy_executor::Spawner;

use embassy_stm32::{
    gpio::{Level, Output, Speed},
    i2c::{
        self,
        I2c
    },
    mode::Async,
    peripherals::SPI1,
    spi::{
        self, Spi
    },
    time::mhz
};

use embassy_time::Timer;

use embassy_sync::{
    blocking_mutex::{
        NoopMutex,
        raw::{
            NoopRawMutex,
            CriticalSectionRawMutex
        }
    },
    mutex::Mutex
};

// SDMMC over SPI support.
use embedded_sdmmc::{
    SdCard,
    Mode,
    VolumeIdx
};

use embassy_time::{
    Delay
};

use embedded_hal_async::delay::DelayNs;

use sdspi::{
    sd_init
};

use static_cell::StaticCell;

use {
    defmt_rtt as _,
    panic_probe as _
};

use firmware::{
    split_resources,
    hardware::{
        self,
        preamble::*
    }
};

// Audio I2C bus.
//
// This communicates with the NAU88C22YG Audio Codec, FM SI4703-C19-GMR RX / SI4710-B30-GMR TX.
static I2C1_BUS: StaticCell<NoopMutex<RefCell<I2c<'static, Async, i2c::Master>>>> = StaticCell::new();

// Power management bus.
//
// This communicates with the BQ24193 battery charger, BQ27531YZFR-G1 fuel gauge, and FUSB302B USB-PD manager.
static I2C2_BUS: StaticCell<NoopMutex<RefCell<I2c<'static, Async, i2c::Master>>>> = StaticCell::new();

// Input bus.
//
// This communicates with the FT6206 Capacitive Touch sensor, TCA8418RTWR Keypad matrix,
// ADS7128IRTER GPIO Breakout for Velocity Grid, DA7280 Haptics driver, and MMA8653FCR1 9DOF.
static I2C4_BUS: StaticCell<NoopMutex<RefCell<I2c<'static, Async, i2c::Master>>>> = StaticCell::new();

/// SPI bus for internal and SD card storage.
static SPI_BUS: StaticCell<Mutex<CriticalSectionRawMutex, Spi<'static, Async>>> = StaticCell::new();

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("initializing clocks and PLL to 480Mhz");
    let p = hardware::init();

    let r = split_resources!(p);

    // Get a handle to the STM6601 power manager and enable PS_HOLD as early as
    // possible to make sure the STM6601 won't power the system regulator off.
    info!("configuring stm6601");
    let mut stm6601 = hardware::get_stm6601(r.power);
    stm6601.power_enable().unwrap();

    // Initialize the bus for the I2C1 peripheral.
    //
    // This communicates with the NAU88C22YG Audio Codec,
    // FM SI4703-C19-GMR RX / SI4710-B30-GMR TX.
    info!("initializing audio i2c1 bus");
    let i2c1_bus = I2C1_BUS.init(hardware::get_i2c1(r.i2c1));

    // Initialize the bus for the I2C2 peripheral.
    //
    // This communicates with the BQ24193 battery charger,
    // BQ27531YZFR-G1 fuel gauge, FUSB302B USB-PD.
    info!("initializing power i2c2 bus");
    let i2c2_bus = I2C2_BUS.init(hardware::get_i2c2(r.i2c2));

    // Initialize the bus for the I2C4 peripheral.
    //
    // This communicates with the FT6206 Capacitive Touch sensor,
    // TCA8418RTWR Keypad matrix, ADS7128IRTER GPIO Breakout for
    // Velocity Grid, DA7280 Haptics driver, and MMA8653FCR1 9DOF.
    info!("initializing input i2c4 bus");
    let i2c4_bus = I2C4_BUS.init(hardware::get_i2c4(r.i2c4));

    // Initialize the bus for the SPI1 peripheral.
    //
    // This communicates with the internal storage and Micro SD card.
    info!("initializing storage spi1 bus");
    let (
        spi1,
        mut sd_cs,
        mut xtsdg_cs
    ) = hardware::get_spi1(r.spi_storage);

    // Convert the SPI1 peripheral handle into a bus handle that can be consumed by multiple devices.
    let spi_bus = SPI_BUS.init(Mutex::new(spi1));

    // SD cards need to be clocked with a at least 74 cycles
    // on their SPI clock with the CS pin held HIGH.
    //
    // sd_init is a helper function that does this for us.
    info!("clocking spi1 74 cyles before initializing SD devices");
    loop {
        match sd_init(spi_bus.get_mut(), &mut xtsdg_cs).await {
            Ok(_) => break,
            Err(e) => {
                defmt::warn!("SD card init error!"); // TODO: log the error
                embassy_time::Timer::after_millis(10).await;
            }
        }
    }

    // Initialize the internal storage.
    //
    // The device internal storage uses an XTSDG IC
    // that acts as a soldered SD card.
    info!("initializing sd card spi device");
    let mut internal_storage = hardware::get_sdcard_async(spi_bus, xtsdg_cs, embassy_time::Delay).await;
    {   
        // Attempt to initialize the connected SD card.
        //
        // If this fails, either there's a problem with the
        // SPI bus or timing, there is an incompatible card
        // attached, or there is no card attached. 
        info!("attempting to initialize micro sd card");
        loop {
            // Attempt to initialize the card.
            //
            // Init also supplies CMD0 which is required when
            // using the XTSDG device with a SPI host.
            if internal_storage.init().await.is_ok() {
                info!("Initialization succeeded, increasing clock to 25Mhz..");

                // If the initialization succeeds then we can
                // increase the speed up to the SD max of 25mhz.
                //
                // NOTE: XTSDG supports a high-speed mode of 50MHz.
                // TODO: test the high-speed mode over SPI.
                let mut config = spi::Config::default();
                config.frequency = mhz(25);
                internal_storage.spi().set_config(config);
                info!("Initialization complete!");

                break;
            }

            info!("Failed to init card, retrying...");
            embassy_time::Delay.delay_ns(5000u32).await;
        }
    }

    // Initialize the Micro SD card.
    info!("configuring sd card spi device");
    let mut sdcard = hardware::get_sdcard_async(spi_bus, sd_cs, embassy_time::Delay).await;
    {
        // Attempt to initialize the connected SD card.
        //
        // If this fails, either there's a problem with the
        // SPI bus or timing, there is an incompatible card
        // attached, or there is no card attached. 
        info!("attempting to initialize micro sd card");
        loop {
            // Attempt to initialize the card.
            if sdcard.init().await.is_ok() {
                info!("Initialization succeeded, increasing clock to 25Mhz..");

                // If the initialization succeeds then we can
                // increase the speed up to the SD max of 25mhz.
                //
                // TODO: check if higher speed modes are available.
                let mut config = spi::Config::default();
                config.frequency = mhz(25);
                sdcard.spi().set_config(config);
                info!("Initialization complete!");

                break;
            }

            info!("Failed to init card, retrying...");
            embassy_time::Delay.delay_ns(5000u32).await;
        }
    }

    
    loop {}

    // let mut led = Output::new(p.PB14, Level::High, Speed::Low);

    // loop {
    //     info!("high");
    //     led.set_high();
    //     Timer::after_millis(500).await;

    //     info!("low");
    //     led.set_low();
    //     Timer::after_millis(500).await;
    // }
}
