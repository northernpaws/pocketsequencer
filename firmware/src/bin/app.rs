#![no_std]
#![no_main]

// https://medium.com/@carlmkadie/how-rust-embassy-shine-on-embedded-devices-part-1-9f4911c92007

use core::cell::RefCell;

use defmt::*;

use embassy_executor::Spawner;

use embassy_stm32::{
    i2c::{
        self,
        I2c
    },
    mode::Async,
    spi::{
        self, Spi
    },
    time::mhz
};

use embassy_sync::{
    blocking_mutex::{
        NoopMutex,
        raw::{
            CriticalSectionRawMutex
        }
    },
    mutex::Mutex
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
// static I2C2_BUS: StaticCell<NoopMutex<RefCell<I2c<'static, Async, i2c::Master>>>> = StaticCell::new();
static I2C2_BUS: StaticCell<Mutex<CriticalSectionRawMutex, I2c<'static, Async, i2c::Master>>> = StaticCell::new();

// Input bus.
//
// This communicates with the FT6206 Capacitive Touch sensor, TCA8418RTWR Keypad matrix,
// ADS7128IRTER GPIO Breakout for Velocity Grid, DA7280 Haptics driver, and MMA8653FCR1 9DOF.
// static I2C4_BUS: StaticCell<NoopMutex<RefCell<I2c<'static, Async, i2c::Master>>>> = StaticCell::new();
static I2C4_BUS: StaticCell<Mutex<CriticalSectionRawMutex, I2c<'static, Async, i2c::Master>>> = StaticCell::new();

/// SPI bus for internal and SD card storage.
static SPI_BUS: StaticCell<Mutex<CriticalSectionRawMutex, Spi<'static, Async>>> = StaticCell::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // If it returns, something went wrong.
    let err = inner_main(spawner).await.unwrap_err();
    defmt::panic!("{}", err);
}

// Inner-main allows us us to returns errors via Result instead of relying on panics.
#[expect(clippy::future_not_send, reason = "Safe in single-threaded, bare-metal embedded context")]
#[expect(clippy::items_after_statements, reason = "Keeps related code together")]
async fn inner_main(_spawner: Spawner) -> Result<Never, ()> { // TODO: add error type
    info!("initializing clocks and PLL to 480Mhz");
    let p = hardware::init();

    let r = split_resources!(p);

    // Get a handle to the STM6601 power manager and enable PS_HOLD as early as
    // possible to make sure the STM6601 won't power the system regulator off.
    //
    // This MUST be configured as early as possible to ensure that the
    // PS_HOLD pin is held high by the MCU to keep the power enabled.
    info!("configuring stm6601");
    let mut stm6601 = hardware::get_stm6601(r.power);
    stm6601.power_enable().unwrap(); // TODO: change to result error

    // Initialize the bus for the I2C4 peripheral.
    //
    // This communicates with the FT6206 Capacitive Touch sensor,
    // TCA8418RTWR Keypad matrix, ADS7128IRTER GPIO Breakout for
    // Velocity Grid, DA7280 Haptics driver, and MMA8653FCR1 9DOF.
    info!("initializing input i2c4 bus");
    let i2c4 = hardware::get_i2c4(r.i2c4);
    let i2c4_bus = I2C4_BUS.init(Mutex::new(i2c4));

    // Next, initialize the device wrapper for the keypad decoder IC.
    //
    // We also do this as early as possible so that we can check for keypresses
    // that modify startup, such as to toggle as debug mode, DFU update mode, etc.
    info!("Tnitializing tca8418 keypad decoder...");
    let mut keypad = hardware::get_tca8418_async(
        r.i2c4_interrupts.keypad_int,
        r.i2c4_interrupts.keypad_exti,
        i2c4_bus);
    // Then, immediately configure the registers for the keypad so that
    // we can start processing keypress events as soon as possible.
    info!("Initializing tca8418 registers...");
    keypad.init().await.unwrap(); // TODO: change to result errror

    info!("TCA8418 initialized!");

    // Initialize the bus for the SPI1 peripheral.
    //
    // This communicates with the internal storage and Micro SD card.
    info!("initializing storage spi1 bus");
    let (
        spi1,
        sd_cs,
        mut xtsdg_cs
    ) = hardware::get_spi1(r.spi_storage);

    // Convert the SPI1 peripheral handle into a bus handle that can be consumed by multiple devices.
    let spi_bus = SPI_BUS.init(Mutex::new(spi1));

    // SD cards need to be clocked with a at least 74 cycles
    // on their SPI clock with the CS pin held HIGH.
    //
    // sd_init is a helper function that does this for us.
    info!("Clocking SPI1 74 cyles before initializing SD devices...");
    loop {
        match sd_init(spi_bus.get_mut(), &mut xtsdg_cs).await {
            Ok(_) => break,
            Err(_e) => {
                defmt::warn!("SD card init error!"); // TODO: log the error
                embassy_time::Timer::after_millis(10).await;
            }
        }
    }

    // Initialize the internal and SD card storage next.
    //
    // We also want to initialize storage fairly early in the startup process so that
    // we can check for key files on the device, such as firmware update indicators.

    // Initialize the internal storage.
    //
    // The device internal storage uses an XTSDG IC
    // that acts as a soldered SD card.
    info!("Initializing internal storage...");
    let mut internal_storage = hardware::get_sdcard_async(spi_bus, xtsdg_cs, embassy_time::Delay).await;
    {   
        // Attempt to initialize the connected internal SD storage.
        //
        // If this fails, either there's a problem with the SPI bus or timing 
        info!("Attempting to initialize internal storage...");
        loop {
            // Attempt to initialize the internal storage.
            //
            // Init also supplies CMD0 which is required when
            // using the XTSDG device with a SPI host.
            if internal_storage.init().await.is_ok() {
                info!("Initialization succeeded, increasing internal storage clock to 25Mhz..");

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

            info!("Failed to initialize internal storage, retrying...");
            embassy_time::Delay.delay_ns(5000u32).await;
        }
    }

    // Initialize the Micro SD card.
    info!("Configuring SD card...");
    let mut sdcard = hardware::get_sdcard_async(spi_bus, sd_cs, embassy_time::Delay).await;
    {
        // Attempt to initialize the connected SD card.
        //
        // If this fails, either there's a problem with the
        // SPI bus or timing, there is an incompatible card
        // attached, or there is no card attached. 
        info!("Attempting to initialize SD card...");
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

            info!("Failed to initialize SD card, retrying...");
            embassy_time::Delay.delay_ns(5000u32).await;
        }
    }

    // Next initialize the bus for the I2C2 peripheral that
    // has all the power management peripherals attatched.
    //
    // This communicates with the BQ24193 battery charger,
    // BQ27531YZFR-G1 fuel gauge, FUSB302B USB-PD.
    //
    // Note that the communication with the BQ24193 charger happens
    // THROUGH the BQ27531YZFR-G1 fuel gauge. They're interconnected
    // on their own I2C bus with a special set of registers on the
    // fuel gauge to interact with the charger.
    info!("Initializing power i2c2 bus...");
    let i2c2_bus = I2C2_BUS.init(Mutex::new(hardware::get_i2c2(r.i2c2)));

    // Get a handle to the battery fuel gauge peripheral.
    let fuel_gauge = hardware::get_bq27531_g1_async(
        r.fuel_gauge.int,
        r.fuel_gauge.int_exti,
        i2c2_bus,
        embassy_time::Delay);

    // Initialize the bus for the I2C1 peripheral.
    //
    // This communicates with the NAU88C22YG Audio Codec,
    // FM SI4703-C19-GMR RX / SI4710-B30-GMR TX.
    info!("initializing audio i2c1 bus");
    let _i2c1_bus = I2C1_BUS.init(hardware::get_i2c1(r.i2c1));

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

/// Rust's `!` is unstable.  This is a locally-defined equivalent which is stable.
#[derive(Debug)]
pub enum Never {}
