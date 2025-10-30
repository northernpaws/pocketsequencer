#![no_std]
#![no_main]

// https://medium.com/@carlmkadie/how-rust-embassy-shine-on-embedded-devices-part-1-9f4911c92007

use core::{cell::RefCell, fmt::Debug};

use defmt::*;

use embassy_executor::Spawner;

use embassy_futures::join::join;
use embassy_stm32::{
    gpio::{Level, Output, OutputType, Speed}, i2c::{
        self,
        I2c
    }, mode::Async, peripherals, rcc::clocks, spi::{
        self, Spi
    }, time::{khz, mhz, Hertz}, timer::{self, low_level::CountingMode, simple_pwm::{PwmPin, SimplePwm}}, usart::{Uart, UartRx, UartTx}, usb::{self, Driver}
};

use embassy_sync::{
    blocking_mutex::{
        self, raw::CriticalSectionRawMutex, NoopMutex
    },
    mutex::Mutex
};

use embassy_time::{with_timeout, Duration, Ticker, Timer};
use embassy_usb::{class::{cdc_acm::{CdcAcmClass, State}, midi::MidiClass}, driver::EndpointError, Builder};
use embedded_fatfs::{format_volume, FormatVolumeOptions, FsOptions};
use embedded_hal_async::delay::DelayNs;

use heapless::format;
use rgb_led_pwm_dma_maker::{calc_dma_buffer_length, LedDataComposition, LedDmaBuffer, RgbLedColor, RGB};
use sdio_host::{common_cmd::{select_card, set_block_length}, sd::BlockSize};
use sdspi::{
    sd_init
};
use block_device_adapters::BufStream;
use block_device_adapters::BufStreamError;

use static_cell::StaticCell;

use {
    // defmt_rtt as _,
    panic_probe as _
};

use firmware::{
    hardware::{
        self, Irqs, keypad::{self, Keypad}, preamble::*
    }, split_resources
};

use engine;

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
// static SPI_BUS: StaticCell<blocking_mutex::Mutex<CriticalSectionRawMutex, RefCell<Spi<'static, Async>>>> = StaticCell::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("program start!");
    // If it returns, something went wrong.
    if let Err(err) = inner_main(spawner).await {
        defmt::panic!("{}", err);
    }    
}

static DEBUG_SERIAL_TX: StaticCell<UartTx<'static, Async>> = StaticCell::new();
static DEBUG_SERIAL_RX: StaticCell<UartRx<'static, Async>> = StaticCell::new();

// Inner-main allows us us to returns errors via Result instead of relying on panics.
#[expect(clippy::future_not_send, reason = "Safe in single-threaded, bare-metal embedded context")]
#[expect(clippy::items_after_statements, reason = "Keeps related code together")]
async fn inner_main(spawner: Spawner) -> Result<(), ()> { // TODO: add error type
    info!("initializing clocks and PLL to 480Mhz");
    let p = hardware::init();

    info!("getting handles to hardware peripherals");
    let r = split_resources!(p);

    // Get a handle to the STM6601 power manager and enable PS_HOLD as early as
    // possible to make sure the STM6601 won't power the system regulator off.
    //
    // This MUST be configured as early as possible to ensure that the
    // PS_HOLD pin is held high by the MCU to keep the power enabled.
    //
    // NOTE: PLL errors before this point will cause the power controller to shut
    // off the paniced MCU after 5 seconds, leaving little time for debugging.
    info!("Configuring stm6601...");
    let mut stm6601 = hardware::get_stm6601(r.power);
    stm6601.power_enable().unwrap(); // TODO: change to result error

    info!("Power good confirmed!");

    // Initialize the debug UART connection to the BMP
    // header and configure it for use with defmt.
    //
    // Baud: 115200
    // DataBits8
    // ParityNone
    // Stop1
    let mut debug_uart = hardware::get_uart_debug(r.uart_debug);
    debug_uart.blocking_write(
        b"debug serial ready"
    ).unwrap();
    let (mut debug_tx_raw, mut debug_rx_raw) = debug_uart.split();
    let mut debug_tx = DEBUG_SERIAL_TX.init(debug_tx_raw);
    let mut debug_rx = DEBUG_SERIAL_RX.init(debug_rx_raw);
    defmt_serial::defmt_serial(debug_tx);

    let clocks = clocks(&p.RCC);
    println!("System clock speed: {}", clocks.sys);
    println!("APB1  clock speed: {}", clocks.pclk1);
    println!("APB1 Timer clock speed: {}", clocks.pclk1_tim);
    println!("APB2  clock speed: {}", clocks.pclk2);
    println!("APB2 Timer clock speed: {}", clocks.pclk2_tim);

    // Initialize the bus for the I2C4 peripheral.
    //
    // This communicates with the FT6206 Capacitive Touch sensor,
    // TCA8418RTWR Keypad matrix, ADS7128IRTER GPIO Breakout for
    // Velocity Grid, DA7280 Haptics driver, and MMA8653FCR1 9DOF.
    info!("initializing input I2C4 bus");
    let i2c4 = hardware::get_i2c4(r.i2c4);
    let i2c4_bus = I2C4_BUS.init(Mutex::new(i2c4));

    // Initialize the keypad fairly early so that we can
    // use the keypad LEDs as status indicators.
    info!("Initializing keypad...");
    let mut keypad = hardware::get_keypad(spawner, r.keypad, i2c4_bus).await.unwrap();

    info!("Initializing SDRAM...");
    keypad.set_led(keypad::Led::Trig1, RGB::new(255, 0, 0));
    let mut sdram_ram = hardware::get_sdram(r.fmc);
    keypad.set_led(keypad::Led::Trig1, RGB::new(0, 255, 0));

    let mut led_color: RGB = RGB::new(255, 255, 0);

    // Initialize the bus for the SPI1 peripheral.
    //
    // This communicates with the internal storage and Micro SD card.
    info!("initializing storage SPI1 bus");
    keypad.set_led(keypad::Led::Trig2, RGB::new(255, 0, 0));
    let (
        spi1,
        sd_cs,
        mut xtsdg_cs
    ) = hardware::get_spi1(r.spi_storage);

    // Convert the SPI1 peripheral handle into a bus handle that can be consumed by multiple devices.
    let spi_bus = SPI_BUS.init(Mutex::new(spi1));
    keypad.set_led(keypad::Led::Trig2, RGB::new(0, 255, 0));

    // Initialize the internal and SD card storage next.
    //
    // We also want to initialize storage fairly early in the startup process so that
    // we can check for key files on the device, such as firmware update indicators.

    // SD cards need to be clocked with a at least 74 cycles
    // on their SPI clock with the CS pin held HIGH.
    //
    // sd_init is a helper function that does this for us.
    info!("Clocking SPI1 74 cyles before initializing SD devices...");
    loop {
        match sd_init(spi_bus.get_mut(), &mut xtsdg_cs).await {
            Ok(_) => break,
            Err(_e) => {
                led_color = RGB::new(255, 0, 0);
                defmt::panic!("internal storage init error!"); // TODO: log the error
                embassy_time::Timer::after_millis(10).await;
            }
        }
    }
    
    info!("Constructing SD card device..");
    keypad.set_led(keypad::Led::Trig3, RGB::new(255, 0, 0));
    let mut sd_card = hardware::get_sdcard_async2(spi_bus, sd_cs, embassy_time::Delay).await.unwrap();
    keypad.set_led(keypad::Led::Trig3, RGB::new(0, 255, 0));

    // sd_card.list_filesystem().await.unwrap();
    
    // Initialize the internal storage.
    //
    // The device internal storage uses an XTSDG IC
    // that acts as a soldered SD card.
    info!("Constructing internal storage device...");
    keypad.set_led(keypad::Led::Trig4, RGB::new(255, 0, 0));
    let mut internal_storage = hardware::get_internal_storage(spi_bus, xtsdg_cs, embassy_time::Delay);
    keypad.set_led(keypad::Led::Trig4, RGB::new(0, 255, 0));

/*
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
    let mut fuel_gauge = hardware::get_bq27531_g1_async(
        r.fuel_gauge.int,
        r.fuel_gauge.int_exti,
        i2c2_bus,
        embassy_time::Delay);

    info!("Attempting to read battery charger internal temp...");
    if let Ok(temp) = fuel_gauge.read_internal_temperature().await {
        info!("Battery charger internal temp: {}", temp);
    } else {
        error!("Failed to read battery charger temp!");
    }

    // Get a handle to the FUSB302B device for managing the USB-PD interface.
    let mut fusb302b = hardware::get_fusb302b_async(r.usb_pd.int, r.usb_pd.int_exti, i2c2_bus);

    // Initialize the bus for the I2C1 peripheral.
    //
    // This communicates with the NAU88C22YG Audio Codec,
    // FM SI4703-C19-GMR RX / SI4710-B30-GMR TX.
    info!("initializing audio i2c1 bus");
    let _i2c1_bus = I2C1_BUS.init(hardware::get_i2c1(r.i2c1));
*/
    
    info!("Starting USB device...");
    hardware::usb::start_usb(spawner, r.usb).await;

    info!("Initializing engine..");
    let mut engine_instance = engine::Engine::new(
        keypad,
    );

    engine_instance.start().await;

    /*info!("Starting wireless UART peripheral...");
    let wireless_uart = hardware::get_uart_rf(r.uart_rf);
    let (mut wireless_tx, mut wireless_rx) = wireless_uart.split();
    
    let wireless_read = async {
        let mut buf = [0u8; 16];
        loop {
            unwrap!(wireless_rx.read(&mut buf).await);
            info!("received wireless uart message: {} {=[u8]:a}", buf, buf);
        }
    };

    let wireless_write = async {
        let mut buf = [0u8; 16];
        loop {
            unwrap!(debug_rx.read(&mut buf).await);
            info!("received wireless uart message: {} {=[u8]:a}", buf, buf);
            unwrap!(wireless_tx.write(&mut buf).await);
        }
    };

    join(wireless_read, wireless_write).await;
*/
    Ok(())
}

/// Rust's `!` is unstable.  This is a locally-defined equivalent which is stable.
#[derive(Debug)]
pub enum Never {}

// struct Disconnected {}

// impl From<EndpointError> for Disconnected {
//     fn from(val: EndpointError) -> Self {
//         match val {
//             EndpointError::BufferOverflow => defmt::panic!("Buffer overflow"),
//             EndpointError::Disabled => Disconnected {},
//         }
//     }
// }

// async fn echo<'d, T: usb::Instance + 'd>(class: &mut CdcAcmClass<'d, Driver<'d, T>>) -> Result<(), Disconnected> {
//     let mut buf = [0; 64];
//     loop {
//         let n = class.read_packet(&mut buf).await?;
//         let data = &buf[..n];
//         info!("data: {:x}", data);
//         class.write_packet(data).await?;
//     }
// }

// async fn midi_echo<'d, T: usb::Instance + 'd>(class: &mut MidiClass<'d, usb::Driver<'d, T>>) -> Result<(), Disconnected> {
//     let mut buf = [0; 64];
//     loop {
//         let n = class.read_packet(&mut buf).await?;
//         let data = &buf[..n];
//         info!("data: {:x}", data);
//         class.write_packet(data).await?;
//     }
// }

// #[embassy_executor::task]
// async fn usb_midi(
//     mut midi_class: &'static mut MidiClass<'static, Driver<'static, embassy_stm32::peripherals::USB_OTG_HS>>
// ) {
//     loop {
//         midi_class.wait_connection().await;
//         info!("Connected");
//         let _ = midi_echo(&mut midi_class).await;
//         info!("Disconnected");
//     }
// }


