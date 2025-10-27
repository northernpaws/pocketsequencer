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
    }, mode::Async, rcc::clocks, spi::{
        self, Spi
    }, time::{khz, mhz, Hertz}, timer::{self, low_level::CountingMode, simple_pwm::{PwmPin, SimplePwm}}, usart::Uart, usb::{self, Driver}
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
        self,
        preamble::*, Irqs
    }, split_resources
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
// static SPI_BUS: StaticCell<blocking_mutex::Mutex<CriticalSectionRawMutex, RefCell<Spi<'static, Async>>>> = StaticCell::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("program start!");
    // If it returns, something went wrong.
    let err = inner_main(spawner).await.unwrap_err();
    defmt::panic!("{}", err);
}

static DEBUG_SERIAL: StaticCell<Uart<'static, Async>> = StaticCell::new();

// Inner-main allows us us to returns errors via Result instead of relying on panics.
#[expect(clippy::future_not_send, reason = "Safe in single-threaded, bare-metal embedded context")]
#[expect(clippy::items_after_statements, reason = "Keeps related code together")]
async fn inner_main(_spawner: Spawner) -> Result<Never, ()> { // TODO: add error type
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
    defmt_serial::defmt_serial(DEBUG_SERIAL.init(debug_uart));

    let clocks = clocks(&p.RCC);
    println!("System clock speed: {}", clocks.sys);
    println!("APB1  clock speed: {}", clocks.pclk1);
    println!("APB1 Timer clock speed: {}", clocks.pclk1_tim);
    println!("APB2  clock speed: {}", clocks.pclk2);
    println!("APB2 Timer clock speed: {}", clocks.pclk2_tim);

    /*
    // Initialize the bus for the I2C4 peripheral.
    //
    // This communicates with the FT6206 Capacitive Touch sensor,
    // TCA8418RTWR Keypad matrix, ADS7128IRTER GPIO Breakout for
    // Velocity Grid, DA7280 Haptics driver, and MMA8653FCR1 9DOF.
    info!("initializing input I2C4 bus");
    let i2c4 = hardware::get_i2c4(r.i2c4);
    let i2c4_bus = I2C4_BUS.init(Mutex::new(i2c4));

    // Next, initialize the device wrapper for the keypad decoder IC.
    //
    // We also do this as early as possible so that we can check for keypresses
    // that modify startup, such as to toggle as debug mode, DFU update mode, etc.
    info!("Initializing TCA8418 keypad decoder...");
    let mut keypad = hardware::get_tca8418_async(
        r.i2c4_interrupts.keypad_int,
        r.i2c4_interrupts.keypad_exti,
        i2c4_bus);
    // Then, immediately configure the registers for the keypad so that
    // we can start processing keypress events as soon as possible.
    info!("Initializing TCA8418 registers...");
    keypad.init().await.unwrap(); // TODO: change to result errror

    info!("TCA8418 initialized!");*/

    let mut led_color: RGB = RGB::new(255, 255, 0);

//     {
//         use embedded_sdmmc::sdcard::{SdCard};
//         use embedded_hal_bus::spi::ExclusiveDevice;

//         struct DummyTimesource();

//         impl embedded_sdmmc::TimeSource for DummyTimesource {
//             fn get_timestamp(&self) -> embedded_sdmmc::Timestamp {
//                 embedded_sdmmc::Timestamp {
//                     year_since_1970: 0,
//                     zero_indexed_month: 0,
//                     zero_indexed_day: 0,
//                     hours: 0,
//                     minutes: 0,
//                     seconds: 0,
//                 }
//             }
//         }

//         info!("initializing storage SPI1 bus");
//         let (
//             spi1,
//             sd_cs,
//             mut xtsdg_cs
//         ) = hardware::get_spi1(r.spi_storage);

//         // Convert the SPI1 peripheral handle into a bus handle that can be consumed by multiple devices.
//         let spi_bus = SPI_BUS.init(blocking_mutex::Mutex::new(RefCell::new(spi1)));
// // SPI clock needs to be running at <= 400kHz during initialization
//         let mut config = spi::Config::default();
//         config.frequency = Hertz(400_000);
//         // let sd_cs = Output::new(r.spi_storage.sd_cs, Level::High, Speed::VeryHigh); // to make sure it's high
//         // let cs = Output::new(r.spi_storage.xtsdg_cs, Level::High, Speed::VeryHigh);
//         let spid = embassy_embedded_hal::shared_bus::blocking::spi::SpiDeviceWithConfig::new(spi_bus, xtsdg_cs, config);

        
//         // let spi = Spi::new_blocking(r.spi_storage.peri, r.spi_storage.clk, r.spi_storage.pico, r.spi_storage.poci, config);
        
//         // let spi_dev = ExclusiveDevice::new_no_delay(spid, cs);
//         let sdcard = embedded_sdmmc::SdCard::new(spid, embassy_time::Delay);
//         let size_bytes = sdcard.num_bytes().unwrap();
//         info!("Card size is {} bytes", size_bytes);

//         // Now that the card is initialized, the SPI clock can go faster
//         let mut config = spi::Config::default();
//         config.frequency = Hertz(16_000_000);
//         sdcard.spi(|dev| dev.set_config(config));

//         // Now let's look for volumes (also known as partitions) on our block device.
//         // To do this we need a Volume Manager. It will take ownership of the block device.
//         let mut volume_mgr = embedded_sdmmc::VolumeManager::new(sdcard, DummyTimesource());

//         // Try and access Volume 0 (i.e. the first partition).
//         // The volume object holds information about the filesystem on that volume.
//         let mut volume0 = volume_mgr.open_volume(embedded_sdmmc::VolumeIdx(0)).unwrap();
//         info!("Volume 0: {:?}", defmt::Debug2Format(&volume0));

//         // Open the root directory (mutably borrows from the volume).
//         let mut root_dir = volume0.open_root_dir().unwrap();

//         // Open a file called "MY_FILE.TXT" in the root directory
//         // This mutably borrows the directory.
//         let mut my_file = root_dir
//             .open_file_in_dir("MY_FILE.TXT", embedded_sdmmc::Mode::ReadOnly)
//             .unwrap();

//         // Print the contents of the file
//         while !my_file.is_eof() {
//             let mut buf = [0u8; 32];
//             if let Ok(n) = my_file.read(&mut buf) {
//                 info!("{:a}", buf[..n]);
//             }
//         }
//     }

    // Initialize the bus for the SPI1 peripheral.
    //
    // This communicates with the internal storage and Micro SD card.
    info!("initializing storage SPI1 bus");
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
                led_color = RGB::new(255, 0, 0);
                defmt::panic!("internal storage init error!"); // TODO: log the error
                embassy_time::Timer::after_millis(10).await;
            }
        }
    }
    
    info!("Constructing SD card device..");
    let mut sd_card = hardware::get_sdcard_async2(spi_bus, sd_cs, embassy_time::Delay).await.unwrap();

    // // TODO: timeout doens't seem to be working?
    // info!("Initializing SD card...");
    // match with_timeout(Duration::from_millis(500), sd_card.init()).await {
    //     Ok(_) => {
    //         info!("SD Card successfully initialized!")
    //     },
    //     Err(err) => {
    //         error!("SD Card initialization failed, no card present?")
    //     },
    // }    

    sd_card.list_filesystem().await.unwrap();
    
    info!("Constructing internal storage device...");
    let mut internal_storage = hardware::get_internal_storage(spi_bus, xtsdg_cs, embassy_time::Delay);

    /*

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
            if internal_storage.init(true).await.is_ok() {
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

                led_color = RGB::new(0, 255, 0);

                break;
            }

            led_color = RGB::new(255, 0, 0);
            info!("Failed to initialize internal storage, retrying...");
            embassy_time::Delay.delay_ns(5000u32).await;
        }
    }

    let size_2 = internal_storage.size().await.unwrap();
    // let size2 = size * (1 << 4 as u64);
    // TODO: why is mul2 required to get accurate
    //  size? something to do with block sizes?
    info!("Internal storage size: {}B", size_2);

    if let Some(card) = internal_storage.card() {
        // info!("Selecting card");
        // internal_storage.cmd(select_card(card.rca)).await.unwrap();

        // info!("Setting storage block length to 1024");
        // internal_storage.cmd(set_block_length(1024)).await.unwrap();
        
        info!("Formatting internal storage...");
        let mut format_inner = BufStream::<_, 512>::new(&mut internal_storage);
        let mut format_options = FormatVolumeOptions::new();
        // let mut format_options = FormatVolumeOptions::new()
        //     .bytes_per_sector(match card.csd.block_length() {
        //         BlockSize::B512 => {
        //             trace!("using 512 bytes_per_sector");
        //             512
        //         }
        //         BlockSize::B1024 => {
        //             trace!("using 1024 bytes_per_sector");
        //             1024
        //         },
        //         _ => defmt::panic!("Unexpected block size!")
        //     });        

        format_volume(&mut format_inner, format_options).await.unwrap();
    } else {
        error!("Failed to get card registers!");
    }


    */

    // info!("Formatting internal storage...");
    // {
    //     // Configure the SPI settings for the SD card.
    //     //
    //     // Before knowing the SD card's capabilities we need to start with a 400khz clock.
    //     let mut spi_config: spi::Config = spi::Config::default();
    //     spi_config.frequency = khz(400);

    //     // Create the SPI device for the SD card, using the SD card's CS pin.
    //     let spid = embassy_embedded_hal::shared_bus::blocking::spi::SpiDeviceWithConfig::new(spi_bus, xtsdg_cs, spi_config);
        
    //     use embedded_sdmmc::{Error, Mode, SdCard, SdCardError, TimeSource, VolumeIdx, VolumeManager};
    //     let internal_storage = SdCard::new(spid, embassy_time::Delay);
    //     into!("Initializing internal storage..");
    //     info!("Card size is {} bytes", internal_storage.num_bytes()?);
    //     let volume_mgr = VolumeManager::new(internal_storage, ts);
    //     let volume0 = volume_mgr.open_volume(VolumeIdx(0))?;
    //     info!("Volume 0: {:?}", volume0);
    //     let root_dir = volume0.open_root_dir()?;
    //     let mut my_file = root_dir.open_file_in_dir("MY_FILE.TXT", Mode::ReadOnly)?;
    //     while !my_file.is_eof() {
    //         let mut buffer = [0u8; 32];
    //         let num_read = my_file.read(&mut buffer)?;
    //         for b in &buffer[0..num_read] {
    //             info!("{}", *b as char);
    //         }
    //     }
    // }

    // let mut format_inner = BufStream::<_, 512>::new(&mut internal_storage);
    // let result = fatfs::format_volume(&mut format_inner, fatfs::FormatVolumeOptions::new());
    // match result {
    //     Ok(_) => info!("formatting finished successfuly!"),
    //     Err(err) => error!("error formatting!")
    // }

   /*info!("Failed to mount, formatting internal storage...");
    let mut format_inner = BufStream::<_, 512>::new(&mut internal_storage);
    let format_options = FormatVolumeOptions::new();
    let result: Result<(), embedded_fatfs::Error<BufStreamError<sdspi::Error>>> = format_volume(&mut format_inner, format_options).await;
    match result {
        Err(err) => {
            match err {
                embedded_fatfs::Error::Io(ioerr) => match ioerr {
                    BufStreamError::Io(ioerr2) => match ioerr2 {
                        sdspi::Error::ChipSelect => error!("Chip select error!"),
                        sdspi::Error::SpiError => error!("SPI Error!"),
                        sdspi::Error::Timeout => error!("Timeout!"),
                        sdspi::Error::UnsupportedCard => error!("Unsupported Card!"),
                        sdspi::Error::Cmd58Error => error!("CMD58 Error!"),
                        sdspi::Error::Cmd59Error => error!("CMD59 Error!"),
                        sdspi::Error::RegisterError(_) => error!("Register error!"),
                        sdspi::Error::CrcMismatch(_, _) => error!("CRC Missmatch!"),
                        sdspi::Error::NotInitialized => error!("Not Initialized !"),
                        sdspi::Error::WriteError => error!("Write Error!"),
                        _ => error!("Unknown sdspi error!"),
                    },
                    _ => error!("Unexpected IO Error!"),
                },
                embedded_fatfs::Error::UnexpectedEof => error!("Unexpected EOF!"),
                embedded_fatfs::Error::WriteZero => error!("Write 0!"),
                embedded_fatfs::Error::InvalidInput => error!("Invalid Input!"),
                embedded_fatfs::Error::NotFound => error!("Not Found!"),
                embedded_fatfs::Error::AlreadyExists => error!("Already Exists!"),
                embedded_fatfs::Error::DirectoryIsNotEmpty => error!("Directiory Not Empty!"),
                embedded_fatfs::Error::CorruptedFileSystem => error!("Corrupted file System!"),
                embedded_fatfs::Error::NotEnoughSpace => error!("Not Enough Space!"),
                embedded_fatfs::Error::InvalidFileNameLength => error!("Invalid File Name Length!"),
                embedded_fatfs::Error::UnsupportedFileNameCharacter => error!("Unsupported File Name Characte!"),
                _ => error!("Unexpected FATFS Error!"),
            }
            defmt::panic!("Failed to format volume! ")
        },
        Ok(_) => {
            info!("Successfuly finished formatting!")
        }
    }*/
    
    // {
    //     info!("Attempting to mount internal storage...");
    //     let mut inner = BufStream::<_, 512>::new(&mut internal_storage);
    //     if let fs = embedded_fatfs::FileSystem::new(inner, FsOptions::new()).await.unwrap() {
    //         {
    //             let root = fs.root_dir();
    //             let mut iter = root.iter();
    //             loop {
    //                 if let Some(Ok(entry)) = iter.next().await {
    //                     let name = entry.short_file_name_as_bytes();
    //                     defmt::info!("Name:{} Length:{}", &name, entry.len());
    //                 } else {
    //                     defmt::info!("end");
    //                     break;
    //                 }
    //             }
    //         }

    //         fs.unmount().await.unwrap();
    //     } else {
    //         info!("Failed to mount, formatting internal storage...");
    //         let mut format_inner = BufStream::<_, 512>::new(&mut internal_storage);
    //         let format_options = FormatVolumeOptions::new()
    //             .volume_id(1)
    //             .volume_label(*b"sd_card    ");
    //         format_volume(&mut format_inner, format_options).await.unwrap();
    //     }
    // }

    // info!("Formatting internal storage...");
    // let mut inner = BufStream::<_, 512>::new(internal_storage);
    // let format_options = FormatVolumeOptions::new()
    //     .volume_id(1)
    //     .volume_label(*b"internal   ");
    // format_volume(&mut inner, format_options).await.unwrap();
    
    // info!("Attempting to mount internal storage...");
    // let fs = embedded_fatfs::FileSystem::new(inner, FsOptions::new()).await.unwrap();
    // {
    //     let root = fs.root_dir();
    //     let mut iter = root.iter();
    //     loop {
    //         if let Some(Ok(entry)) = iter.next().await {
    //             let name = entry.short_file_name_as_bytes();
    //             // let name: String<256> = String::from_utf8(
    //             //     Vec::from_slice(entry.short_file_name_as_bytes()).unwrap(),
    //             // )
    //             // .unwrap();
    //             defmt::info!("Name:{} Length:{}", &name, entry.len());
    //         } else {
    //             defmt::info!("end");
    //             break;
    //         }
    //     }
    // }
    // fs.unmount().await.unwrap();

    // Initialize the Micro SD card.
    /*info!("Configuring SD card...");
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
            //
            // TODO: This hangs indefinetly waiting for a
            //  reply on DMA if no SD card is connected!
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

    let size_2 = sdcard.size().await.unwrap();
    // let size2 = size * (1 << 4 as u64);
    // TODO: why is mul2 required to get accurate
    //  size? something to do with block sizes?
    info!("SD card storage size: {}B", size_2);

     info!("Failed to mount, formatting sd card...");
    let mut format_inner = BufStream::<_, 512>::new(&mut sdcard);
    let format_options = FormatVolumeOptions::new()
        .volume_id(1)
        .volume_label(*b"sd_card    ");
    format_volume(&mut format_inner, format_options).await.unwrap();*/
    // {
    //     info!("Attempting to mount sd card...");
    //     let mut inner = BufStream::<_, 512>::new(&mut sdcard);
    //     if let fs = embedded_fatfs::FileSystem::new(inner, FsOptions::new()).await.unwrap() {
    //         {
    //             let root = fs.root_dir();
    //             let mut iter = root.iter();
    //             loop {
    //                 if let Some(Ok(entry)) = iter.next().await {
    //                     let name = entry.short_file_name_as_bytes();
    //                     // let name: String<256> = String::from_utf8(
    //                     //     Vec::from_slice(entry.short_file_name_as_bytes()).unwrap(),
    //                     // )
    //                     // .unwrap();
    //                     defmt::info!("Name:{} Length:{}", &name, entry.len());
    //                 } else {
    //                     defmt::info!("end");
    //                     break;
    //                 }
    //             }
    //         }

    //         fs.unmount().await.unwrap();
    //     } else {
    //         info!("Failed to mount, formatting sd card...");
    //         let mut format_inner = BufStream::<_, 512>::new(&mut sdcard);
    //         let format_options = FormatVolumeOptions::new()
    //             .volume_id(1)
    //             .volume_label(*b"sd_card    ");
    //         format_volume(&mut format_inner, format_options).await.unwrap();
    //     }
    // }

    //TODO: need to read partition table and open the first FAT partition

    // let inner = BufStream::<_, 512>::new(sdcard);
    // let fs = embedded_fatfs::FileSystem::new(inner, FsOptions::new()).await.unwrap();
    // {
    //     let root = fs.root_dir();
    //     let mut iter = root.iter();
    //     loop {
    //         if let Some(Ok(entry)) = iter.next().await {
    //             let name = entry.short_file_name_as_bytes();
    //             // let name: String<256> = String::from_utf8(
    //             //     Vec::from_slice(entry.short_file_name_as_bytes()).unwrap(),
    //             // )
    //             // .unwrap();
    //             defmt::info!("Name:{} Length:{}", &name, entry.len());
    //         } else {
    //             defmt::info!("end");
    //             break;
    //         }
    //     }
    // }
    // fs.unmount().await.unwrap();



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

    /*let mut ws2812_pwm = SimplePwm::new(
        r.led.tim,
        None,
        None,
        None,
        Some(PwmPin::new(r.led.dat, OutputType::PushPull)),
        khz(800), // data rate of ws2812
        CountingMode::EdgeAlignedUp,
    );

    // construct ws2812 non-return-to-zero (NRZ) code bit by bit
    // ws2812 only need 24 bits for each LED, but we add one bit more to keep PWM output low

    let max_duty = ws2812_pwm.max_duty_cycle();
    let n0 = 8 * max_duty / 25; // ws2812 Bit 0 high level timing
    let n1 = 2 * n0; // ws2812 Bit 1 high level timing

    let turn_off = [
        n0, n0, n0, n0, n0, n0, n0, n0, // Green
        n0, n0, n0, n0, n0, n0, n0, n0, // Red
        n0, n0, n0, n0, n0, n0, n0, n0, // Blue
        0,  // keep PWM output low after a transfer
    ];

    let dim_white = [
        n0, n0, n0, n0, n0, n0, n1, n0, // Green
        n0, n0, n0, n0, n0, n0, n1, n0, // Red
        n0, n0, n0, n0, n0, n0, n1, n0, // Blue
        0,  // keep PWM output low after a transfer
    ];

    let color_list = &[&turn_off, &dim_white];

    let pwm_channel = timer::Channel::Ch1;

    // make sure PWM output keep low on first start
    ws2812_pwm.channel(pwm_channel).set_duty_cycle(0);

    // flip color at 2 Hz
    let mut ticker = Ticker::every(Duration::from_millis(500));

    let mut led_dma = r.led.dma;
    loop {
        for &color in color_list {
            // with &mut, we can easily reuse same DMA channel multiple times
            ws2812_pwm.waveform_up(led_dma.reborrow(), pwm_channel, color).await;
            // ws2812 need at least 50 us low level input to confirm the input data and change it's state
            Timer::after_micros(50).await;
            // wait until ticker tick
            ticker.next().await;
        }
    }*/

    // let ch1_pin = PwmPin::new(r.led.dat, OutputType::PushPull);
    // let mut pwm = SimplePwm::new(
    //     r.led.tim,
    //     None,
    //     None,
    //     None,
    //     Some(ch1_pin),
    //     khz(10),
    //     Default::default()
    // );
    // let mut ch1 = pwm.ch4();
    // ch1.enable();

    // info!("PWM initialized");
    // info!("PWM max duty {}", ch1.max_duty_cycle());

    //     ch1.set_duty_cycle_fraction(1, 2);
    // loop {
    //     ch1.set_duty_cycle_fully_off();
    //     Timer::after_millis(300).await;
    //     ch1.set_duty_cycle_fraction(1, 4);
    //     Timer::after_millis(300).await;
    //     ch1.set_duty_cycle_fraction(1, 2);
    //     Timer::after_millis(300).await;
    //     ch1.set_duty_cycle(ch1.max_duty_cycle() - 1);
    //     Timer::after_millis(300).await;
    // }

    info!("Initializing keypad...");
    let mut keypad = hardware::get_keypad(r.led);
    keypad.init();
    keypad.set_led(0, led_color);
    
    keypad.refresh_leds().await;
    
    // loop {
    //     keypad.refresh_leds().await;
    // }

    
    let mut ep_out_buffer = [0u8; 256];
    let usb_driver = hardware::get_usb_hs_driver(r.usb, &mut ep_out_buffer);

    // Create embassy-usb Config
    let mut usb_config = embassy_usb::Config::new(0xc0de, 0xcafe);
    usb_config.manufacturer = Some("Northernpaws");
    usb_config.product = Some("PocketSynth");
    usb_config.serial_number = Some("12345678");

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut control_buf = [0; 64];

    let mut builder = Builder::new(
        usb_driver,
        usb_config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut [], // no msos descriptors
        &mut control_buf,
    );

    // Create classes on the builder.
    let mut class = MidiClass::new(&mut builder, 1, 1, 64);

    // The `MidiClass` can be split into `Sender` and `Receiver`, to be used in separate tasks.
    // let (sender, receiver) = class.split();

    // Build the builder.
    let mut usb = builder.build();

    // Run the USB device.
    let usb_fut = usb.run();

    // Use the Midi class!
    let midi_fut = async {
        loop {
            class.wait_connection().await;
            info!("Connected");
            let _ = midi_echo(&mut class).await;
            info!("Disconnected");
        }
    };

    // Run everything concurrently.
    // If we had made everything `'static` above instead, we could do this using separate tasks instead.
    join(usb_fut, midi_fut).await;


    // Create the driver, from the HAL.
    
    /*let mut config = embassy_stm32::usb::Config::default();

    // Do not enable vbus_detection. This is a safe default that works in all boards.
    // However, if your USB device is self-powered (can stay powered on if USB is unplugged), you need
    // to enable vbus_detection to comply with the USB spec. If you enable it, the board
    // has to support it or USB won't work at all. See docs on `vbus_detection` for details.
    config.vbus_detection = true;

    let driver = Driver::new_fs(r.usb.peri, Irqs, r.usb.dp, r.usb.dm, &mut ep_out_buffer, config);

    // Create embassy-usb Config
    let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("Northernpaws");
    config.product = Some("PocketSynth");
    config.serial_number = Some("12345678");

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut control_buf = [0; 64];

    let mut state = State::new();

    let mut builder = Builder::new(
        driver,
        config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut [], // no msos descriptors
        &mut control_buf,
    );

    // Create classes on the builder.
    let mut class = CdcAcmClass::new(&mut builder, &mut state, 64);

    // Build the builder.
    let mut usb = builder.build();

    // Run the USB device.
    let usb_fut = usb.run();

    // Do stuff with the class!
    let echo_fut = async {
        loop {
            class.wait_connection().await;
            info!("Connected");
            let _ = echo(&mut class).await;
            info!("Disconnected");
        }
    };

    // Run everything concurrently.
    // If we had made everything `'static` above instead, we could do this using separate tasks instead.
    join(usb_fut, echo_fut).await;
*/
    loop {}

    /*// TIM5 is 32bit timer off APB1 @ 120MHz max
    // 

    // Configure the pin to PWM
    let pwm_pin = PwmPin::new(r.led.dat, OutputType::PushPull);

    // Obtain a PWM handler, configure the Timer and Frequency.
    // The prescaler and ARR are automatically set.
    // Given this system frequency and pwm frequency the max duty cycle will be 300.
    let mut pwm = SimplePwm::new(
        r.led.tim,
        None,
        None,
        None,
        Some(pwm_pin),
        // PWM_FREQ = 1 / data_transfer_time = 1 / 1.25us = 800kHz
        Hertz::khz(800),
        CountingMode::EdgeAlignedUp,
    );
    
    // Enable the channel corresponding to the PWM channel above.
    let mut pwm_channel = pwm.ch4();
    pwm_channel.enable();
    
    /*// Set it off when we're not transmitting LED data.
    //
    // This state is "restored" after a DMA transfer.
    // pwm_channel.set_duty_cycle_fully_off();

    // pwm_channel.set_duty_cycle_fraction(1, 2);
    // Make sure PWM output keep low on first start.
    //
    // This duty cycle is reset to after the DMA waveform has been transmitter.
    pwm_channel.set_duty_cycle(0);

    let mut led_dma = r.led.dma;
    let duty = &[300, 0, 0, 0, 150, 0, 0, 0];
    loop {
        pwm.waveform::<embassy_stm32::timer::Ch4>(led_dma.reborrow(), duty)
            .await;
        Timer::after_millis(600).await;
    }*/

    const RED: RGB = RGB::new(255, 0, 0);
    const GREEN: RGB = RGB::new(0, 255, 0);
    const BLUE: RGB = RGB::new(0, 0, 255);
    const MAGENTA: RGB = RGB::new(255, 0, 255);
    const CYAN: RGB = RGB::new(0, 255, 255);
    const YELLOW: RGB = RGB::new(255, 255, 0);
    const ORANGE: RGB = RGB::new(255, 20, 0);

    const RAINBOW: [RGB; 1] = [RED];
    const LED_COUNT: usize = RAINBOW.len();

    // APB1 = 120MHz
    // doubled for clocks = 240MHz
    // max duty cycle = 240MHz/0.800MHz (pwm freq) = 300 cycles

    // PWM_FREQ = 1 / data_transfer_time = 1 / 1.25us = 800kHz
    // |-------- 1.25us --------|
    // 0%         duty       100%
    // 0 ->  max duty cycle = 300

    // Information needed from datasheet:
    // Data Transfer Time = 1.25us
    //     * Used to calculate pwm frequency, t1h, and t0h values
    // T1H = 0.6us
    // T0H = 0.3us
    // Reset Period = RES >= 80us
    
    //
    // |-------T-------| Symbol Period
    // |-T0H-|_________| 0 symbol
    // |---T1H----|____| 1 Symbol
    // |------- RESET ------|
    /// `t1h` = `T1H / data_transfer_time * max_duty_value`
    /// `t0h` = `T0H / data_transfer_time * max_duty_value`
    /// 
    /// `t1h` = `0.6us / 1.25us * 300` = 144
    /// `t0h` = `0.3us / 1.25us * 300` = 72

    /// NEED TO UPDATE IF SYSTEM CLOCK SPEED CHANGES
    // max_duty_cycle = ?? calculated from PWM
    let max_duty = pwm_channel.max_duty_cycle();
    info!("LED Max Duty Cycle: {}", max_duty);

    // RESET_LENGTH = reset_period / data_transfer_time = 80us / 1.25us = 64
    const RESET_LENGTH: usize = 64; // 64 frames

    // Calculate the dma buffer's length at compile time
    const DMA_BUFFER_LEN: usize = calc_dma_buffer_length(RGB::BIT_COUNT, LED_COUNT, RESET_LENGTH);
    
    // t1h = T1H / data_transfer_time * max_duty_cycle = 0.6us / 1.25us * 300 = 144
    let t1h: u16 = 144; // 144;
    // t0h = T0H / data_transfer_time * max_duty_cycle = 0.3us / 1.25us * 300 = 72
    let t0h: u16 = 72; // 72;

    // Create a DMA buffer for the led strip
    // From datasheet, data structure of 24 bit data is green -> red -> blue, so use LedDataComposition::GRB
    let mut dma_buffer = LedDmaBuffer::<DMA_BUFFER_LEN>::new(t1h, t0h, LedDataComposition::GRB);

    let mut led_dma: embassy_stm32::Peri<'static, embassy_stm32::peripherals::DMA2_CH4> = r.led.dma;
    let mut brightness = 100u8;
    let mut increase = true;

    let mut u32_dma_buffer: [u16; DMA_BUFFER_LEN*2] = [0u16; DMA_BUFFER_LEN*2];
    loop {
        // Set the DMA buffer
        dma_buffer
            .set_dma_buffer_with_brightness(&RAINBOW, None, brightness)
            .unwrap();
        let mut i = 0;
        for d in dma_buffer.get_dma_buffer() {
            u32_dma_buffer[i*2] = *d;
            i += 1;
        }

        // Create a pwm waveform usng the dma buffer
        // pwm.waveform::<embassy_stm32::timer::Ch4>(led_dma.reborrow(), dma_buffer.get_dma_buffer())
        //     .await;
        pwm.waveform::<embassy_stm32::timer::Ch4>(led_dma.reborrow(), &u32_dma_buffer)
            .await;

        // Increase or Decrease brightness accordingly
        if brightness >= 100 {
            increase = false;
        } else if brightness == 0 {
            increase = true;
        }
        if increase {
            brightness += 1;
        } else {
            brightness = brightness.saturating_sub(1);
        }
        if brightness > 100 {
            brightness = 100;
        }
         Timer::after_millis(10).await;
    }*/

    // loop {}

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

struct Disconnected {}

impl From<EndpointError> for Disconnected {
    fn from(val: EndpointError) -> Self {
        match val {
            EndpointError::BufferOverflow => defmt::panic!("Buffer overflow"),
            EndpointError::Disabled => Disconnected {},
        }
    }
}

async fn echo<'d, T: usb::Instance + 'd>(class: &mut CdcAcmClass<'d, Driver<'d, T>>) -> Result<(), Disconnected> {
    let mut buf = [0; 64];
    loop {
        let n = class.read_packet(&mut buf).await?;
        let data = &buf[..n];
        info!("data: {:x}", data);
        class.write_packet(data).await?;
    }
}

async fn midi_echo<'d, T: usb::Instance + 'd>(class: &mut MidiClass<'d, usb::Driver<'d, T>>) -> Result<(), Disconnected> {
    let mut buf = [0; 64];
    loop {
        let n = class.read_packet(&mut buf).await?;
        let data = &buf[..n];
        info!("data: {:x}", data);
        class.write_packet(data).await?;
    }
}
