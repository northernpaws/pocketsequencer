
use defmt::{info, trace};

use embassy_embedded_hal::{shared_bus::asynch::spi::SpiDeviceWithConfig, SetConfig};
use embassy_stm32::{spi, time::mhz};
use embassy_sync::blocking_mutex::raw::RawMutex;
use embedded_fatfs::{format_volume, FormatVolumeOptions};
use sdio_host::{
    common_cmd::{
        select_card,
        set_block_length
    },
    sd::BlockSize
};

use sdspi::{
    sd_init, SdSpi
};

use block_device_adapters::BufStream;
use block_device_adapters::BufStreamError;

//  use embedded_hal::delay::DelayNs;
use embedded_hal_async::delay::DelayNs;


pub struct SdCard<'a,
    M: RawMutex,
    BUS: SetConfig<Config = spi::Config> + embedded_hal_async::spi::SpiBus,
    DELAY: embedded_hal_async::delay::DelayNs + Clone
> {
    spi_sd: SdSpi<
                SpiDeviceWithConfig<'a, M, BUS, embassy_stm32::gpio::Output<'a>>,
                DELAY,
                aligned::A4>
}

impl<'a,
    M: RawMutex,
    BUS: SetConfig<Config = spi::Config> + embedded_hal_async::spi::SpiBus,
    DELAY: embedded_hal_async::delay::DelayNs + Clone
> SdCard<'a, M, BUS, DELAY> {
    pub fn new (
        spi_sd: SdSpi<
            SpiDeviceWithConfig<'a, M, BUS, embassy_stm32::gpio::Output<'a>>,
            DELAY,
            aligned::A4>
    ) -> Self {
        Self {
            spi_sd
        }
    }

    pub async fn init (&mut self) {
        // Initialize the Micro SD card.
        info!("Configuring SD card...");

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
            if self.spi_sd.init(true).await.is_ok() {
                info!("Initialization succeeded, increasing clock to 25Mhz..");

                // If the initialization succeeds then we can
                // increase the speed up to the SD max of 25mhz.
                //
                // TODO: check if higher speed modes are available.
                let mut config = spi::Config::default();
                config.frequency = mhz(25);
                self.spi_sd.spi().set_config(config);
                info!("Initialization complete!");

                break;
            }

            info!("Failed to initialize SD card, retrying...");
            embassy_time::Delay.delay_ns(5000u32).await;
        }
        
        let size = self.spi_sd.size().await;
        // let size2 = size * (1 << 4 as u64);
        // TODO: why is mul2 required to get accurate
        //  size? something to do with block sizes?
        info!("SD card storage size: {}B", size);
    }

    /// Deletes and re-creates the volume information, and then erases the card.
    pub async fn format(&mut self) -> Result<(), embedded_fatfs::Error<BufStreamError<sdspi::Error>>> {
        trace!("Formatting SD card...");
        let mut format_inner = BufStream::<_, 512>::new(&mut self.spi_sd);
        let format_options = FormatVolumeOptions::new()
            .volume_id(1)
            .volume_label(*b"sd_card    ");

        format_volume(&mut format_inner, format_options).await
    }
}