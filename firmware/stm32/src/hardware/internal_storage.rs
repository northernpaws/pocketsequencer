use defmt::{info, trace};

use block_device_adapters::{StreamSlice, StreamSliceError};
use embassy_embedded_hal::{SetConfig, shared_bus::asynch::spi::SpiDeviceWithConfig};
use embassy_stm32::mode;
use embassy_stm32::spi::Spi;
use embassy_stm32::{gpio::Output, spi, time::mhz};
use embassy_sync::blocking_mutex::raw::RawMutex;
use embedded_fatfs::{FileSystem, FsOptions};
use embedded_fatfs::{FormatVolumeOptions, format_volume};
use embedded_io_async::Read;
use embedded_io_async::Write;

use sdspi::SdSpi;

use block_device_adapters::BufStream;
use block_device_adapters::BufStreamError;

use embedded_hal_async::delay::DelayNs;

#[derive(Debug)]
pub enum InitError {
    SDSPIError(embedded_fatfs::Error<BufStreamError<sdspi::Error>>),
    BufStreamError(BufStreamError<sdspi::Error>),
    StreamSliceError(embedded_fatfs::Error<BufStreamError<sdspi::Error>>),
    SpiSD(sdspi::Error),
}

impl From<BufStreamError<sdspi::Error>> for InitError {
    fn from(value: BufStreamError<sdspi::Error>) -> Self {
        Self::BufStreamError(value)
    }
}

impl From<embedded_fatfs::Error<BufStreamError<sdspi::Error>>> for InitError {
    fn from(value: embedded_fatfs::Error<BufStreamError<sdspi::Error>>) -> Self {
        Self::StreamSliceError(value)
    }
}

impl From<sdspi::Error> for InitError {
    fn from(value: sdspi::Error) -> Self {
        Self::SpiSD(value)
    }
}

/// Driver for the internal storage provided
/// by a XTSDG08GWSIGA flash chip with an SD
/// card "style" interface.
///
/// Note that unlike typical SD cards, this device
/// has a 1024 block size.
pub struct InternalStorage<'a, M: RawMutex> {
    filesystem: FileSystem<
        BufStream<
            SdSpi<
                embassy_embedded_hal::shared_bus::asynch::spi::SpiDeviceWithConfig<
                    'a,
                    M,
                    Spi<'static, mode::Async, spi::mode::Master>,
                    Output<'a>,
                >,
                embassy_time::Delay,
                aligned::A4,
            >,
            1024,
        >,
        embedded_fatfs::NullTimeProvider,
        embedded_fatfs::LossyOemCpConverter,
    >,
}

impl<'a, M: RawMutex> InternalStorage<'a, M> {
    /// Initialize a new internal storage driver.
    pub async fn init(
        mut spi_sd: SdSpi<
            SpiDeviceWithConfig<
                'a,
                M,
                Spi<'static, mode::Async, spi::mode::Master>,
                embassy_stm32::gpio::Output<'a>,
            >,
            embassy_time::Delay,
            aligned::A4,
        >,
    ) -> Result<Self, InitError> {
        // Initialize the internal storage.
        info!("Configuring internal storage...");

        // Attempt to initialize the internal storage.
        //
        // If this fails, either there's a problem with the
        // SPI bus or timing, there is an incompatible card
        // attached, or there is no card attached.
        info!("Attempting to initialize internal storage...");
        loop {
            // Attempt to initialize the storage.
            //
            // SD card init procedure:
            // https://electronics.stackexchange.com/a/238217
            //
            // It seems like CRCs don't work on the internal memory,
            // even after being enabled by the appropriate command.
            if spi_sd.init(false).await.is_ok() {
                info!("Initialization succeeded, increasing clock to 25Mhz..");

                // If the initialization succeeds then we can
                // increase the speed up to the SD max of 25mhz.
                //
                // NOTE: internal storage chip _should_ support 50mhz.
                let mut config = spi::Config::default();
                config.frequency = mhz(25);
                spi_sd.spi().set_config(config);
                info!("Initialization complete!");

                break;
            }

            // TODO: at this point, should be displaying a message indicating the error?
            info!("Failed to initialize internal storage, retrying...");
            embassy_time::Delay.delay_ns(5000u32).await;
        }

        let size = spi_sd.size().await?;
        info!(
            "Internal storage storage size: {}Gb",
            size / 1000 / 1000 / 1000
        );

        let Some(card) = spi_sd.card() else {
            panic!("failed to initialize card");
        };

        if card.ocr.high_capacity() {
            info!("uses sector addressing");
        } else {
            info!("uses byte addressing");
        }

        // NOTE: We don't put a partition table on the internal storage, we just use it as FatFS.

        let mut buf_stream = BufStream::new(spi_sd);

        /*let mut buf_stream = BufStream::new(spi_sd);
        if let Ok(filesystem) = FileSystem::new(buf_stream, FsOptions::new()).await {
            return Ok(Self { filesystem });
        } else {
            info!("Failed to load filesystem, formatting volume...");
            let mut buf_stream = BufStream::new(spi_sd);
            format_volume(
                &mut buf_stream,
                FormatVolumeOptions::default().bytes_per_sector(1024), // IC uses 1024 blocks.
            )
            .await
            .unwrap();

            // Create a filesystem using the stream slice from the partition table.
            info!("Attempting to load volume as FATFS again...");
            let filesystem = FileSystem::new(buf_stream, FsOptions::new()).await?;

            // info!("Attempting to read from filesystem...");
            // let root_dir = filesystem.root_dir();
            // let mut iter = root_dir.iter();
            // while let Some(r) = iter.next().await {
            //     let e = r?;
            //     info!("found file: {}", e.short_file_name_as_bytes());
            // }

            Ok(Self { filesystem })
        }*/

        // Create a filesystem using the stream slice from the partition table.
        info!("Attempting to load volume as FATFS...");
        let filesystem = FileSystem::new(buf_stream, FsOptions::new()).await?;

        {
            let root_dir = filesystem.root_dir();

            info!("Attempting to read from filesystem...");
            if !root_dir.file_exists("test.txt").await.unwrap() {
                info!("Creating test file...");
                let mut file = root_dir.create_file("test.txt").await.unwrap();
                file.write("this is a test".as_bytes()).await.unwrap();
                file.close().await.unwrap();
            }

            info!("Filesystem directory:");
            let mut iter = root_dir.iter();
            while let Some(r) = iter.next().await {
                let e = r?;
                info!(
                    "found file: {=[u8]:x} {:a}",
                    e.short_file_name_as_bytes(),
                    e.short_file_name_as_bytes()
                );
            }
        }

        Ok(Self { filesystem })
    }

    pub async fn list_filesystem(
        &mut self,
    ) -> Result<(), embedded_fatfs::Error<BufStreamError<sdspi::Error>>> {
        info!("Attempting to read from filesystem...");
        let root_dir = self.filesystem.root_dir();
        let mut iter = root_dir.iter();
        while let Some(r) = iter.next().await {
            let e = r?;
            info!("found file: {}", e.short_file_name_as_bytes());
        }

        Ok(())
    }

    //// Deletes and re-creates the volume information, and then erases the card.
    // pub async fn format(
    //     &mut self,
    // ) -> Result<(), embedded_fatfs::Error<BufStreamError<sdspi::Error>>> {
    //     trace!("Formatting SD card...");
    //     let mut format_inner = BufStream::<_, 512>::new(&mut self.spi_sd);
    //     let format_options = FormatVolumeOptions::new()
    //         .volume_id(1)
    //         .volume_label(*b"sd_card    ");

    //     format_volume(&mut format_inner, format_options).await
    // }
}
