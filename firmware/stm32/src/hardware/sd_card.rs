
use defmt::{info, trace, error};

use embassy_embedded_hal::{shared_bus::asynch::spi::SpiDeviceWithConfig, SetConfig};
use embassy_stm32::{gpio::Output, spi, time::mhz};
use embassy_sync::blocking_mutex::raw::RawMutex;
use embedded_fatfs::{format_volume, DateTime, Error, FileSystem, FormatVolumeOptions, FsOptions, ReadWriteSeek};
use embedded_io_async::Read;

use block_device_adapters::{StreamSlice, StreamSliceError};

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

use mbr_nostd::{MasterBootRecord, PartitionTable};

#[derive(Debug)]
pub enum InitError {
    SDSPIError(embedded_fatfs::Error<BufStreamError<sdspi::Error>>),
    BufStreamError(BufStreamError<sdspi::Error>),
    StreamSliceError(embedded_fatfs::Error<StreamSliceError<BufStreamError<sdspi::Error>>>),
    SpiSD(sdspi::Error)
}

impl From<BufStreamError<sdspi::Error>> for InitError {
    fn from(value: BufStreamError<sdspi::Error>) -> Self {
        Self::BufStreamError(value)
    }
}

impl From<embedded_fatfs::Error<StreamSliceError<BufStreamError<sdspi::Error>>>> for InitError {
    fn from(value: embedded_fatfs::Error<StreamSliceError<BufStreamError<sdspi::Error>>>) -> Self {
        Self::StreamSliceError(value)
    }
}

impl From<sdspi::Error> for InitError {
    fn from(value: sdspi::Error) -> Self {
        Self::SpiSD(value)
    }
}

pub struct SdCard<'a,
    M: RawMutex,
    BUS: SetConfig<Config = spi::Config> + embedded_hal_async::spi::SpiBus,
    DELAY: embedded_hal_async::delay::DelayNs + Clone
> {
    // spi_sd: SdSpi<
    //             SpiDeviceWithConfig<'a, M, BUS, embassy_stm32::gpio::Output<'a>>,
    //             DELAY,
    //             aligned::A4>,
    // buf_stream: Option<&'b mut BufStream::<&'a mut SdSpi<SpiDeviceWithConfig<'a, M, BUS, Output<'a>>, DELAY, aligned::A4>, 512>>,
    filesystem: FileSystem<
                        StreamSlice<
                            BufStream<
                                SdSpi<SpiDeviceWithConfig<'a, M, BUS, Output<'a>>, DELAY, aligned::A4
                            >, 512>
                        >, embedded_fatfs::NullTimeProvider, embedded_fatfs::LossyOemCpConverter>
                    
}

impl<'a,
    M: RawMutex,
    BUS: SetConfig<Config = spi::Config> + embedded_hal_async::spi::SpiBus,
    DELAY: embedded_hal_async::delay::DelayNs + Clone
> SdCard<'a, M, BUS, DELAY> {
    pub async fn init (
        mut spi_sd: SdSpi<
            SpiDeviceWithConfig<'a, M, BUS, embassy_stm32::gpio::Output<'a>>,
            DELAY,
            aligned::A4>,
    ) -> Result<Self, InitError> {
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
            if spi_sd.init(true).await.is_ok() {
                info!("Initialization succeeded, increasing clock to 25Mhz..");

                // If the initialization succeeds then we can
                // increase the speed up to the SD max of 25mhz.
                //
                // TODO: check if higher speed modes are available.
                let mut config = spi::Config::default();
                config.frequency = mhz(25);
                spi_sd.spi().set_config(config);
                info!("Initialization complete!");

                break;
            }

            // TODO: at this point, should be displaying a message indicating the lack of SD card?
            info!("Failed to initialize SD card, retrying...");
            embassy_time::Delay.delay_ns(5000u32).await;
        }
        
        let size = spi_sd.size().await?;
        info!("SD card storage size: {}B", size);

        let mut buf_stream = BufStream::new(spi_sd);

        trace!("Attempting to read MBR...");
        let mut buf = [0; 512];
        buf_stream.read(&mut buf).await?;
        let mbr = MasterBootRecord::from_bytes(&buf).unwrap();
        let mut index = 0;
        for partition in mbr.partition_table_entries() {
            trace!("MBR Partition {}:", index);
            trace!("MBR Partition Sectors {}", partition.sector_count);
            trace!("MBR Partition Logical Block Address {}", partition.logical_block_address);
            index += 1;
        }
        
        // Read the first partition table entry from the MBR.
        let partition = mbr.partition_table_entries()[0];
        let start_offset = partition.logical_block_address as u64 * 512;
        let end_offset = start_offset + partition.sector_count as u64 * 512;

        // Make a wrapper around the BufStream that limits the start
        // and end of reads/writes to within the partition.
        let inner = StreamSlice::new(buf_stream, start_offset, end_offset)
            .await
            .unwrap();

        // Create a filesystem using the stream slice from the partition table.
        info!("Loading partition 0 as FATFS...");
        let filesystem = FileSystem::new(inner, FsOptions::new()).await?;
        
        // info!("Attempting to read from filesystem...");
        // let root_dir = filesystem.root_dir();
        // let mut iter = root_dir.iter();
        // while let Some(r) = iter.next().await {
        //     let e = r?;
        //     info!("found file: {}", e.short_file_name_as_bytes());
        // }

        Ok(Self {
            filesystem
         })
    }

    pub async fn list_filesystem (&mut self) -> Result<(), embedded_fatfs::Error<StreamSliceError<BufStreamError<sdspi::Error>>>>{
        info!("Attempting to read from filesystem...");
        let root_dir = self.filesystem.root_dir();
        let mut iter = root_dir.iter();
        while let Some(r) = iter.next().await {
            let e = r?;
            info!("found file: {}", e.short_file_name_as_bytes());
        }

        Ok(())
    }

    // TODO: re-do with MBR
    // /// Deletes and re-creates the volume information, and then erases the card.
    // pub async fn format(&mut self) -> Result<(), embedded_fatfs::Error<BufStreamError<sdspi::Error>>> {
    //     trace!("Formatting SD card...");
    //     let mut format_inner = BufStream::<_, 512>::new(&mut self.spi_sd);
    //     let format_options = FormatVolumeOptions::new()
    //         .volume_id(1)
    //         .volume_label(*b"sd_card    ");

    //     format_volume(&mut format_inner, format_options).await
    // }

}