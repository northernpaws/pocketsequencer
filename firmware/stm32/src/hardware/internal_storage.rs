// use embedded_sdmmc::{
//     SdCard,
//     VolumeManager,
//     Mode,
//     VolumeIdx
// };

// TODO: There seems to be read/write issues with the internal storage,
//  possibly stemming from the TXSD chip having a 1024 block size.

pub struct InternalStorage {}

impl InternalStorage {
    pub fn new() -> Self {
        Self {}
    }

    pub fn init() {}
}
