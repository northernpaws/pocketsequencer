use embassy_executor::{SpawnError, Spawner};

use firmware::hardware::internal_storage::InternalStorage;
use firmware::hardware::sd_card::SdCard;

pub fn start_drive(
    sd_card: SdCard<'static>,
    internal_storage: InternalStorage,
    spawner: Spawner,
) -> Result<(), SpawnError> {
    spawner.spawn(drive_task(sd_card, internal_storage)?);

    Ok(())
}

#[embassy_executor::task]
pub async fn drive_task(sd_card: SdCard<'static>, internal_storage: InternalStorage) -> ! {
    // should never return
    let err = inner_drive_task(sd_card, internal_storage).await;
    panic!("{:?}", err);
}

#[derive(Debug)]
enum Never {}

async fn inner_drive_task(
    mut sd_card: SdCard<'static>,
    mut internal_storage: InternalStorage,
) -> Result<(), Never> {
    loop {}
    Ok(())
}
