use embassy_executor::{SpawnError, Spawner};
use firmware::hardware::{internal_storage::InternalStorage, sd_card::SdCard};

pub mod task;

/// Start the drive filesystem subsystem.
pub fn start(
    sd_card: SdCard<'static>,
    internal_storage: InternalStorage,
    spawner: Spawner,
) -> Result<(), SpawnError> {
    task::start_drive(sd_card, internal_storage, spawner)
}
