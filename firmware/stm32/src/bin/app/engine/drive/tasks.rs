use defmt::{error, info, unwrap};
use embassy_executor::{SpawnError, Spawner};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pipe::{self, Pipe};
use embedded_io_async::{Read, Write};
use heapless::index_map::FnvIndexMap;

use firmware::hardware::internal_storage::InternalStorage;
use firmware::hardware::sd_card::{self, SdFilesystem};
use static_cell::StaticCell;

use crate::engine::drive::{CommandID, CommandReceiver, CommandResult, CommandResultPublisher};

static SD_FILESYSTEM: StaticCell<SdFilesystem<'static>> = StaticCell::new();

pub fn spawn_drive(
    sd_card: SdFilesystem<'static>,
    internal_storage: InternalStorage,
    spawner: Spawner,
    command_receiver: CommandReceiver<'static>,
    command_result_publisher: CommandResultPublisher<'static>,
) -> Result<(), SpawnError> {
    let sd_fs = SD_FILESYSTEM.init(sd_card);

    spawner.spawn(drive_task(
        spawner,
        sd_fs,
        internal_storage,
        command_receiver,
        command_result_publisher,
    )?);

    Ok(())
}

#[embassy_executor::task]
pub async fn drive_task(
    spawner: Spawner,
    sd_card: &'static mut SdFilesystem<'static>,
    internal_storage: InternalStorage,
    command_receiver: CommandReceiver<'static>,
    command_result_publisher: CommandResultPublisher<'static>,
) -> ! {
    // should never return
    let err = inner_drive_task(
        spawner,
        sd_card,
        internal_storage,
        command_receiver,
        command_result_publisher,
    )
    .await;
    panic!("drive task exited unexpectedly: {:?}", err);
}

#[derive(Debug)]
enum Never {}

async fn inner_drive_task(
    spawner: Spawner,
    sd_card: &'static mut sd_card::SdFilesystem<'static>,
    mut internal_storage: InternalStorage,
    command_receiver: CommandReceiver<'static>,
    command_result_publisher: CommandResultPublisher<'static>,
) -> Result<(), Never> {
    loop {
        // Wait for the next drive command to arrive.
        let (command_id, command) = command_receiver.receive().await;

        match command {
            // Writes the contents of a provided buffer to the file.
            super::Command::WriteFile { path, buffer } => {
                info!("drive: open_file path={:?}", path.as_str());

                // Open a handle to the requested file.
                if let Ok(mut file) = sd_card.root_dir().open_file(path.as_str()).await {
                    file.write_all(&buffer).await.unwrap();
                    file.close().await.unwrap();

                    command_result_publisher
                        .publish((command_id, CommandResult::Ok))
                        .await;
                } else {
                    error!("error opening file!");
                    command_result_publisher
                        .publish((command_id, CommandResult::Error))
                        .await;
                }
            }

            // Opens a file for stream reading.
            super::Command::OpenFile { path, writer } => {
                info!("drive: open_file path={:?}", path.as_str());

                // Get a handle to the root directory to start traversal.
                let root_dir = sd_card.root_dir();

                // Open a handle to the requested file.
                if let Ok(file) = root_dir.open_file(path.as_str()).await {
                    // Spawn the task to handle reading from the file to the pipe.
                    spawner.spawn(unwrap!(read_file_task(file, writer)));

                    command_result_publisher
                        .publish((command_id, CommandResult::Ok))
                        .await;
                } else {
                    // TODO: need to signal closure somehow..
                    // writer.close()
                    error!("error opening file!");
                    command_result_publisher
                        .publish((command_id, CommandResult::Error))
                        .await;
                }
            }
        }
    }
}

/// Takes an open file handle and a pipe writer, and
/// buffers the file into the writer as required.
#[embassy_executor::task]
async fn read_file_task(
    mut file: sd_card::File<'static, 'static>,
    mut writer: pipe::Writer<'static, CriticalSectionRawMutex, { super::PIPE_SIZE }>,
) {
    // Loop to read from the file into the provided cross-task pipe.
    loop {
        let mut buf = [0u8; { super::PIPE_SIZE }];

        // Read a block of bytes from the file on the SD card.
        let _read = file.read(&mut buf).await.unwrap();

        // Write the bytes back to the file opened via the pipe.
        if let Err(err) = writer.write_all(&buf).await {
            error!("error writing to pipe: {:?}", err);
            break;
        }

        // TODO: there is currently no rewind handling, which we need...
    }

    // Ensure the file handle is closed.
    file.close().await.unwrap();
}
