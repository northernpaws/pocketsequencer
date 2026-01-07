use core::hash::{BuildHasher, BuildHasherDefault, Hash};

use alloc::string::String;
use alloc::vec::Vec;
use defmt::{error, info, unwrap};
use embassy_executor::{SpawnError, Spawner};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pipe;
use embedded_io_async::{Read, Write};

use firmware::hardware::internal_storage::InternalStorage;
use firmware::hardware::sd_card::{self, SdFilesystem};
use hash32::Hasher;
use heapless::index_map::FnvIndexMap;
use static_cell::StaticCell;

use crate::engine::drive::{CommandReceiver, CommandResult, CommandResultPublisher};

static SD_FILESYSTEM: StaticCell<SdFilesystem<'static>> = StaticCell::new();

/// Spawns all the tasks required for the drive.
///
/// These include the SD card and internal storage tasks
/// that handle serving file read/write operation requests.
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
    let mut file_table = FnvIndexMap::<u32, sd_card::File<'static, 'static>, 16>::new();

    use hash32::FnvHasher;
    let mut hasher = BuildHasherDefault::<FnvHasher>::new().build_hasher();

    loop {
        // Wait for the next drive command to arrive.
        let (command_id, command) = command_receiver.receive().await;

        match command {
            super::Command::WriteFile { path, buffer } => {
                info!("drive: write_file path={:?}", path.as_str());

                // Open a handle to the requested file.
                if let Ok(mut file) = sd_card.root_dir().open_file(path.as_str()).await {
                    // Write the buffer to the file, waiting if needed.
                    file.write_all(&buffer).await.unwrap();

                    // Ensure the write buffer is flushed to the file.
                    file.flush().await.unwrap();

                    // Close the file handle.
                    file.close().await.unwrap();

                    command_result_publisher
                        .publish((command_id, CommandResult::Ok))
                        .await;
                } else {
                    error!("error opening file!");
                    // TODO: include error type in error response
                    command_result_publisher
                        .publish((command_id, CommandResult::Error))
                        .await;
                }
            }
            super::Command::ReadFile { path } => {
                info!("drive: read_file path={:?}", path.as_str());

                // Open a handle to the requested file.
                if let Ok(mut file) = sd_card.root_dir().open_file(path.as_str()).await {
                    let mut buf: Vec<u8> = Vec::new();

                    // Read the file in a loop until we hit EOF.
                    loop {
                        let mut buffer = [0u8; 64];

                        match file.read(&mut buffer).await {
                            Ok(read) => {
                                if read == 0 {
                                    break;
                                } else {
                                    buf.extend_from_slice(&buffer[0..read])
                                }
                            }
                            Err(err) => {
                                error!("error reading file!");
                                // TODO: include error type in error response
                                command_result_publisher
                                    .publish((command_id, CommandResult::Error))
                                    .await;
                            }
                        }
                    }

                    file.close().await.unwrap();

                    // Box the vec and send it as the command results.
                    command_result_publisher
                        .publish((command_id, CommandResult::Content(buf.into_boxed_slice())))
                        .await;
                } else {
                    error!("error opening file!");
                    command_result_publisher
                        .publish((command_id, CommandResult::Error))
                        .await;
                }
            }
            super::Command::ListDirectory { path } => {
                info!("drive: list_directory path={:?}", path.as_str());

                // Vec for storing the directory listing.
                let mut entries: Vec<super::FileInfo> = Vec::new();

                // Open a handle to the requested directory.
                if let Ok(dir) = sd_card.root_dir().open_dir(path.as_str()).await {
                    // Open an interator over the directory's contents.
                    while let Some(r) = dir.iter().next().await {
                        // Add the entry to the file contents vector depending on the type.
                        match r {
                            Ok(entry) => {
                                if entry.is_dir() {
                                    entries.push(super::FileInfo::Directory(entry.file_name()));
                                } else {
                                    entries.push(super::FileInfo::File(entry.file_name()));
                                }
                            }
                            Err(err) => {
                                error!("error reading file in iter!");
                            }
                        }
                    }
                } else {
                    error!("error opening file!");
                    // TODO: include error type in error response
                    command_result_publisher
                        .publish((command_id, CommandResult::Error))
                        .await;
                }

                // Send the directory listing back to the caller.
                command_result_publisher
                    .publish((command_id, CommandResult::Listing(entries)))
                    .await;
            }
            super::Command::OpenFile { path } => {
                info!("drive: open_file path={}", path.as_str());

                // Get a handle to the root directory to start traversal.
                let root_dir = sd_card.root_dir();

                // Open a handle to the requested file.
                if let Ok(file) = root_dir.open_file(path.as_str()).await {
                    path.hash(&mut hasher);
                    let hash = hasher.finish32();

                    if let Err(_err) = file_table.insert(hash, file) {
                        panic!("failed to add file to table: {}", path);
                    };

                    // Inform the caller that the operation started successfully.
                    command_result_publisher
                        .publish((command_id, CommandResult::FileOpened { hash }))
                        .await;
                } else {
                    // TODO: need to signal closure somehow..
                    // writer.close()
                    error!("error opening file!");
                    // TODO: include error type in error response
                    command_result_publisher
                        .publish((command_id, CommandResult::Error))
                        .await;
                }
            }
            super::Command::ReadFromFile {
                file_id,
                mut buffer,
            } => {
                if let Some(file) = file_table.get_mut(&file_id) {
                    info!("drive: read_from_file file_id={}", file_id);

                    file.read_exact(&mut buffer).await.unwrap();

                    command_result_publisher
                        .publish((command_id, CommandResult::Content(buffer)))
                        .await;
                } else {
                    command_result_publisher
                        .publish((command_id, CommandResult::Error))
                        .await;
                }
            }
            super::Command::CloseFile { file_id } => {
                if let Some(file) = file_table.remove(&file_id) {
                    info!("drive: close_file file_id={}", file_id);

                    // Close the file handle.
                    file.close().await.unwrap();

                    command_result_publisher
                        .publish((command_id, CommandResult::Ok))
                        .await;
                } else {
                    command_result_publisher
                        .publish((command_id, CommandResult::Error))
                        .await;
                }
            }
        }
    }
}
