use core::{
    mem::ManuallyDrop,
    sync::atomic::{AtomicU32, Ordering},
};

use alloc::{boxed::Box, string::String, vec::Vec};
use defmt::error;
use embassy_executor::{SpawnError, Spawner};
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    channel,
    pubsub::{self, PubSubChannel},
};

use firmware::hardware::{
    internal_storage::InternalStorage,
    sd_card::{self, SdFilesystem},
};

pub mod tasks;

/// Alias type for the channel receiver used by the filesystem task to receive commands.
pub type CommandReceiver<'a> =
    channel::Receiver<'a, CriticalSectionRawMutex, (CommandID, Command), 2>;
/// Alias type for the channel sender used by other tasks to send commands to the filesystem task.
pub type CommandSender<'a> = channel::Sender<'a, CriticalSectionRawMutex, (CommandID, Command), 2>;
/// Alias type for the channel used to send commands to the filesystem.
pub type CommandChannel = channel::Channel<CriticalSectionRawMutex, (CommandID, Command), 2>;

// NOTE: Only one pub, the filesystem task!
pub type CommandResultSubscriber<'a> =
    pubsub::Subscriber<'a, CriticalSectionRawMutex, (CommandID, CommandResult), 12, 6, 1>;
pub type CommandResultPublisher<'a> =
    pubsub::Publisher<'a, CriticalSectionRawMutex, (CommandID, CommandResult), 12, 6, 1>;
/// Alias type for the pubsub channel used to send command results back to callers.
pub type CommandResultChannel =
    pubsub::PubSubChannel<CriticalSectionRawMutex, (CommandID, CommandResult), 12, 6, 1>;

/// Generated before dispatching a command to the filesystem task,
/// and used to corrolate that command with the corrosponding result.
pub type CommandID = u32;

/// A command is send to the drive task to perform a file operation.
pub enum Command {
    /// Writes the provided contents to the specified file.
    WriteFile {
        /// Path to the file.
        path: String,

        /// Byte buffer to write to the file.
        buffer: Box<[u8]>,
    },

    /// Attempts to read the entire contexts of a file.
    ReadFile {
        /// Path to the file.
        path: String,
    },

    /// Attempts to list the contents of a directory.
    ListDirectory {
        /// Path to the file.
        path: String,
    },

    /// Opens a file for stream reading.
    OpenFile {
        /// Path to the file to open relative to the root of the SD card.
        path: String,
    },

    /// Reads from a previously opened file handle.
    ReadFromFile { file_id: u32, buffer: Vec<u8> },

    /// Request to close an open file handle.
    CloseFile { file_id: u32 },
}

/// Provides information about a file.
#[derive(Debug, Clone, PartialEq, Eq, defmt::Format)]
pub enum FileInfo {
    /// Provides information about a file entry in a directory.
    File(String),
    /// Provides information about a directory entry in a directory.
    Directory(String),
}

/// Indicates the type of file access error that occured.
#[derive(Debug, Clone, PartialEq, Eq, defmt::Format)]
pub enum FilesystemError {
    UnexpectedEof,
    WriteZero,
    InvalidInput,
    NotFound,
    AlreadyExists,
    DirectoryIsNotEmpty,
    CorruptedFileSystem,
    NotEnoughSpace,
    InvalidFileNameLength,
    UnsupportedFileNameCharacter,
}

/// Indicates the type of file access error that occured.
#[derive(Debug, Clone, PartialEq, Eq, defmt::Format)]
pub enum IOError {
    InvalidSeek(i64),
    WriteZero,
    Unknown,
}

/// Indicates the type of SPI device error that occured trying to access a file.
#[derive(Debug, Clone, PartialEq, Eq, defmt::Format)]
pub enum DeviceError {
    ChipSelect,
    SpiError,
    Timeout,
    UnsupportedCard,
    Cmd58Error,
    Cmd59Error,
    RegisterError(u8),
    CrcMismatch(u16, u16),
    NotInitialized,
    WriteError,
    Unknown,
}

/// Indicates the type of error that occured while accessing the drive.
#[derive(Debug, Clone, PartialEq, Eq, defmt::Format)]
pub enum Error {
    Filesystem(FilesystemError),
    IO(IOError),
    Device(DeviceError),
}

/// Converts from an SD card filesystem error to an IO error.
impl From<sd_card::FilesystemError> for Error {
    fn from(err: sd_card::FilesystemError) -> Self {
        match err {
            embedded_fatfs::Error::Io(err) => match err {
                block_device_adapters::StreamSliceError::InvalidSeek(i64) => {
                    Error::IO(IOError::InvalidSeek(i64))
                }
                block_device_adapters::StreamSliceError::WriteZero => Error::IO(IOError::WriteZero),
                block_device_adapters::StreamSliceError::Other(other) => match other {
                    block_device_adapters::BufStreamError::Io(buf_io) => {
                        Error::Device(match buf_io {
                            sdspi::Error::ChipSelect => DeviceError::ChipSelect,
                            sdspi::Error::SpiError => DeviceError::SpiError,
                            sdspi::Error::Timeout => DeviceError::Timeout,
                            sdspi::Error::UnsupportedCard => DeviceError::UnsupportedCard,
                            sdspi::Error::Cmd58Error => DeviceError::Cmd58Error,
                            sdspi::Error::Cmd59Error => DeviceError::Cmd59Error,
                            sdspi::Error::RegisterError(reg) => DeviceError::RegisterError(reg),
                            sdspi::Error::CrcMismatch(a, b) => DeviceError::CrcMismatch(a, b),
                            sdspi::Error::NotInitialized => DeviceError::NotInitialized,
                            sdspi::Error::WriteError => DeviceError::WriteError,
                            _ => DeviceError::Unknown,
                        })
                    }
                    _ => Error::IO(IOError::Unknown),
                },
                _ => Error::IO(IOError::Unknown),
            },
            embedded_fatfs::Error::UnexpectedEof => {
                Error::Filesystem(FilesystemError::UnexpectedEof)
            }
            embedded_fatfs::Error::WriteZero => Error::Filesystem(FilesystemError::WriteZero),
            embedded_fatfs::Error::InvalidInput => Error::Filesystem(FilesystemError::InvalidInput),
            embedded_fatfs::Error::NotFound => Error::Filesystem(FilesystemError::NotFound),
            embedded_fatfs::Error::AlreadyExists => {
                Error::Filesystem(FilesystemError::AlreadyExists)
            }
            embedded_fatfs::Error::DirectoryIsNotEmpty => {
                Error::Filesystem(FilesystemError::DirectoryIsNotEmpty)
            }
            embedded_fatfs::Error::CorruptedFileSystem => {
                Error::Filesystem(FilesystemError::CorruptedFileSystem)
            }
            embedded_fatfs::Error::NotEnoughSpace => {
                Error::Filesystem(FilesystemError::NotEnoughSpace)
            }
            embedded_fatfs::Error::InvalidFileNameLength => {
                Error::Filesystem(FilesystemError::InvalidFileNameLength)
            }
            embedded_fatfs::Error::UnsupportedFileNameCharacter => {
                Error::Filesystem(FilesystemError::UnsupportedFileNameCharacter)
            }
            _ => todo!(),
        }
    }
}

/// Specifies the results of a command exection.
#[derive(Debug, Clone, PartialEq, Eq, defmt::Format)]
pub enum CommandResult {
    /// There was a filesystem error.
    ///
    /// Ideally should be `sd_card::FilesystemError`, but it's not clone'able
    /// due to embedded_io::Error not being annotated with Clone.
    Error(Error),

    /// The operation finished successfully.
    Ok,

    /// Contents of the file as a result of a read.
    ///
    /// TODO: this is not ideal because the box will
    ///  be cloned to every subscriber waiting for
    ///  a filesystem result.
    Content(Vec<u8>),

    /// A vector listing the contents of a directory.
    Listing(Vec<FileInfo>),

    FileOpened {
        hash: u32,
    },
}

/// A handle for a file opened by the filesystem used to access it.
///
/// Returned by the [`open_file`] method, referenced by [`read_from_file`]
/// and taken and destroyed by [`close_file`].
pub struct FileID(u32);

/// Start the drive filesystem subsystem.
pub async fn start<'a>(
    sd_card: SdFilesystem<'static>,
    internal_storage: InternalStorage,
    spawner: Spawner,
) -> Result<Drive<'a>, SpawnError> {
    /// Channel for sending commands to the filesystem task.
    static DRIVE_COMMANDS: CommandChannel = channel::Channel::new();
    /// PubSub channel for receiving results from the filesystem task.
    static DRIVE_RESULTS: CommandResultChannel = PubSubChannel::new();

    let command_receiver: CommandReceiver<'static> = DRIVE_COMMANDS.receiver();

    let command_result_publisher: CommandResultPublisher<'static> = DRIVE_RESULTS
        .publisher()
        .expect("The only publisher should be the filesystem task");

    // Start the async tasks for managing the drive.
    tasks::spawn_drive(
        sd_card,
        internal_storage,
        spawner,
        command_receiver,
        command_result_publisher,
    )?;

    // Return a convenience wrapper around the channels for running drive operations.
    Ok(Drive {
        commands: &DRIVE_COMMANDS,
        results: &DRIVE_RESULTS,
    })
}

// Fetches the next filesystem command ID.
pub fn next_command_id() -> CommandID {
    static NEXT_COMMAND_ID: AtomicU32 = AtomicU32::new(0);
    NEXT_COMMAND_ID.fetch_add(1, Ordering::Relaxed)
}

/// Instructs the filesystem task write the provided buffer to the specified path.
pub async fn write_file(
    // Channel sender for dispatching commands to the filesystem task.
    command_sender: &'_ CommandSender<'_>,
    // Pubsub subscriber for getting the command results.
    command_result_subscriber: &'_ mut CommandResultSubscriber<'_>,

    // Path of the file to write.
    path: String,

    // Byte buffer to write to the file.
    buffer: Box<[u8]>,
) -> Result<(), CommandResult> {
    let command_id = next_command_id();

    // Dispatch the filesystem command to write the file.
    command_sender
        .send((command_id, Command::WriteFile { path, buffer }))
        .await;

    // Wait for the filesystem task to process the request.
    loop {
        // TODO: need some kind of timeout

        match command_result_subscriber.next_message().await {
            pubsub::WaitResult::Lagged(_) => {
                // TODO: should never occur, but should probably have some type of handling
                error!("command result lagged!");
            }
            pubsub::WaitResult::Message((id, result)) => {
                if id != command_id {
                    continue;
                }

                return match result {
                    CommandResult::Ok => Ok(()),
                    _ => Err(result),
                };
            }
        }
    }
}

/// Instructs the filesystem task to read a file from the specified path.
///
/// This only works well for fairly small files.
pub async fn read_file(
    // Channel sender for dispatching commands to the filesystem task.
    command_sender: &'_ CommandSender<'_>,
    // Pubsub subscriber for getting the command results.
    command_result_subscriber: &'_ mut CommandResultSubscriber<'_>,

    // Path of the file to write.
    path: String,
) -> Result<Vec<u8>, CommandResult> {
    let command_id = next_command_id();

    // Dispatch the filesystem command to write the file.
    command_sender
        .send((command_id, Command::ReadFile { path }))
        .await;

    // Wait for the filesystem task to process the request.
    loop {
        // TODO: need some kind of timeout
        match command_result_subscriber.next_message().await {
            pubsub::WaitResult::Lagged(_) => {
                // TODO: should never occur, but should probably have some type of handling
                error!("command result lagged!");
            }
            pubsub::WaitResult::Message((id, result)) => {
                if id != command_id {
                    continue;
                }

                return match result {
                    CommandResult::Content(buffer) => Ok(buffer),
                    _ => Err(result),
                };
            }
        }
    }
}

/// Instructs the filesystem task to list the contents of a directory at the specified path.
pub async fn list_directory(
    // Channel sender for dispatching commands to the filesystem task.
    command_sender: &'_ CommandSender<'_>,
    // Pubsub subscriber for getting the command results.
    command_result_subscriber: &'_ mut CommandResultSubscriber<'_>,

    // Path of the directory to list.
    path: String,
) -> Result<Vec<FileInfo>, CommandResult> {
    let command_id = next_command_id();

    // Dispatch the filesystem command to write the file.
    command_sender
        .send((command_id, Command::ListDirectory { path }))
        .await;

    // Wait for the filesystem task to process the request.
    loop {
        // TODO: need some kind of timeout
        match command_result_subscriber.next_message().await {
            pubsub::WaitResult::Lagged(_) => {
                // TODO: should never occur, but should probably have some type of handling
                error!("command result lagged!");
            }
            pubsub::WaitResult::Message((id, result)) => {
                if id != command_id {
                    continue;
                }

                return match result {
                    CommandResult::Listing(file_infos) => Ok(file_infos),
                    _ => Err(result),
                };
            }
        }
    }
}

/// Instructs the filesystem task to open the specified file
/// and return a file ID that can be used to access the file
/// for subsequent streaming reads and writes.
pub async fn open_file(
    // Channel sender for dispatching commands to the filesystem task.
    command_sender: &'_ CommandSender<'_>,
    // Pubsub subscriber for getting the command results.
    command_result_subscriber: &'_ mut CommandResultSubscriber<'_>,

    // Path of the file to read.
    path: String,
) -> Result<FileID, CommandResult> {
    let command_id = next_command_id();

    // Dispatch the filesystem command to open the file, and use
    // the provided writer to stream the file back to the caller.
    command_sender
        .send((command_id, Command::OpenFile { path }))
        .await;

    // Wait for the filesystem task to awknowledge the request.
    loop {
        // TODO: need some kind of timeout
        match command_result_subscriber.next_message().await {
            pubsub::WaitResult::Lagged(_) => {
                // TODO: should never occur, but should probably have some type of handling
                error!("command result lagged!");
            }
            pubsub::WaitResult::Message((id, result)) => {
                if id != command_id {
                    continue;
                }

                return match result {
                    CommandResult::FileOpened { hash } => Ok(FileID(hash)),
                    _ => Err(result),
                };
            }
        }
    }
}

/// Read a buffer of bytes from a previously opened file.
pub async fn read_from_file(
    // Channel sender for dispatching commands to the filesystem task.
    command_sender: &'_ CommandSender<'_>,
    // Pubsub subscriber for getting the command results.
    command_result_subscriber: &'_ mut CommandResultSubscriber<'_>,

    file_id: &FileID,

    buffer: Vec<u8>,
) -> Result<Vec<u8>, CommandResult> {
    let command_id = next_command_id();

    command_sender
        .send((
            command_id,
            Command::ReadFromFile {
                file_id: file_id.0,
                buffer,
            },
        ))
        .await;

    // Wait for the filesystem task to awknowledge the request.
    loop {
        // TODO: need some kind of timeout
        match command_result_subscriber.next_message().await {
            pubsub::WaitResult::Lagged(_) => {
                // TODO: should never occur, but should probably have some type of handling
                error!("command result lagged!");
            }
            pubsub::WaitResult::Message((id, result)) => {
                if id != command_id {
                    continue;
                }

                return match result {
                    CommandResult::Content(buffer) => Ok(buffer),
                    _ => Err(result),
                };
            }
        }
    }
}

/// Close a file that was previously opened.
pub async fn close_file(
    // Channel sender for dispatching commands to the filesystem task.
    command_sender: &'_ CommandSender<'_>,
    // Pubsub subscriber for getting the command results.
    command_result_subscriber: &'_ mut CommandResultSubscriber<'_>,

    file_id: FileID,
) -> Result<(), CommandResult> {
    let command_id = next_command_id();

    command_sender
        .send((command_id, Command::CloseFile { file_id: file_id.0 }))
        .await;

    // Wait for the filesystem task to acknowledge the request.
    loop {
        // TODO: need some kind of timeout
        match command_result_subscriber.next_message().await {
            pubsub::WaitResult::Lagged(_) => {
                // TODO: should never occur, but should probably have some type of handling
                error!("command result lagged!");
            }
            pubsub::WaitResult::Message((id, result)) => {
                if id != command_id {
                    continue;
                }

                return match result {
                    CommandResult::Ok => Ok(()),
                    _ => Err(result),
                };
            }
        }
    }
}

pub struct FilesystemInterface<'a> {
    // Channel sender for dispatching commands to the filesystem task.
    command_sender: CommandSender<'a>,

    // Pubsub subscriber for getting the command results.
    command_result_subscriber: CommandResultSubscriber<'a>,
}

impl<'a> FilesystemInterface<'a> {
    /// Overwrites or creates a file at the specified path with the contents specified in the buffer.
    pub async fn write_file(
        &mut self,
        path: String,
        buffer: Box<[u8]>,
    ) -> Result<(), CommandResult> {
        write_file(
            &self.command_sender,
            &mut self.command_result_subscriber,
            path,
            buffer,
        )
        .await
    }

    /// Reads an entire file into a buffer.
    pub async fn read_file(&mut self, path: String) -> Result<Vec<u8>, CommandResult> {
        read_file(
            &self.command_sender,
            &mut self.command_result_subscriber,
            path,
        )
        .await
    }

    /// Lists the entries in a specified directory.
    pub async fn list_directory(&mut self, path: String) -> Result<Vec<FileInfo>, CommandResult> {
        list_directory(
            &self.command_sender,
            &mut self.command_result_subscriber,
            path,
        )
        .await
    }

    /// Opens the specified file for stream reading/writing.
    pub async fn open_file(&'a mut self, path: String) -> Result<FileHandle<'a>, CommandResult> {
        let file_id = open_file(
            &self.command_sender,
            &mut self.command_result_subscriber,
            path,
        )
        .await?;

        Ok(FileHandle { fs: self, file_id })
    }
}

/// Wraps a filesystem interfaces and a file ID to reference a file opened in the filesystem.
pub struct FileHandle<'fs> {
    fs: &'fs mut FilesystemInterface<'fs>,
    file_id: FileID,
}

impl<'fs> FileHandle<'fs> {
    /// Reads a block of bytes into the provided buffer from the open file.
    pub async fn read(&mut self, buffer: Vec<u8>) -> Result<Vec<u8>, CommandResult> {
        read_from_file(
            &self.fs.command_sender,
            &mut self.fs.command_result_subscriber,
            &self.file_id,
            buffer,
        )
        .await
    }

    /// Closes the open file, consuming the handle and the underlying file reference ID.
    pub async fn close(self) -> Result<(), CommandResult> {
        close_file(
            &self.fs.command_sender,
            &mut self.fs.command_result_subscriber,
            self.file_id,
        )
        .await
    }
}

/// Returned when initializing the drive.
pub struct Drive<'a> {
    /// Channel for sending commands to the drive task.
    commands: &'a CommandChannel,

    /// Channel for receiving responses from the drive task.
    results: &'a CommandResultChannel,
}

impl<'a> Drive<'a> {
    /// Constructs a filesystem interface with the nessessary channel interfaces.
    pub async fn create_interface(&mut self) -> Result<FilesystemInterface<'_>, pubsub::Error> {
        // Create a channel sender for sending commands to the filesystem task.
        let command_sender = self.commands.sender();

        // Create a subscriber for receiving command results from the filesystem task.
        let command_result_subscriber = self.results.subscriber()?;

        Ok(FilesystemInterface {
            command_sender,
            command_result_subscriber,
        })
    }
}
