use core::sync::atomic::{AtomicU32, Ordering};

use alloc::boxed::Box;
use defmt::error;
use embassy_executor::{SpawnError, Spawner};
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    channel::{self, Receiver, Sender},
    pipe::{self, Pipe},
    pubsub,
};

use firmware::hardware::{
    internal_storage::InternalStorage,
    sd_card::{self, SdFilesystem},
};

pub mod tasks;

/// Size of the buffer backing file access pipes.
pub const PIPE_SIZE: usize = 64;

pub type CommandID = u32;

/// A command is send to the drive task to perform a file operation.
pub enum Command {
    /// Writes the provided contents to the specified file.
    WriteFile {
        /// Path to the file.
        path: heapless::String<255>,

        /// Byte buffer to write to the file.
        buffer: Box<[u8]>,
    },

    /// Attempts to read the entire contexts of a file.
    ReadFile {
        /// Path to the file.
        path: heapless::String<255>,
    },

    /// Opens a file for stream reading.
    OpenFile {
        /// Path to the file to open relative to the root of the SD card.
        path: heapless::String<255>,

        /// Pipe writer held by the command sender to read in the requested file.
        writer: pipe::Writer<'static, CriticalSectionRawMutex, PIPE_SIZE>,
    },
}

/// Specifies the results of a command exection.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum CommandResult {
    /// There was a filesystem error.
    ///
    /// Ideally should be `sd_card::FilesystemError`, but it's not clone'able
    /// due to embedded_io::Error not being annotated with Clone.
    Error,
    Ok,
    /// Contents of the file as a result of a read.
    ///
    /// TODO: this is not ideal because the box will
    ///  be cloned to every subscriber waiting for
    ///  a filesystem result.
    Content(Box<[u8]>),
}

pub type CommandReceiver<'a> =
    channel::Receiver<'a, CriticalSectionRawMutex, (CommandID, Command), 2>;
pub type CommandSender<'a> = channel::Sender<'a, CriticalSectionRawMutex, (CommandID, Command), 2>;

// Only one pub, the filesystem task!
pub type CommandResultSubscriber<'a> =
    pubsub::Subscriber<'a, CriticalSectionRawMutex, (CommandID, CommandResult), 5, 5, 1>;
pub type CommandResultPublisher<'a> =
    pubsub::Publisher<'a, CriticalSectionRawMutex, (CommandID, CommandResult), 5, 5, 1>;

/// Start the drive filesystem subsystem.
pub fn start(
    sd_card: SdFilesystem<'static>,
    internal_storage: InternalStorage,
    spawner: Spawner,
    command_receiver: CommandReceiver<'static>,
    command_result_publisher: CommandResultPublisher<'static>,
) -> Result<(), SpawnError> {
    // Start the async tasks for managing the drive.
    tasks::spawn_drive(
        sd_card,
        internal_storage,
        spawner,
        command_receiver,
        command_result_publisher,
    )?;

    Ok(())
}

// Fetches the next filesystem command ID.
pub fn next_command_id() -> CommandID {
    static NEXT_COMMAND_ID: AtomicU32 = AtomicU32::new(0);
    NEXT_COMMAND_ID.fetch_add(1, Ordering::Relaxed)
}

/// Instructs the filesystem task write the provided buffer to the specified path.
pub async fn write_file<'r>(
    // Channel sender for dispatching commands to the filesystem task.
    command_sender: CommandSender<'static>,
    // Pubsub subscriber for getting the command results.
    mut command_result_subscriber: CommandResultSubscriber<'static>,

    // Path of the file to write.
    path: heapless::String<255>,

    // Byte buffer to write to the file.
    buffer: Box<[u8]>,
) -> CommandResult {
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

                return result;
            }
        }
    }
}

/// Instructs the filesystem task to read a file from the specified path.
///
/// This only works well for fairly small files.
pub async fn read_file<'r>(
    // Channel sender for dispatching commands to the filesystem task.
    command_sender: CommandSender<'static>,
    // Pubsub subscriber for getting the command results.
    mut command_result_subscriber: CommandResultSubscriber<'static>,

    // Path of the file to write.
    path: heapless::String<255>,
) -> CommandResult {
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

                return result;
            }
        }
    }
}

/// Instructs the filesystem task to open the specified file
/// path and start reading it into the returned pipe.
pub async fn open_file<'r>(
    // Channel sender for dispatching commands to the filesystem task.
    command_sender: CommandSender<'static>,
    // Pubsub subscriber for getting the command results.
    mut command_result_subscriber: CommandResultSubscriber<'static>,

    // Path of the file to read.
    path: heapless::String<255>,
) -> Result<pipe::Reader<'r, CriticalSectionRawMutex, PIPE_SIZE>, CommandResult> {
    let command_id = next_command_id();

    // Create a pipe for reading the file back.
    let mut pipe = Pipe::new();
    let (reader, writer) = pipe.split();

    // Dispatch the filesystem command to open the file, and use
    // the provided writer to stream the file back to the caller.
    command_sender
        .send((command_id, Command::OpenFile { path, writer }))
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

                if result == CommandResult::Error {
                    return Err(result);
                }

                break;
            }
        }
    }

    Ok(reader)
}
