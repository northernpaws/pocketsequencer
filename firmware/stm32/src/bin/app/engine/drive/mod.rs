use alloc::boxed::Box;
use embassy_executor::{SpawnError, Spawner};
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    channel::{Receiver, Sender},
    pipe::{self, Pipe},
};

use firmware::hardware::{internal_storage::InternalStorage, sd_card::SdFilesystem};

pub mod tasks;

/// Size of the buffer backing file access pipes.
pub const PIPE_SIZE: usize = 64;

/// A command is send to the drive task to perform a file operation.
pub enum Command {
    /// Writes the provided contents to the specified file.
    WriteFile {
        /// Path to the file.
        path: heapless::String<255>,

        /// Byte buffer to write to the file.
        buffer: Box<[u8]>,
    },

    /// Opens a file for stream reading.
    OpenFile {
        /// Path to the file to open relative to the root of the SD card.
        path: heapless::String<255>,

        /// Pipe writer held by the command sender to read in the requested file.
        writer: pipe::Writer<'static, CriticalSectionRawMutex, PIPE_SIZE>,
    },
}

pub type CommandReceiver<'a> = Receiver<'a, CriticalSectionRawMutex, Command, 2>;
pub type CommandSender<'a> = Sender<'a, CriticalSectionRawMutex, Command, 2>;

/// Start the drive filesystem subsystem.
pub fn start(
    sd_card: SdFilesystem<'static>,
    internal_storage: InternalStorage,
    spawner: Spawner,
    command_receiver: CommandReceiver<'static>,
) -> Result<(), SpawnError> {
    // Start the async tasks for managing the drive.
    tasks::spawn_drive(sd_card, internal_storage, spawner, command_receiver)?;

    Ok(())
}

/// Instructs the filesystem task write the provided buffer to the specified path.
pub async fn write_file<'r>(
    // Channel sender for dispatching commands to the filesystem task.
    command_sender: CommandSender<'static>,

    // Path of the file to write.
    path: heapless::String<255>,

    // Byte buffer to write to the file.
    buffer: Box<[u8]>,
) {
    // Dispatch the filesystem command to write the file.
    command_sender
        .send(Command::WriteFile { path, buffer })
        .await;
}

// /// Instructs the filesystem task to open the specified file
// /// path and start reading it into the returned pipe.
// pub async fn read_file<'r>(
//     // Channel sender for dispatching commands to the filesystem task.
//     command_sender: CommandSender<'static>,

//     // Path of the file to read.
//     path: heapless::String<255>,
// ) -> pipe::Reader<'r, CriticalSectionRawMutex, PIPE_SIZE> {
//     // Create a pipe for reading the file back.
//     // let mut pipe = Pipe::new();
//     // let (reader, writer) = pipe.split();

//     // Dispatch the filesystem command to open the file, and use
//     // the provided writer to stream the file back to the caller.
//     command_sender
//         .send(Command::OpenFile { path, writer })
//         .await;

//     reader
// }
