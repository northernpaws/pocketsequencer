use alloc::boxed::Box;

use crate::engine::drive::{self, FileHandle};

mod chunk;

const CHUNK_HEADER_RIFF: [u8; 4] = [b'R', b'I', b'F', b'F'];
const RIFF_FORMAT_WAVE: [u8; 4] = [b'W', b'A', b'V', b'E'];

#[derive(Debug, Clone, PartialEq, defmt::Format)]
pub enum Error {
    FileError(drive::CommandResult),
    /// The header read from the file is invalid.
    InvalidHeader,
    UnexpectedChunkSize([u8; 4], u32),
    UnexpectedRiffFormat([u8; 4]),
}

pub async fn read_header_steaming(mut handle: FileHandle<'_>) -> Result<(), Error> {
    // First, read the first 4 bytes.
    //
    // For a valid WAV file, these should be RIFF.
    let header = handle
        .read(Box::new([0u8; 4]))
        .await
        .map_err(|e| Error::FileError(e))?;

    // Check that the header is as expected.
    if header.into_vec() != CHUNK_HEADER_RIFF {
        return Err(Error::InvalidHeader);
    }

    // Read the next 4 bytes as the chunk size.
    let riff_chunk_size = handle
        .read(Box::new([0u8; 4]))
        .await
        .map_err(|e| Error::FileError(e))?;

    // Decode the little-endian chunk size.
    let riff_chunk_size = u32::from_le_bytes([
        riff_chunk_size[0],
        riff_chunk_size[1],
        riff_chunk_size[2],
        riff_chunk_size[3],
    ]);

    // Sanity-check the read chunk size.
    if riff_chunk_size != 4 {
        return Err(Error::UnexpectedChunkSize(
            CHUNK_HEADER_RIFF,
            riff_chunk_size,
        ));
    }

    // The next 4 bytes will be the RIFF format.
    let format = handle
        .read(Box::new([0u8; 4]))
        .await
        .map_err(|e| Error::FileError(e))?;

    // Check that the declared format is "WAVE".
    if format.to_vec() != RIFF_FORMAT_WAVE {
        return Err(Error::UnexpectedRiffFormat([
            format[0], format[1], format[2], format[3],
        ]));
    }

    Ok(())
}

pub async fn read_header(buffer: &[u8; 44]) -> Result<(), Error> {
    // First, read the first 4 bytes.
    //
    // For a valid WAV file, these should be RIFF.
    if buffer[0..4] != CHUNK_HEADER_RIFF {
        return Err(Error::InvalidHeader);
    }

    // Decode the little-endian chunk size.
    let riff_chunk_size = u32::from_le_bytes([buffer[4], buffer[5], buffer[6], buffer[7]]);

    // Sanity-check the read chunk size.
    if riff_chunk_size != 4 {
        return Err(Error::UnexpectedChunkSize(
            CHUNK_HEADER_RIFF,
            riff_chunk_size,
        ));
    }

    // The next 4 bytes will be the RIFF format.
    //
    // Check that the declared format is "WAVE".
    if buffer[7..12] != RIFF_FORMAT_WAVE {
        return Err(Error::UnexpectedRiffFormat([
            buffer[7], buffer[8], buffer[9], buffer[10],
        ]));
    }

    Ok(())
}
