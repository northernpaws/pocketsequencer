use alloc::vec::Vec;

/// Identifies the type of chunk in a RIFF file.
pub enum ChunkType {
    Riff,
    Fmt,
    Data,
    Wave,
    Unknown([u8; 4]),
}

impl ChunkType {
    fn from_bytes(bytes: &[u8; 4]) -> Self {
        match bytes {
            [b'R', b'I', b'F', b'F'] => ChunkType::Riff,
            [b'f', b'm', b't', b' '] => ChunkType::Fmt,
            [b'd', b'a', b't', b'a'] => ChunkType::Data,
            [b'W', b'A', b'V', b'E'] => ChunkType::Wave,
            _ => ChunkType::Unknown(*bytes),
        }
    }

    fn to_bytes(self) -> [u8; 4] {
        match self {
            ChunkType::Riff => [b'R', b'I', b'F', b'F'],
            ChunkType::Fmt => [b'f', b'm', b't', b' '],
            ChunkType::Data => [b'd', b'a', b't', b'a'],
            ChunkType::Wave => [b'W', b'A', b'V', b'E'],
            ChunkType::Unknown(bytes) => bytes,
        }
    }
}

/// A chunk decoded from a RIFF file.
pub struct Chunk {
    /// The type of the chunk.
    pub kind: ChunkType,
    /// The raw chunk data, extracted
    /// using the chunk length header.
    pub data: Vec<u8>,
}
