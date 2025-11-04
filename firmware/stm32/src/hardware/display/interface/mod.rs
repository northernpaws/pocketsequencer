use embedded_graphics_core::pixelcolor::{Rgb565, Rgb666, RgbColor};
use num::PrimInt;

pub mod fmc;

/// Base trait for implementing display interfaces.
/// 
/// Defines the size of a word.
pub trait Interface {
    type Word: Copy + PrimInt;
    type Error: core::fmt::Debug;
}

pub trait WriteInterface: Interface {
    /// Writes a command word to the display.
    fn write_command(&mut self, cmd: u8, args: &[u8]) -> impl Future<Output = Result<(), Self::Error>>;

    /// Writes a data word to the display.
    fn write_data(&mut self, data: Self::Word) -> impl Future<Output = Result<(), Self::Error>>;
}

pub trait ReadInterface: Interface {
    /// Reads a data word from the display.
    fn read_data(&mut self) -> impl Future<Output = Result<Self::Word, Self::Error>>;
}

pub trait ColorFormat<Word> {
    fn send_pixels<I: Interface<Word = Word>>(
        di: &mut I,
        pixels: impl IntoIterator<Item = Self>,
    ) -> Result<(), I::Error>;

    fn send_repeated_pixel<I: Interface<Word = Word>>(
        di: &mut I,
        pixel: Self,
        count: u32,
    ) -> Result<(), I::Error>;
}

fn rgb565_to_bytes(pixel: Rgb565) -> [u8; 2] {
    embedded_graphics_core::pixelcolor::raw::ToBytes::to_be_bytes(pixel)
}
fn rgb565_to_u16(pixel: Rgb565) -> [u16; 1] {
    [u16::from_ne_bytes(
        embedded_graphics_core::pixelcolor::raw::ToBytes::to_ne_bytes(pixel),
    )]
}
fn rgb666_to_bytes(pixel: Rgb666) -> [u8; 3] {
    [pixel.r(), pixel.g(), pixel.b()].map(|x| x << 2)
}

// TODO: other formats..

impl ColorFormat<u16> for Rgb565 {
    fn send_pixels<I: Interface<Word = u16>>(
        interface: &mut I,
        pixels: impl IntoIterator<Item = Self>,
    ) -> Result<(), I::Error> {
        di.send_pixels(pixels.into_iter().map(rgb565_to_u16))
    }

    fn send_repeated_pixel<I: Interface<Word = u16>>(
        interface: &mut I,
        pixel: Self,
        count: u32,
    ) -> Result<(), I::Error> {
        di.send_repeated_pixel(rgb565_to_u16(pixel), count)
    }
}
