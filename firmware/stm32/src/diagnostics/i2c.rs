use embassy_time::Delay;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::{
    mono_font::{MonoTextStyle, ascii::FONT_6X10, ascii::FONT_9X18_BOLD},
    pixelcolor::Rgb888,
    prelude::*,
    primitives::{PrimitiveStyleBuilder, Rectangle},
    text::{Alignment, LineHeight, Text, TextStyleBuilder},
};
use embedded_graphics_coordinate_transform::Rotate270;
use embedded_hal::i2c::SevenBitAddress;

use crate::hardware::{
    display::Display,
    drivers::{bq27531_g1, fusb302b, nau88c22yg, tca8418},
    keypad::Keypad,
};

pub async fn i2c_diagnostics(
    display: &'_ mut Rotate270<Display<'_, Delay>>,
    _keypad: &'_ mut Keypad,
) {
    let devices = [
        make_bq27532_g1_device(),
        make_tca8418_device(),
        make_fusb302b_device(),
        make_nau88c22yg_device(),
    ];

    let mut console = DiagnosticsConsole::new(&devices);

    loop {
        console.draw(display).unwrap();
        display.push_buffer_dma().await.unwrap();
    }
}

fn make_bq27532_g1_device() -> Device {
    Device::new(2, "bq27532_g1", bq27531_g1::DEVICE_ADDRESS)
}

fn make_tca8418_device() -> Device {
    Device::new(4, "tca8418", tca8418::DEFAULT_ADDRESS)
}

fn make_fusb302b_device() -> Device {
    Device::new(2, "fusb302b", fusb302b::FUSB302BMPX_ADDRESS)
}

fn make_nau88c22yg_device() -> Device {
    Device::new(1, "nau88c22yg", nau88c22yg::ADDRESS)
}

struct Device {
    bus: u8,
    name: &'static str,
    addr: SevenBitAddress,
}

impl Device {
    pub fn new(bus: u8, name: &'static str, addr: SevenBitAddress) -> Self {
        Self { bus, name, addr }
    }
}

pub enum Side {
    Controller,
    Peripheral,
}

pub enum Event {
    Address(SevenBitAddress, bool), // bool is write
    WriteByte(u8),
    ReadByte(u8),
}

impl Event {
    pub fn side(&self) -> Side {
        match self {
            Event::Address(_, _) => Side::Controller,
            Event::WriteByte(_) => Side::Controller,
            Event::ReadByte(_) => Side::Peripheral,
        }
    }
}

struct DiagnosticsConsole<'a> {
    devices: &'a [Device],
    event_log: [Option<Event>; 24],
}

impl<'a> DiagnosticsConsole<'a> {
    pub fn new(devices: &'a [Device]) -> Self {
        Self {
            devices,
            event_log: Default::default(),
        }
    }

    pub fn draw_header<D: DrawTarget>(&mut self, target: &mut D) -> Result<(), D::Error>
    where
        D::Color: RgbColor,
    {
        Rectangle::new(
            Point::new(0, 0),
            Size::new(
                target.bounding_box().size.width,
                target.bounding_box().size.height,
            ),
        )
        .into_styled(
            PrimitiveStyleBuilder::new()
                .stroke_width(0)
                .fill_color(D::Color::YELLOW)
                .build(),
        )
        .draw(target)?;

        // Black header font color so it appears inverted on a yellow header.
        let large_character_style = MonoTextStyle::new(&FONT_9X18_BOLD, D::Color::BLACK);

        let text_style = TextStyleBuilder::new()
            .alignment(Alignment::Left)
            .line_height(LineHeight::Percent(150))
            .build();

        // "DIAGNOSTICS" header text
        Text::with_text_style("I2C", Point::new(10, 19), large_character_style, text_style)
            .draw(target)?;

        Ok(())
    }

    pub fn draw_event_log<D: DrawTarget>(&mut self, target: &mut D) -> Result<(), D::Error>
    where
        D::Color: RgbColor,
    {
        Ok(())
    }

    /// Draw the diagnostic console to the specified draw target.
    pub fn draw<D: DrawTarget + OriginDimensions>(&mut self, target: &mut D) -> Result<(), D::Error>
    where
        D::Color: RgbColor,
    {
        target.clear(D::Color::BLACK)?;

        let mut header_area = target.clipped(&Rectangle::new(
            Point::new(0, 0),
            Size::new(target.size().width, 30),
        ));
        self.draw_header(&mut header_area)?;

        let mut event_log_area = target.clipped(&Rectangle::new(
            Point::new(0, 30),
            Size::new(120, target.size().height - 30),
        ));
        self.draw_event_log(&mut event_log_area)?;

        Ok(())
    }
}
