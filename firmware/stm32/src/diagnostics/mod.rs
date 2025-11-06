use embassy_time::Delay;
use embedded_graphics::{
    mono_font::{
        MonoTextStyle,
        ascii::{FONT_6X10, FONT_10X20},
    },
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{PrimitiveStyleBuilder, Rectangle},
    text::{Alignment, LineHeight, Text, TextStyleBuilder},
};
use embedded_graphics_coordinate_transform::Rotate270;

use crate::hardware::{display::Display, keypad::Keypad};

pub mod input;

pub async fn run_diagnostics(
    display: &'_ mut Rotate270<Display<'_, Delay>>,
    keypad: &'_ mut Keypad<'_>,
) {
    loop {
        draw_diagnostics_menu(display).await.unwrap();

        // TODO: select diagnostics like input
    }
}

async fn draw_diagnostics_menu<'a>(
    display: &'_ mut Rotate270<Display<'_, Delay>>,
) -> Result<(), <Rotate270<Display<'a, Delay>> as DrawTarget>::Error> {
    let display_size = display.size();

    // Yellow header background
    Rectangle::new(Point::new(0, 0), Size::new(display_size.width, 30))
        .into_styled(
            PrimitiveStyleBuilder::new()
                .stroke_width(0)
                .fill_color(Rgb565::YELLOW)
                .build(),
        )
        .draw(display)?;

    // Black header font color so it appears inverted on a yellow header.
    let large_character_style = MonoTextStyle::new(&FONT_10X20, Rgb565::BLACK);

    let text_style = TextStyleBuilder::new()
        .alignment(Alignment::Left)
        .line_height(LineHeight::Percent(150))
        .build();

    // "DIAGNOSTICS" header text
    Text::with_text_style(
        "DIAGNOSTICS",
        Point::new(10, 10),
        large_character_style,
        text_style,
    )
    .draw(display)?;

    Ok(())
}
