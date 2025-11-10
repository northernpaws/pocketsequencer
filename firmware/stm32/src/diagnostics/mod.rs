use embassy_time::Delay;
use embedded_graphics::{
    mono_font::{MonoTextStyle, ascii::FONT_9X18_BOLD},
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{PrimitiveStyleBuilder, Rectangle},
    text::{Alignment, LineHeight, Text, TextStyleBuilder},
};
use embedded_graphics_coordinate_transform::Rotate270;

use crate::hardware::{display::Display, keypad::Keypad};

pub mod i2c;
pub mod input;

pub async fn run_diagnostics(
    display: &'_ mut Rotate270<Display<'_, Delay>>,
    keypad: &'_ mut Keypad,
) {
    let mut channel = keypad.subscribe().unwrap();

    loop {
        draw_diagnostics_menu(display).await.unwrap();
        display.push_buffer_dma().await.unwrap();
        // TODO: select diagnostics like input

        loop {
            // Immediately process any button events in the keypad channel.
            let Some(message) = channel.try_next_message() else {
                break;
            };

            use embassy_sync::pubsub::WaitResult::Message;

            let Message(event): embassy_sync::pubsub::WaitResult<
                crate::hardware::keypad::buttons::Event,
            > = message
            else {
                continue;
            };

            // match event {
            //     crate::hardware::keypad::buttons::Event::FIFOCleared(_) => todo!(),
            //     crate::hardware::keypad::buttons::Event::ButtonPress(_) => todo!(),
            //     crate::hardware::keypad::buttons::Event::ButtonRelease(button) => match button {
            //         Button::Menu0 => todo!(),
            //         Button::Menu1 => todo!(),
            //         Button::Menu2 => todo!(),
            //         Button::Menu3 => todo!(),
            //         Button::Trig1 => todo!(),
            //         Button::Trig2 => todo!(),
            //         Button::Trig3 => todo!(),
            //         Button::Trig4 => todo!(),
            //         Button::Trig5 => todo!(),
            //         Button::Trig6 => todo!(),
            //         Button::Trig7 => todo!(),
            //         Button::Trig8 => todo!(),
            //         Button::Trig9 => todo!(),
            //         Button::Trig10 => todo!(),
            //         Button::Trig11 => todo!(),
            //         Button::Trig12 => todo!(),
            //         Button::Trig13 => todo!(),
            //         Button::Trig14 => todo!(),
            //         Button::Trig15 => todo!(),
            //         Button::Trig16 => todo!(),
            //     },
            // }
        }
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
    let large_character_style = MonoTextStyle::new(&FONT_9X18_BOLD, Rgb565::BLACK);

    let text_style = TextStyleBuilder::new()
        .alignment(Alignment::Left)
        .line_height(LineHeight::Percent(150))
        .build();

    // "DIAGNOSTICS" header text
    Text::with_text_style(
        "DIAGNOSTICS",
        Point::new(10, 19),
        large_character_style,
        text_style,
    )
    .draw(display)?;

    Ok(())
}
