use embassy_time::Delay;
use embedded_graphics::{
    mono_font::{MonoTextStyle, ascii::FONT_6X10, ascii::FONT_9X18_BOLD},
    pixelcolor::{Rgb565, Rgb888},
    prelude::*,
    primitives::{PrimitiveStyleBuilder, Rectangle},
    text::{Alignment, LineHeight, Text, TextStyleBuilder},
};
use embedded_graphics_coordinate_transform::Rotate270;

use crate::hardware::{
    display::Display,
    keypad::{Button, Keypad, buttons::KeyCode},
};

pub mod i2c;
pub mod input;

pub async fn run_diagnostics(
    display: &'_ mut Rotate270<Display<'_, Delay>>,
    keypad: &'_ mut Keypad,
) {
    let mut channel = keypad.subscribe().unwrap();

    // We use *_raw and the flush methods to incrementally update the buffer
    // and push the update all at once instead of repeatedly signaling the
    // LED update task.
    keypad.clear_leds_raw();
    keypad.set_led_raw(Button::Trig1, Rgb888::YELLOW);
    keypad.set_led_raw(Button::Trig2, Rgb888::YELLOW);
    keypad.set_led_raw(Button::Trig3, Rgb888::YELLOW);
    keypad.set_led_raw(Button::Trig4, Rgb888::YELLOW);
    keypad.set_led_raw(Button::Trig5, Rgb888::YELLOW);
    keypad.set_led_raw(Button::Trig6, Rgb888::YELLOW);
    keypad.set_led_raw(Button::Trig7, Rgb888::YELLOW);
    keypad.set_led_raw(Button::Trig8, Rgb888::YELLOW);
    keypad.set_led_raw(Button::Trig13, Rgb888::RED);
    keypad.flush_leds();

    loop {
        draw_diagnostics_menu(display).await.unwrap();
        display.push_buffer_dma().await.unwrap();
        // TODO: select diagnostics like input

        // Process input events.
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

            match event {
                crate::hardware::keypad::buttons::Event::KeyPress(key_code) => {
                    if let Ok(button) = <KeyCode as TryInto<Button>>::try_into(key_code) {
                        match button {
                            Button::Trig1 => input::input_diagnostics(display, keypad).await,

                            // Exit from diagnostics
                            Button::Trig13 => return,

                            _ => {}
                        }
                    }
                }
                _ => {}
            }
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

    let outline_style = PrimitiveStyleBuilder::new()
        .stroke_color(Rgb565::YELLOW)
        .stroke_width(3)
        .build();

    let mut content_area = Rectangle::new(
        Point::new(0, 30),
        Size::new(display_size.width, display_size.height - 31),
    );

    // Add some padding around the outside
    // content_area = content_area.resized(
    //     Size::new(content_area.size.width - 20, content_area.size.height - 20),
    //     embedded_graphics::geometry::AnchorPoint::Center,
    // );

    let box_width = content_area.size.width as i32 / 4;
    let box_height = content_area.size.height as i32 / 2;

    let menu_item_character_style = MonoTextStyle::new(&FONT_6X10, Rgb565::YELLOW);

    let menu_item_text_style = TextStyleBuilder::new().alignment(Alignment::Center).build();

    // Draw the menu content boxes.
    for x in 0..4 {
        for y in 0..2 {
            let box_top_left = Point::new(
                content_area.top_left.x + (box_width * x),
                content_area.top_left.y + (box_height * y),
            );

            Rectangle::new(box_top_left, Size::new(box_width as u32, box_height as u32))
                .into_styled(outline_style)
                .draw(display)?;

            let index = (y * 4) + x;
            if index == 0 {
                Text::with_text_style(
                    "INPUT",
                    box_top_left + Point::new(box_width / 2, box_height - 12),
                    menu_item_character_style,
                    menu_item_text_style,
                )
                .draw(display)?;
            }
        }
    }

    Ok(())
}
