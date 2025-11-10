use crate::hardware::{display::Display, keypad::Keypad};
use embassy_time::Delay;
use embedded_graphics::{
    pixelcolor::{Rgb565, Rgb888},
    prelude::*,
};
use embedded_graphics_coordinate_transform::Rotate270;

pub async fn input_diagnostics(
    display: &'_ mut Rotate270<Display<'_, Delay>>,
    keypad: &'_ mut Keypad,
) {
    let mut channel = keypad.subscribe().unwrap();

    keypad.clear_leds();

    loop {
        draw_input_diagnostics(display).await;
        display.push_buffer_dma().await.unwrap();

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

            // Turn the keypad LEDs on and off when they're pressed.
            match event {
                crate::hardware::keypad::buttons::Event::KeyPress(key_code) => {
                    if let Ok(button) = key_code.try_into() {
                        keypad.set_led_raw(button, Rgb888::WHITE);
                    }
                }
                crate::hardware::keypad::buttons::Event::KeyRelease(key_code) => {
                    if let Ok(button) = key_code.try_into() {
                        keypad.set_led_raw(button, Rgb888::BLACK);
                    }
                }
                _ => {}
            }
        }

        // Flush the LED buffer all at once so that we're not sending
        // incremental updates repeatedly for each keypress.
        keypad.flush_leds();
    }
}

/// Updates the display with the input diagnostics info.
async fn draw_input_diagnostics(display: &'_ mut Rotate270<Display<'_, Delay>>) {
    display.clear(Rgb565::BLACK);
}
