use embassy_time::Delay;
use embedded_graphics_coordinate_transform::Rotate270;

use crate::hardware::{display::Display, keypad::Keypad};

pub async fn input_diagnostics(
    display: &'_ mut Rotate270<Display<'_, Delay>>,
    _keypad: &'_ mut Keypad,
) {
    loop {
        draw_input_diagnostics(display).await;
        display.push_buffer_dma().await;
    }
}

/// Updates the display with the input diagnostics info.
async fn draw_input_diagnostics(_display: &'_ mut Rotate270<Display<'_, Delay>>) {}
