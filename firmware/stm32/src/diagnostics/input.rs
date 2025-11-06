use embassy_time::Delay;
use embedded_graphics_coordinate_transform::Rotate270;

use crate::hardware::{display::Display, keypad::Keypad};

pub async fn input_diagnostics(
    display: &'_ mut Rotate270<Display<'_, Delay>>,
    keypad: &'_ mut Keypad<'_>,
) {
    loop {
        draw_input_diagnostics(display).await;

        // NOTE: We use CPU framebuffer updates as a "safe" option
        // in case there is some sort of problem with the DMA, as
        // we want diagnostics to have the best chance of working.
        display.push_buffer();
    }
}

/// Updates the display with the input diagnostics info.
async fn draw_input_diagnostics(display: &'_ mut Rotate270<Display<'_, Delay>>) {}
