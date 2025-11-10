use embedded_graphics::prelude::RgbColor;
use mousefood::prelude::Rgb888;

/// Calculates the length needed for the DMA buffer.
pub const fn calc_dma_buffer_length(
    bits_per_led: usize,
    led_count: usize,
    reset_length: usize,
) -> usize {
    // 2x to add a zero-padding 1-byte stride to account for DMA issues.
    // see: https://github.com/embassy-rs/embassy/issues/4788
    ((bits_per_led * led_count) + reset_length) * 2
}

/// Convenince wrapper for setting the correct byte encoding.
pub struct Buffer<const DMA_BUFFER_LEN: usize> {
    dma_buffer: [u16; DMA_BUFFER_LEN],
    t1h: u16,
    t0h: u16,
    brightness: u8,
}

impl<const DMA_BUFFER_LEN: usize> Buffer<DMA_BUFFER_LEN> {
    pub fn new(t1h: u16, t0h: u16) -> Self {
        Self {
            dma_buffer: [0u16; DMA_BUFFER_LEN],
            t1h,
            t0h,
            brightness: 100,
        }
    }

    pub fn set_color(&mut self, led_index: usize, color: &Rgb888) {
        let byte_index = led_index * 8 * 3;

        self.set_byte(color.g(), byte_index);
        self.set_byte(color.r(), byte_index + 8);

        // RGB encoding.
        // led_dma_buffer.set_byte(self.r, led_index);
        // led_dma_buffer.set_byte(self.g, led_index + 8);

        self.set_byte(color.b(), byte_index + 16);
    }

    /// Set a byte in the DMA buffer
    pub fn set_byte(&mut self, byte: u8, byte_index: usize) {
        // Scale the value of the byte based on the globally configured brightness value.
        //
        // This allows us to globally adjust the maxmimum brightness of all
        // the LEDs without having to modify the input data individually.
        let adjusted_byte = (f32::from(byte) * f32::from(self.brightness) / 100f32) as u8;

        for i in 0..8 {
            // Stride of two to account for 16-bit to 32-bit DMA issues.
            // see: https://github.com/embassy-rs/embassy/issues/4788
            self.dma_buffer[(i + byte_index) * 2] = if (adjusted_byte & (1 << (7 - i))) > 0 {
                self.t1h
            } else {
                self.t0h
            };
        }
    }

    pub fn get_dma_buffer(&self) -> &[u16] {
        &self.dma_buffer
    }
}
