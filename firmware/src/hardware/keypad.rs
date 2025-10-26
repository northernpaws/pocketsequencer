
use defmt::trace;
use embassy_stm32::{
    peripherals::DMA2_CH4, timer::{
        self,
        low_level::CountingMode,
        simple_pwm::{PwmPin, SimplePwm, SimplePwmChannel},
        GeneralInstance4Channel
    }, Peri
};

use rgb_led_pwm_dma_maker::{
    calc_dma_buffer_length,
    LedDataComposition,
    LedDmaBuffer,
    RgbLedColor,
    RGB
};

const LED_COUNT: usize = 1;

// APB1 = 120MHz
// doubled for clocks = 240MHz
// max duty cycle = 240MHz/0.800MHz (pwm freq) = 300 cycles

// PWM_FREQ = 1 / data_transfer_time = 1 / 1.25us = 800kHz
// |-------- 1.25us --------|
// 0%         duty       100%
// 0 ->  max duty cycle = 300

// Information needed from datasheet:
// Data Transfer Time = 1.25us
//     * Used to calculate pwm frequency, t1h, and t0h values
// T1H = 0.6us
// T0H = 0.3us
// Reset Period = RES >= 80us

//
// |-------T-------| Symbol Period
// |-T0H-|_________| 0 symbol
// |---T1H----|____| 1 Symbol
// |------- RESET ------|
/// `t1h` = `T1H / data_transfer_time * max_duty_value`
/// `t0h` = `T0H / data_transfer_time * max_duty_value`
/// 
/// `t1h` = `0.6us / 1.25us * 300` = 144
/// `t0h` = `0.3us / 1.25us * 300` = 72
// TODO: should do some of these calculations based off the max duty measurement.
// RESET_LENGTH = reset_period / data_transfer_time = 80us / 1.25us = 64
const RESET_LENGTH: usize = 64; // 64 frames
// Calculate the dma buffer's length at compile time
const DMA_BUFFER_LEN: usize = calc_dma_buffer_length(RGB::BIT_COUNT, LED_COUNT, RESET_LENGTH);

// t1h = T1H / data_transfer_time * max_duty_cycle = 0.6us / 1.25us * 300 = 144
const T1H: u16 = 144; // 144;
// t0h = T0H / data_transfer_time * max_duty_cycle = 0.3us / 1.25us * 300 = 72
const T0H: u16 = 72; // 72;

const RED: RGB = RGB::new(255, 0, 0);
const GREEN: RGB = RGB::new(0, 255, 0);
const BLUE: RGB = RGB::new(0, 0, 255);
const MAGENTA: RGB = RGB::new(255, 0, 255);
const CYAN: RGB = RGB::new(0, 255, 255);
const YELLOW: RGB = RGB::new(255, 255, 0);
const ORANGE: RGB = RGB::new(255, 20, 0);

pub struct Keypad<'a, TIM: GeneralInstance4Channel> {
    led_pwm: SimplePwm<'a, TIM>,
    channel: timer::Channel,
    // led_channel: SimplePwmChannel<'a, TIM>,
    led_dma: Peri<'a, embassy_stm32::peripherals::DMA2_CH4>,
    led_max_duty: u16,
    
    // percentage from 0-100
    led_max_brightness: u8,

    led_matrix: [RGB; LED_COUNT],
    dma_buffer: LedDmaBuffer<DMA_BUFFER_LEN>,
    // ref: https://github.com/embassy-rs/embassy/issues/4788
    u32_dma_buffer: [u16; DMA_BUFFER_LEN*2]
}

impl<'a, TIM: GeneralInstance4Channel> Keypad<'a, TIM>
where
    DMA2_CH4: embassy_stm32::timer::Dma<TIM, embassy_stm32::timer::Ch4> {
    pub fn new (
        mut led_pwm: SimplePwm<'a, TIM>,
        led_dma: Peri<'a, embassy_stm32::peripherals::DMA2_CH4>,
        channel: timer::Channel,
    ) -> Self {
        Self {
            led_pwm,
            channel,
            // led_channel,
            led_dma,

            led_max_duty: 300, // 300 is default for 800kHz to 240MHz

            led_max_brightness: 1,

            led_matrix: [RED],
            // Create a DMA buffer for the led strip
            // From datasheet, data structure of 24 bit data is green -> red -> blue, so use LedDataComposition::GRB
            dma_buffer: LedDmaBuffer::<DMA_BUFFER_LEN>::new(T1H, T0H, LedDataComposition::GRB),
            // ref: https://github.com/embassy-rs/embassy/issues/4788
            u32_dma_buffer: [0u16; DMA_BUFFER_LEN*2]
        }
    }

    pub fn init (&mut self) {
        // Enable the channel corresponding to the PWM channel above.
        // self.led_channel = self.led_pwm.channel(self.channel.index());

        self.led_max_duty = self.led_pwm.channel(self.channel).max_duty_cycle();
        trace!("LED Max Duty Cycle: {}", self.led_max_duty);
    
        // TODO: calculate t0h and t1h here

        // Enable the timer channel for the PWM pin.
        self.led_pwm.channel(self.channel).enable();
    }

    pub fn set_leds(&mut self, leds: [RGB; LED_COUNT]) {
        self.led_matrix = leds;
    }

    pub fn set_led(&mut self, led: u8, color: RGB) {
        assert!(led < LED_COUNT);
        self.led_matrix[led] = color;
    }

    pub async fn refresh_leds (&mut self) {
        self.dma_buffer
            .set_dma_buffer_with_brightness(&self.led_matrix, None, self.led_max_brightness)
            .unwrap();
        let mut i = 0;
        for d in self.dma_buffer.get_dma_buffer() {
            self.u32_dma_buffer[i*2] = *d;
            i += 1;
        }

        // Create a pwm waveform usng the dma buffer
        // pwm.waveform::<embassy_stm32::timer::Ch4>(led_dma.reborrow(), dma_buffer.get_dma_buffer())
        //     .await;
        self.led_pwm.waveform::<embassy_stm32::timer::Ch4>(self.led_dma.reborrow(), &self.u32_dma_buffer)
            .await;
    }
}
