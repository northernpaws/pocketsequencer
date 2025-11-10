pub mod dma;

use defmt::trace;

use embassy_executor::{SpawnError, Spawner};
use embassy_stm32::{
    Peri,
    peripherals::{DMA2_CH4, TIM5},
    timer::{self, simple_pwm::SimplePwm},
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};

use embedded_graphics::pixelcolor::Rgb888;

use crate::hardware::keypad::lights::dma::{Buffer, calc_dma_buffer_length};

#[repr(u8)]
pub enum Led {
    Trig1 = 0,
    Trig2 = 1,
    Trig3 = 2,
    Trig4 = 3,
    Trig5 = 7,
    Trig6 = 6,
    Trig7 = 5,
    Trig8 = 4,
    Trig9 = 8,
    Trig10 = 9,
    Trig11 = 10,
    Trig12 = 11,
    Trig13 = 15,
    Trig14 = 14,
    Trig15 = 13,
    Trig16 = 12,
}

/// Count of the LEDs in the matrix as a constant.
pub const LED_COUNT: usize = 16;

/// Signal used to propagate updates to the keypad LEDs to the LED PWM management task.
pub type Notifier = Signal<CriticalSectionRawMutex, [Rgb888; LED_COUNT]>;

/// Constructs a new LightNotifier which serves as the bridge between the
/// LED PWM task and other tasks that want to update the LED states.
#[must_use]
pub const fn notifier() -> Notifier {
    Signal::new()
}

/// A convenience wrapper around the syncronization
/// signals used for managing the keypad.
pub struct KeypadLights {
    notifier: &'static Notifier,
}

impl KeypadLights {
    /// Creates a new keypad light manager.
    ///
    /// This spawns the light updater task that processes
    /// LED updates from the provided notifier.
    #[must_use = "Must be used to manage the spawned task"]
    pub fn new(
        pwm: SimplePwm<'static, TIM5>,
        channel: timer::Channel,
        dma: Peri<'static, DMA2_CH4>,
        notifier: &'static Notifier,
        spawner: Spawner,
    ) -> Result<Self, SpawnError> {
        spawner.spawn(device_loop(pwm, channel, dma, notifier)?);

        Ok(Self { notifier })
    }

    /// Creates a sender-only keypad light manager
    /// that doesn't spawn the LED management task.
    pub fn sender(notifier: &'static Notifier) -> Self {
        Self { notifier }
    }

    /// Sets the lights to the specified value.
    pub fn set_lights(&mut self, leds: [Rgb888; LED_COUNT]) {
        self.notifier.signal(leds);
    }
}

#[embassy_executor::task]
async fn device_loop(
    pwm: SimplePwm<'static, TIM5>,
    channel: timer::Channel,
    dma: Peri<'static, DMA2_CH4>,
    notifier: &'static Notifier,
) -> ! {
    // should never return
    let err = inner_device_loop(pwm, channel, dma, notifier).await;
    panic!("{:?}", err);
}

#[derive(Debug)]
enum Never {}

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

// Calculate the length of the buffer required for the PWM data.
const BUFFER_LENGTH: usize = calc_dma_buffer_length(32, LED_COUNT, RESET_LENGTH);

async fn inner_device_loop(
    mut pwm: SimplePwm<'static, TIM5>,
    channel: timer::Channel,
    mut dma: Peri<'static, DMA2_CH4>,
    notifier: &'static Notifier,
) -> Result<(), Never> {
    // Enable the timer channel for the PWM pin.
    pwm.channel(channel).enable();

    let led_max_duty = pwm.max_duty_cycle();
    trace!("LED Max Duty Cycle: {}", led_max_duty);

    // Values from the datasheet used to calculate the PWM period.
    const T1H: f32 = 0.6; // us;
    const T0H: f32 = 0.3; // us;
    const DATA_TRANSFER_TIME: f32 = 1.25; // us

    // Calculate the t0h and t1h periods used to signal 0 and 1 via PWM duty at realtime to prevent
    // having to re-calculate them if the PLL changes the timer clock's max duty rate.
    let t1h: u16 = (T1H / DATA_TRANSFER_TIME * (led_max_duty) as f32) as u16; // 144
    let t0h: u16 = (T0H / DATA_TRANSFER_TIME * (led_max_duty) as f32) as u16; // 70

    // Create a DMA buffer for the led strip.
    //
    // From datasheet, data structure of 24 bit data is green -> red -> blue, so use LedDataComposition::GRB
    //
    // T1H and T0H define the PWM duty cycle width for a 1 and 0 respectively.
    //
    // NOTE: The LED data is 24-bit GRB encoded, which is internally represented as a u32. The PWM timer used
    // on this initial prototype is also a 32-bit timer, and feeding it with u16 data breaks the composition
    // of the outputted signal, see: https://github.com/embassy-rs/embassy/issues/4788
    let mut dma_buffer: Buffer<BUFFER_LENGTH> = Buffer::new(t1h, t0h);

    loop {
        // Wait for an LED update before refreshing the LEDs.
        let led_array = notifier.wait().await;

        // Convert the RGB array into the 24-bit duty cycles
        // needed to clock out the colors over PWM.
        //
        // Note that there is 1-byte padding between each item.
        //
        // Really shouldn't be required, but it is due to how Embassy's
        // waveform method was typed and using a 32-bit timer.
        // see: https://github.com/embassy-rs/embassy/issues/4788
        for (led_index, led) in led_array.iter().enumerate() {
            dma_buffer.set_color(led_index, led);
        }

        // Block out the PWM waveform using the DMA buffer.
        //
        // Note: The PWM timer used on this initial prototype uses a 32-bit timer, and
        // feeding it with single u16 half-words breaks the composition of the output
        // PWM signal without extra padding to make the 32-bit timer happy.
        //
        // see: https://github.com/embassy-rs/embassy/issues/4788
        pwm.waveform::<embassy_stm32::timer::Ch4>(dma.reborrow(), dma_buffer.get_dma_buffer())
            .await;
    }
}
