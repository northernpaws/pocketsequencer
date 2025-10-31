
use defmt::{unwrap, trace, info, error};
use embassy_executor::{SpawnError, Spawner};
use embassy_stm32::{
    Peri, i2c::I2c, mode, peripherals::{self, DMA2_CH4}, timer::{
        self, GeneralInstance4Channel, low_level::CountingMode, simple_pwm::{PwmPin, SimplePwm, SimplePwmChannel}
    }
};
use embassy_embedded_hal::shared_bus::{I2cDeviceError, asynch::{self, i2c}};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel, signal::Signal};
use embassy_sync::{
    blocking_mutex::{
        NoopMutex,
        raw::{
            RawMutex,
            NoopRawMutex
        }
    },
    mutex::Mutex,
    channel::{
        Channel
    }
};


use rgb_led_pwm_dma_maker::{
    calc_dma_buffer_length,
    LedDataComposition,
    LedDmaBuffer,
    RgbLedColor,
    RGB
};

use crate::{hardware::drivers::tca8418::{self, Tca8418}};

use engine::input;


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

static LEDS_UPDATED: Signal<CriticalSectionRawMutex, bool> = Signal::new();

static LED_MAX_BRIGHTNESS: u8 = 5;

const LED_COUNT: usize = 16;

// static LED_MATRIX: [RGB; LED_COUNT] = [
//     RED, GREEN, BLUE, RED,
//     GREEN, BLUE, RED, GREEN,
//     BLUE, RED, GREEN, BLUE,
//     RED, GREEN, BLUE, RED
// ];
static LED_DMA_BUFFER: [u16; DMA_BUFFER_LEN*2] = [0u16; DMA_BUFFER_LEN*2];

pub type KeypadNotifier = Signal<CriticalSectionRawMutex, [RGB; LED_COUNT]>;

pub type ButtonChannel = Channel<CriticalSectionRawMutex, Button, 10>;
pub type ButtonSender<'a> = channel::Sender<'a, CriticalSectionRawMutex, Button, 10>;
pub type ButtonReceiver<'a> = channel::Receiver<'a, CriticalSectionRawMutex, Button, 10>;

#[must_use]
pub const fn notifier() -> KeypadNotifier {
    Signal::new()
}

#[must_use]
pub const fn channel() -> ButtonChannel {
    Channel::new()
}

/// Mapping from keypad button name to it's LED index.
#[repr(usize)]
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

/// Holds all the buttons on the device mapped
/// to the TCA8418RTWR keypad decoder.
/// 
/// ROW4-ROW7 is configured as MENU3-MENU0
/// 
/// ROW0-ROW3 is configured as the matrix rows
/// COL3 - COL3 are configured as columns 3-9 (inverted)
#[repr(u8)]
pub enum Button {
    Unknown = 0,

    // Standalone buttons
    Menu0 = 104,
    Menu1 = 103,
    Menu2 = 102,
    Menu3 = 101,

    // Matrix buttons
    Trig1 = 4,
    Trig2 = 3,
    Trig3 = 2,
    Trig4 = 1,
    Trig5 = 14,
    Trig6 = 13,
    Trig7 = 12,
    Trig8 = 11,
    Trig9 = 24,
    Trig10 = 23,
    Trig11 = 22,
    Trig12 = 21,
    Trig13 = 34,
    Trig14 = 33,
    Trig15 = 32,
    Trig16 = 31,
}

impl Button {
    pub fn led(&self) -> Option<Led> {
        match self {
            Button::Unknown => None,
            Button::Menu0 => None,
            Button::Menu1 => None,
            Button::Menu2 => None,
            Button::Menu3 => None,
            Button::Trig1 => Some(Led::Trig1),
            Button::Trig2 => Some(Led::Trig2),
            Button::Trig3 => Some(Led::Trig3),
            Button::Trig4 => Some(Led::Trig4),
            Button::Trig5 => Some(Led::Trig5),
            Button::Trig6 => Some(Led::Trig6),
            Button::Trig7 => Some(Led::Trig7),
            Button::Trig8 => Some(Led::Trig8),
            Button::Trig9 => Some(Led::Trig9),
            Button::Trig10 => Some(Led::Trig10),
            Button::Trig11 => Some(Led::Trig11),
            Button::Trig12 => Some(Led::Trig12),
            Button::Trig13 => Some(Led::Trig13),
            Button::Trig14 => Some(Led::Trig14),
            Button::Trig15 => Some(Led::Trig15),
            Button::Trig16 => Some(Led::Trig16),
        }
    }
}

impl TryFrom<u8> for Button {
        type Error = &'static str; // Define an error type for failed conversions

        fn try_from(value: u8) -> Result<Self, Self::Error> {
            match value {
                0 => Ok(Button::Unknown),

                // Standalone buttons
                104=> Ok(Button::Menu0),
                103 => Ok(Button::Menu1),
                102 => Ok(Button::Menu2),
                101 => Ok(Button::Menu3),

                // Matrix buttons
                4 => Ok(Button::Trig1),
                3 => Ok(Button::Trig2),
                2 => Ok(Button::Trig3),
                1 => Ok(Button::Trig4),
                14 => Ok(Button::Trig5),
                13 => Ok(Button::Trig6),
                12 => Ok(Button::Trig7),
                11 => Ok(Button::Trig8),
                24 => Ok(Button::Trig9),
                23 => Ok(Button::Trig10),
                22 => Ok(Button::Trig11),
                21 => Ok(Button::Trig12),
                34 => Ok(Button::Trig13),
                33 => Ok(Button::Trig14),
                32 => Ok(Button::Trig15),
                31 => Ok(Button::Trig16),

                _ => Err("Invalid enum variant for Button"), // Handle invalid values
            }
        }
    }

impl input::Input for Button {}

pub struct Keypad<'a> {
    notifier: &'a KeypadNotifier,
    button_receiver: ButtonReceiver<'a>,

    // percentage from 0-100
    led_max_brightness: u8,

    cache: [RGB; LED_COUNT]
}

#[derive(Debug)]
pub enum InitError {
    /// Occurs if there was an issue communicating with one of the I2C
    /// devices that makes up the keypad, I.e. the key matrix controller.
    I2CError(I2cDeviceError<embassy_stm32::i2c::Error>),
    /// Occurs if there was an error spawning one of the keypad-related tasks.
    SpawnError(SpawnError),
}

impl From<I2cDeviceError<embassy_stm32::i2c::Error>> for InitError {
    fn from(value: I2cDeviceError<embassy_stm32::i2c::Error>) -> Self {
        Self::I2CError(value)
    }
}

impl From<SpawnError> for InitError {
    fn from(value: SpawnError) -> Self {
        Self::SpawnError(value)
    }
}

impl<'a> Keypad<'a> {
    pub async fn new (
        notifier: &'static KeypadNotifier,
        button_channel: &'static ButtonChannel,
        mut driver: Tca8418<'static, asynch::i2c::I2cDevice<'static, CriticalSectionRawMutex, embassy_stm32::i2c::I2c<'static, mode::Async, embassy_stm32::i2c::Master>>>,
        mut led_pwm: SimplePwm<'static, peripherals::TIM5>,
        led_dma: Peri<'static, embassy_stm32::peripherals::DMA2_CH4>,
        channel: timer::Channel,
        spawner: Spawner,
    ) -> Result<Self, InitError> {
        // Trigger and inital refresh when the driver starts.
        notifier.signal([
            RED, GREEN, BLUE, RED,
            GREEN, BLUE, RED, GREEN,
            BLUE, RED, GREEN, BLUE,
            RED, GREEN, BLUE, RED]);

        info!("Initializing TCA8418 driver...");
        driver.init().await.unwrap(); // TODO : return error result

        trace!("Configuring TCA8418");

        driver.write_register(tca8418::register::Configuration::new(
            false, // auto increment for read-write operations
            false, // GPI evens tracked when keypad locked
            true, // overflow data shifts with last event pushing first event out
            true, //  processor interrupt is deasserted for 50 μs and reassert with pending interrupts
            true, // assert INT on overflow
            false, // don't assert interrupt after keypad unlock
            false, // assert INT for GPI events // TODO: change
            true, // assert INT for keypad events
        )).await?;
        
        driver.write_register_byte(tca8418::register::GPIO_INTERRUPT_ENABLE1_ADDRESS, 0x00).await?;
        driver.write_register_byte(tca8418::register::GPIO_INTERRUPT_ENABLE2_ADDRESS, 0x00).await?;
        driver.write_register_byte(tca8418::register::GPIO_INTERRUPT_ENABLE3_ADDRESS, 0x00).await?;

        // Enable FIFO for the MENU GPIO buttons.
        driver.write_register_byte(tca8418::register::GPI_EVENT_MODE1_ADDRESS, 0b11110000).await?;

        // Enable rows 0-3
        driver.write_register_byte(tca8418::register::KEYPAD_GPIO1_ADDRESS, 0b00001111).await?;
        // Enable colums 0-3
        driver.write_register_byte(tca8418::register::KEYPAD_GPIO2_ADDRESS, 0b00001111).await?;
        driver.write_register_byte(tca8418::register::KEYPAD_GPIO3_ADDRESS, 0b00000000).await?;

        // Split the button event channel since
        // the task only needs a channel sender.
        let button_receiver = button_channel.receiver();
        let button_sender = button_channel.sender();

        // Spawn the tasks for handling the keypad events and LEDs.
        info!("Spawning keypad tasks...");
        spawner.spawn(unwrap!(buttons_event_task(driver.receiver(), button_sender)));
        spawner.spawn(unwrap!(leds_task(notifier, led_pwm, channel, led_dma)));
        spawner.spawn(unwrap!(buttons_task(driver)));

        Ok(Self {
            notifier,
            button_receiver,

            led_max_brightness: 1,

            cache: [RGB::new(0, 0, 0); LED_COUNT]
        })
    }

    /// Updates the entire LED matrix.
    pub fn set_leds(&mut self, leds: [RGB; LED_COUNT]) {
        self.cache = leds;
        self.notifier.signal(leds);
    }

    /// Updates a single LED in the matrix.
    pub fn set_led(&mut self, led: Led, color: RGB) {
        self.cache[led as usize] = color;
        self.notifier.signal(self.cache);
    }
}

/// Implement an input driver with the keypad for
/// contexts where the inputs are keypad buttons.
impl<'a> input::Driver<Button> for Keypad<'a> {
    async fn poll(&mut self) -> Result<input::InputEvent<Button>, ()> {
        // Wait to receive a button event on the input channel.
        let button = self.button_receiver.receive().await;

        // Send it out as an input event.
        Ok(input::InputEvent::<Button>{
            input: button,
            pressed: true
        })
    }
}

/// Task that listens for button events from the TCA8418, maps
/// the key event code to a hardware button, and then emits a
/// button press event on a channel.
#[embassy_executor::task]
pub async fn buttons_event_task(
    mut driver_channel: tca8418::KeyChannelReceiver<'static>,
    mut button_channel: ButtonSender<'static>,
) -> ! {
    trace!("Starting keypad button event task...");
    loop {
        // Wait to receive a key event from the TCA8418.
        let event = driver_channel.receive().await;

        // Convert the key event into a known button on the hardware.
        if let Ok(button) = Button::try_from(event.key) {
            button_channel.send(button);
        }
    }
}

/// Event that runs the TCA8418's interrupt handler.
#[embassy_executor::task]
pub async fn buttons_task(
    mut driver: Tca8418<'static, asynch::i2c::I2cDevice<'static, CriticalSectionRawMutex, embassy_stm32::i2c::I2c<'static, mode::Async, embassy_stm32::i2c::Master>>>,
) -> ! {
    trace!("Starting keypad button matrix interrupt processor...");
    loop {
        // Wait for an IRQ event and process it.
        driver.process().await;
    }
}

/// Event that listens for LED update signals and clocks
/// them out to the LEDs over the PWM signal.
#[embassy_executor::task]
pub async fn leds_task(
    notifier: &'static KeypadNotifier,
    mut led_pwm: SimplePwm<'static, peripherals::TIM5>,
    channel: timer::Channel,
    // led_channel: SimplePwmChannel<'a, TIM>,
    mut led_dma: Peri<'static, embassy_stm32::peripherals::DMA2_CH4>,
) -> ! {
    // Enable the timer channel for the PWM pin.
    led_pwm.channel(channel).enable();

    let led_max_duty = led_pwm.channel(channel).max_duty_cycle();
    trace!("LED Max Duty Cycle: {}", led_max_duty);

    // t1h = T1H / data_transfer_time * max_duty_cycle = 0.6us / 1.25us * 300 = 144
    // const T1H: u16 = 144; // 144;
    // // t0h = T0H / data_transfer_time * max_duty_cycle = 0.3us / 1.25us * 300 = 72
    // const T0H: u16 = 72; // 72;

    // t1h = T1H / data_transfer_time * max_duty_cycle = 0.6us / 1.25us * 300 = 144
    const T1H: f32 = 0.6; // 144;
    // t0h = T0H / data_transfer_time * max_duty_cycle = 0.3us / 1.25us * 300 = 72
    const T0H: f32 = 0.3; // 72;

    const DATA_TRANSFER_TIME: f32 = 1.25;

    let t1h: u16 = (T1H / DATA_TRANSFER_TIME * (led_max_duty) as f32) as u16;
    let t0h: u16 = (T0H / DATA_TRANSFER_TIME * (led_max_duty) as f32) as u16;

    // Create a DMA buffer for the led strip.
    //
    // From datasheet, data structure of 24 bit data is green -> red -> blue, so use LedDataComposition::GRB
    //
    // T1H and T0H define the PWM duty cycle width for a 1 and 0 respectively.
    let mut dma_buffer = LedDmaBuffer::<DMA_BUFFER_LEN>::new(t1h, t0h, LedDataComposition::GRB);
    
    // ref: https://github.com/embassy-rs/embassy/issues/4788
    let mut u32_dma_buffer: [u16; DMA_BUFFER_LEN*2] = [0u16; DMA_BUFFER_LEN*2];

    loop {
        // Wait for an LED update before refreshing the LEDs.
        let matrix = notifier.wait().await;

        trace!("LED refresh");

        // Convert the RGB array into the 24-bit duty cycles
        // needed to clock out the colors over PWM. 
        dma_buffer.set_dma_buffer_with_brightness(&matrix, None, LED_MAX_BRIGHTNESS).unwrap();

        // Hack to fix DMA transfers.
        //
        // This is really inefficiant and we should move to implementing
        // our own `set_dma_buffer_with_brightness` method that includes
        // the required stride from the start.
        //
        // ref: https://github.com/embassy-rs/embassy/issues/4788
        let mut i = 0;
        for d in dma_buffer.get_dma_buffer() {
            u32_dma_buffer[i*2] = *d;
            i += 1;
        }

        // Block out the PWM waveform using the DMA buffer
        led_pwm.waveform::<embassy_stm32::timer::Ch4>(led_dma.reborrow(), &u32_dma_buffer).await;
    }
}
