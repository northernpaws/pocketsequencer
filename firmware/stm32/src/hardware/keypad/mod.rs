pub mod buttons;
pub mod lights;

use derive_more::{Display, From};

use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_executor::{SpawnError, Spawner};
use embassy_stm32::{
    Peri,
    exti::ExtiInput,
    i2c::{I2c, Master},
    mode,
    peripherals::{DMA2_CH4, TIM5},
    timer::{self, simple_pwm::SimplePwm},
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, pubsub};

use embedded_graphics::{pixelcolor::Rgb888, prelude::RgbColor};

use crate::hardware::{
    drivers::tca8418::Tca8418,
    keypad::{
        buttons::{KeyCode, KeypadButtons, Watcher},
        lights::KeypadLights,
    },
};

/// Represents a keypad button.
#[derive(Debug, Clone, PartialEq, defmt::Format)]
pub enum Button {
    // Standalone buttons
    Menu0,
    Menu1,
    Menu2,
    Menu3,

    // Matrix buttons
    Trig1,
    Trig2,
    Trig3,
    Trig4,
    Trig5,
    Trig6,
    Trig7,
    Trig8,
    Trig9,
    Trig10,
    Trig11,
    Trig12,
    Trig13,
    Trig14,
    Trig15,
    Trig16,
}

impl Button {
    /// Converts a button to it's index in the LED matrix.
    ///
    /// Returns `None` for the menu buttons that don't have an associated LED.
    pub const fn led(&self) -> Option<usize> {
        match self {
            Button::Trig1 => Some(0),
            Button::Trig2 => Some(1),
            Button::Trig3 => Some(2),
            Button::Trig4 => Some(3),
            Button::Trig5 => Some(7),
            Button::Trig6 => Some(6),
            Button::Trig7 => Some(5),
            Button::Trig8 => Some(4),
            Button::Trig9 => Some(8),
            Button::Trig10 => Some(9),
            Button::Trig11 => Some(10),
            Button::Trig12 => Some(11),
            Button::Trig13 => Some(15),
            Button::Trig14 => Some(14),
            Button::Trig15 => Some(13),
            Button::Trig16 => Some(12),
            _ => None,
        }
    }

    /// Returns the keycode index matching this button to a key event.
    pub const fn keycode(&self) -> KeyCode {
        match self {
            // Standalone buttons
            Button::Menu0 => KeyCode::Menu0,
            Button::Menu1 => KeyCode::Menu1,
            Button::Menu2 => KeyCode::Menu2,
            Button::Menu3 => KeyCode::Menu3,

            // Matrix buttons
            Button::Trig1 => KeyCode::Trig1,
            Button::Trig2 => KeyCode::Trig2,
            Button::Trig3 => KeyCode::Trig3,
            Button::Trig4 => KeyCode::Trig4,
            Button::Trig5 => KeyCode::Trig5,
            Button::Trig6 => KeyCode::Trig6,
            Button::Trig7 => KeyCode::Trig7,
            Button::Trig8 => KeyCode::Trig8,
            Button::Trig9 => KeyCode::Trig9,
            Button::Trig10 => KeyCode::Trig10,
            Button::Trig11 => KeyCode::Trig11,
            Button::Trig12 => KeyCode::Trig12,
            Button::Trig13 => KeyCode::Trig13,
            Button::Trig14 => KeyCode::Trig14,
            Button::Trig15 => KeyCode::Trig15,
            Button::Trig16 => KeyCode::Trig16,
        }
    }
}

impl Into<KeyCode> for Button {
    fn into(self) -> KeyCode {
        self.keycode()
    }
}

/// Signal used to propagate updates to the keypad components.
pub type Notifier = (buttons::Notifier, lights::Notifier);

/// Constructs a new LightNotifier which serves as the bridge between the
/// LED PWM task and other tasks that want to update the LED states.
#[must_use]
pub const fn notifier() -> Notifier {
    (buttons::notifier(), lights::notifier())
}

#[derive(Debug, From, Display, derive_more::Error)]
pub enum Error {
    /// Indicates there was an error spawning a child task.
    #[display("{_0:?}")]
    SpawnError(#[error(not(source))] SpawnError),

    /// Indicates that there was an error initializing the button matrix handling routines.
    ButtonInitError(buttons::InitError),
}

/// A convenience wrapper around the syncronization
/// signals used for managing the keypad.
pub struct Keypad {
    notifier: &'static Notifier,

    /// Inner layer for managing the keypad LED matrix.
    lights: lights::KeypadLights,
    buttons: buttons::KeypadButtons,

    leds: [Rgb888; lights::LED_COUNT],
}

impl Keypad {
    #[must_use = "Must be used to manage the spawned tasks"]
    pub async fn new(
        pwm: SimplePwm<'static, TIM5>,
        channel: timer::Channel,
        dma: Peri<'static, DMA2_CH4>,
        notifier: &'static Notifier,
        watcher: &'static Watcher,
        driver: Tca8418<
            'static,
            I2cDevice<'static, CriticalSectionRawMutex, I2c<'static, mode::Async, Master>>,
        >,
        interrupt: ExtiInput<'static>,
        spawner: Spawner,
    ) -> Result<Self, Error> {
        // Construct the inner wrapper that manages the keypad LED's PWM signal.
        let lights = KeypadLights::new(pwm, channel, dma, &notifier.1, spawner)?;

        // Construct the inner wrapper that manages the TCA8418 keypad matrix driver.
        let buttons = KeypadButtons::new(driver, interrupt, &notifier.0, watcher, spawner).await?;

        Ok(Self {
            notifier,
            lights,
            buttons,
            leds: [Rgb888::BLACK; lights::LED_COUNT],
        })
    }

    /// Creates a derrived wrapper around the light notifier
    /// that can be used to update the keypad lights.
    ///
    /// This should be used sparingly. The primary intention
    /// is to allow some internal tasks that need to reset the
    /// light state (i.e. on power down) to do so.
    pub fn lights(&mut self) -> KeypadLights {
        KeypadLights::sender(&self.notifier.1)
    }

    /// Clears all the LEDS to black.
    pub fn clear_leds(&mut self) {
        for led in &mut self.leds {
            *led = Rgb888::BLACK;
        }

        self.notifier.1.signal(self.leds);
    }

    /// Sets all the LEDs in the matrix to a single color.
    pub fn set_leds(&mut self, color: Rgb888) {
        for led in &mut self.leds {
            *led = color;
        }

        self.notifier.1.signal(self.leds);
    }

    /// Sets a single LED to the specified value.
    pub fn set_led(&mut self, button: Button, color: Rgb888) {
        let Some(index) = button.led() else {
            return;
        };

        self.leds[index] = color;

        self.notifier.1.signal(self.leds);
    }

    /// Returns a pubsub channel subscriber for keypress events.
    pub fn subscribe<'a>(&self) -> Result<buttons::WatcherReceiver<'a>, pubsub::Error> {
        self.buttons.subscribe()
    }

    /// Queues a button FIFO flush and waits for it to complete.
    ///
    /// This ensures that any pending button events raised by the
    /// TCA8418 prior to this flush call are processed and made
    /// immediately available to the caller.
    pub async fn flush(&mut self) {
        self.buttons.flush().await
    }
}
