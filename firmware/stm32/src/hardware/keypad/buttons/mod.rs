use core::u16;

use defmt::{error, info};
use derive_more::{Display, Error};
use embassy_embedded_hal::shared_bus::{I2cDeviceError, asynch::i2c::I2cDevice};
use embassy_executor::{SpawnError, Spawner};
use embassy_futures::select::{Either, select};
use embassy_stm32::{
    exti::ExtiInput,
    i2c::{I2c, Master},
    mode,
};
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    pubsub::{self, PubSubChannel},
    signal::Signal,
};

use crate::hardware::drivers::tca8418::{self, Tca8418};

#[derive(Debug, Clone, PartialEq, defmt::Format)]
#[repr(u8)]
pub enum KeyCode {
    Unknown = 0,

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

impl From<u8> for KeyCode {
    fn from(value: u8) -> Self {
        match value {
            104 => KeyCode::Menu0,
            103 => KeyCode::Menu1,
            102 => KeyCode::Menu2,
            101 => KeyCode::Menu3,

            // Matrix buttons
            4 => KeyCode::Trig1,
            3 => KeyCode::Trig2,
            2 => KeyCode::Trig3,
            1 => KeyCode::Trig4,
            14 => KeyCode::Trig5,
            13 => KeyCode::Trig6,
            12 => KeyCode::Trig7,
            11 => KeyCode::Trig8,
            24 => KeyCode::Trig9,
            23 => KeyCode::Trig10,
            22 => KeyCode::Trig11,
            21 => KeyCode::Trig12,
            34 => KeyCode::Trig13,
            33 => KeyCode::Trig14,
            32 => KeyCode::Trig15,
            31 => KeyCode::Trig16,

            _ => KeyCode::Unknown,
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum Event {
    FIFOCleared(u16),
    KeyPress(KeyCode),
    KeyRelease(KeyCode),
}

/// Notifies the button task manually to read the FIFO for button events.
pub type Notifier = Signal<CriticalSectionRawMutex, u16>;

/// Notifiers consumers that the button matrix has been scanned/updated.
///
/// We set the capacity to more then the TCA8418's FIFO capacity of 10 to
/// ensure we have enough space if we're not processing events as fast as
/// we're scanning them.
///
/// The publisher is set to 1 because only the keypad button manager should be a publisher.
pub type Watcher = PubSubChannel<CriticalSectionRawMutex, Event, 20, 5, 1>;
pub type WatcherSender<'a> = pubsub::Publisher<'a, CriticalSectionRawMutex, Event, 20, 5, 1>;
pub type WatcherReceiver<'a> = pubsub::Subscriber<'a, CriticalSectionRawMutex, Event, 20, 5, 1>;

#[must_use]
pub const fn notifier() -> Notifier {
    Signal::new()
}

#[must_use]
pub const fn watcher() -> Watcher {
    PubSubChannel::new()
}

/// Errors the happen when initializing the keypad button task.
#[derive(Debug, Display, Error)]
pub enum InitError {
    /// Occurs if there was an issue communicating with one of the I2C
    /// devices that makes up the keypad, I.e. the key matrix controller.
    #[display("{_0:?}")]
    I2CError(#[error(not(source))] I2cDeviceError<embassy_stm32::i2c::Error>),

    /// Occurs if there was an error spawning one of the keypad-related tasks.
    #[display("{_0:?}")]
    SpawnError(#[error(not(source))] SpawnError),
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

/// Wrapper for managing the TCA9314 that matrixes the keypad buttons.
pub struct KeypadButtons {
    notifier: &'static Notifier,
    channel: &'static Watcher,
    subscriber: WatcherReceiver<'static>,
}

impl KeypadButtons {
    pub async fn new(
        mut driver: Tca8418<
            'static,
            I2cDevice<'static, CriticalSectionRawMutex, I2c<'static, mode::Async, Master>>,
        >,
        interrupt: ExtiInput<'static>,
        notifier: &'static Notifier,
        channel: &'static Watcher,
        spawner: Spawner,
    ) -> Result<Self, InitError> {
        // First call any initialization routines needed by the TCA8418 driver.
        driver.init().await.unwrap(); // TODO : return error result

        // Next, we configure the TAC8418 driver with our key matrix and GPIO mappings.
        //
        // Note that on an MCU reset the TCA8418 might be running with the previous config
        // and not the default register values, so need to push the entire config again.

        driver
            .write_register(tca8418::register::Configuration::new(
                false, // disable auto increment for write operations
                false, // GPI events tracked when keypad locked
                true,  // overflow data shifts with last event pushing first event out
                true, //  processor interrupt is deasserted for 50 μs and reassert with pending interrupts
                true, // assert INT on overflow
                false, // don't assert interrupt after keypad unlock
                false, // assert INT for GPI events // TODO: change
                true, // assert INT for keypad events
            ))
            .await?;

        driver
            .write_register_byte(tca8418::register::GPIO_INTERRUPT_ENABLE1_ADDRESS, 0x00)
            .await?;
        driver
            .write_register_byte(tca8418::register::GPIO_INTERRUPT_ENABLE2_ADDRESS, 0x00)
            .await?;
        driver
            .write_register_byte(tca8418::register::GPIO_INTERRUPT_ENABLE3_ADDRESS, 0x00)
            .await?;

        // Enable FIFO for the MENU GPIO buttons connected to rows 4-8.
        driver
            .write_register_byte(tca8418::register::GPI_EVENT_MODE1_ADDRESS, 0b11110000)
            .await?;

        // Enable rows 0-3 in the button matrix.
        driver
            .write_register_byte(tca8418::register::KEYPAD_GPIO1_ADDRESS, 0b00001111)
            .await?;

        // Enable columns 0-3 in the button matrix.
        driver
            .write_register_byte(tca8418::register::KEYPAD_GPIO2_ADDRESS, 0b00001111)
            .await?;
        driver
            .write_register_byte(tca8418::register::KEYPAD_GPIO3_ADDRESS, 0b00000000)
            .await?;

        // At this point, with the columns and rows enabled, the TCA8418
        // enters idle scanning mode waiting for keypresses.

        // Spawn the task that polls the TCA8418's
        // interrupt line and scans the keypad.
        spawner.spawn(device_loop(
            driver,
            interrupt,
            notifier,
            channel.publisher().expect(
                "There should be enough publisher channels for the keypad button controller",
            ),
        )?);

        Ok(Self {
            notifier,
            channel,
            subscriber: channel.subscriber().expect("Failed to get a subscriber"),
        })
    }

    /// Returns a pubsub channel subscriber for keypress events.
    pub fn subscribe<'a>(&self) -> Result<WatcherReceiver<'a>, pubsub::Error> {
        self.channel.subscriber()
    }

    /// Immediately polls and processes the TCA8418 FIFO until it's clear.
    ///
    /// Primarily used to immediately ensure all pending button events have
    /// been handled, such as during a state transition or during the boot
    /// process.
    pub async fn flush(&mut self) {
        // Signal to the button processing task
        // that we want an immediate scan.
        self.notifier.signal(u16::MAX);

        // Wait for the scan to finish so we
        // know the FIFO has been flushed.
        //
        // Also check that the ticket in the FIFO cleared event
        // matches the one we issued. This is important to ensure
        // that the events up to the point the flush was called
        // have been processed and we're not receiving an earlier
        // FIFO event.
        loop {
            let message = self.subscriber.next_message().await;

            match message {
                pubsub::WaitResult::Lagged(_) => {
                    error!("Keypad pubsub lagged");
                }
                pubsub::WaitResult::Message(event) => match event {
                    Event::FIFOCleared(ticket) => {
                        if ticket == u16::MAX {
                            return;
                        }
                    }
                    _ => {
                        continue;
                    }
                },
            }
        }
    }
}

#[embassy_executor::task]
async fn device_loop(
    driver: Tca8418<
        'static,
        I2cDevice<'static, CriticalSectionRawMutex, I2c<'static, mode::Async, Master>>,
    >,
    interrupt: ExtiInput<'static>,
    notifier: &'static Notifier,
    publisher: WatcherSender<'static>,
) -> ! {
    // should never return
    let err = inner_device_loop(driver, interrupt, notifier, publisher).await;
    panic!("{:?}", err);
}

#[derive(Debug)]
enum Never {}

async fn inner_device_loop(
    mut driver: Tca8418<
        'static,
        I2cDevice<'static, CriticalSectionRawMutex, I2c<'static, mode::Async, Master>>,
    >,
    mut interrupt: ExtiInput<'static>,
    notifier: &'static Notifier,
    publisher: WatcherSender<'static>,
) -> Result<(), Never> {
    loop {
        let mut ticket: u16 = 0;
        let mut clear_interrupts = false;

        // If the interrupt pin is high at the start of the loop
        // then we need to immediately try to process the FIFO.
        if !interrupt.is_high() {
            // Wait for either a manual refresh signal, or
            // for the TCA8418 to raise an interrupt.
            let event = select(interrupt.wait_for_falling_edge(), notifier.wait()).await;

            // If the event was manually triggered, grab the syncronization ticket.
            ticket = match event {
                Either::First(_) => 0,
                Either::Second(ticket) => ticket,
            };

            if let Either::First(_) = event {
                clear_interrupts = true;
            }
        } else {
            clear_interrupts = true;
            info!("Interrupt line was already high, uncleared FIFO from previous loop?")
        }

        loop {
            // Poll the FIFO for the key event currently at it's head.
            let Ok(result) = driver.poll_fifo().await else {
                error!("I2C communication error polling TCA8418 for events!");
                break;
            };

            // If there was no event then the FIFO is empty.
            let Some(mut event) = result else {
                break;
            };

            let keycode: KeyCode = event.key_index().get().value().into();

            // Send out the button press events.
            if event.key_pressed().get() {
                publisher.publish_immediate(Event::KeyPress(keycode));
            } else {
                publisher.publish_immediate(Event::KeyRelease(keycode));
            }
        }

        // Send out a special event signally that we cleared the FIFO. This is
        // used by some methods to ensure the FIFO is flushed and up-to-date.
        //
        // If the scan was manually triggered we also include the ticket supplied
        // by the manual scan request so that the caller can corrolate the FIFO
        // event with it's request.
        publisher.publish_immediate(Event::FIFOCleared(ticket));

        if clear_interrupts {
            // If the condition was triggered by an interrupt
            // then we need to clear the interrupt status table
            // after clearning the button FIFO.
            driver.clear_all_interrupts().await.unwrap();
        }
    }
}
