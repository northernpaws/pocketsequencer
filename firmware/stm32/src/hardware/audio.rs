use embassy_embedded_hal::shared_bus::asynch::{self};
use embassy_stm32::i2c::{self, I2c};
use embassy_stm32::mode;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

pub struct Audio<'a, DELAY: embedded_hal_async::delay::DelayNs> {
    delay: DELAY,
    codec: nau88c22_rs::Nau88c22<
        asynch::i2c::I2cDevice<'a, CriticalSectionRawMutex, I2c<'static, mode::Async, i2c::Master>>,
    >,
}

impl<'a, DELAY: embedded_hal_async::delay::DelayNs> Audio<'a, DELAY> {
    pub fn new(
        device: asynch::i2c::I2cDevice<
            'a,
            CriticalSectionRawMutex,
            I2c<'static, mode::Async, i2c::Master>,
        >,
        delay: DELAY,
    ) -> Self {
        Self {
            delay,
            codec: nau88c22_rs::Nau88c22::new(device),
        }
    }

    /// Initializes the audio component.
    pub async fn init(&self) -> Result<(), i2c::Error> {
        Ok(())
    }
}
