use embassy_executor::Spawner;
use embassy_stm32::usb::Driver;

use crate::engine::midi::MIDIEndpoint;

mod tasks;

/// Starts the USB component tasks.
pub fn start(
    spawner: Spawner,
    usb_driver: Driver<'static, embassy_stm32::peripherals::USB_OTG_HS>,
    midi_endpoint: MIDIEndpoint,
) {
    // Start the USB tasks.
    //
    // This includes the serial port and MIDI device.
    tasks::start_usb_tasks(spawner, usb_driver, midi_endpoint).unwrap();
}
