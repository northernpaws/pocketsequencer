use embassy_executor::Spawner;
use embassy_stm32::usb::Driver;

use crate::engine::midi::{MIDIDestinationReceiver, MIDISourceSender};

mod tasks;

/// Starts the USB component tasks.
pub fn start(
    spawner: Spawner,
    usb_driver: Driver<'static, embassy_stm32::peripherals::USB_OTG_HS>,
    // For sending MIDI events received from the USB endpoint.
    midi_source: MIDISourceSender<'static>,
    // For receiving MIDI events to transmit over the USB endpoint.
    midi_sink: MIDIDestinationReceiver<'static>,
) {
    // Start the USB tasks.
    //
    // This includes the serial port and MIDI device.
    tasks::start_usb_tasks(spawner, usb_driver, midi_source, midi_sink).unwrap();
}
