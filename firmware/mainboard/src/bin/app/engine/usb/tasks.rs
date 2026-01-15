use defmt::{error, info};

use embassy_executor::{SpawnError, Spawner};
use embassy_stm32::{
    peripherals::USB_OTG_HS,
    usb::{self, Driver},
};

use embassy_usb::{
    Builder, UsbDevice,
    class::midi::{self, MidiClass},
    driver::EndpointError,
};
use midly::live::LiveEvent;
use static_cell::StaticCell;

use crate::engine::{
    midi::{MIDIDestinationReceiver, MIDIDestinationSender, MIDIEndpoint},
    usb::midi::{decode_midi_message, encode_midi_message, log_event},
};

pub fn start_usb_tasks(
    spawner: Spawner,
    usb_driver: usb::Driver<'static, embassy_stm32::peripherals::USB_OTG_HS>,
    midi_endpoint: MIDIEndpoint,
) -> Result<(), SpawnError> {
    // Create embassy-usb Config
    let mut usb_config = embassy_usb::Config::new(0xc0de, 0xcafe);
    usb_config.manufacturer = Some("Northernpaws");
    usb_config.product = Some("PocketSynth");
    usb_config.serial_number = Some("12345678");

    // Required for windows compatibility.
    // https://developer.nordicsemi.com/nRF_Connect_SDK/doc/1.9.1/kconfig/CONFIG_CDC_ACM_IAD.html#help
    usb_config.device_class = 0xEF;
    usb_config.device_sub_class = 0x02;
    usb_config.device_protocol = 0x01;
    usb_config.composite_with_iads = true;

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    #[unsafe(link_section = ".ram3_d2")]
    static USB_CONFIG_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
    let config_descriptor = USB_CONFIG_DESCRIPTOR.init([0; 256]);

    #[unsafe(link_section = ".ram3_d2")]
    static USB_BOS_DESCIRPTOR: StaticCell<[u8; 256]> = StaticCell::new();
    let bos_descriptor = USB_BOS_DESCIRPTOR.init([0; 256]);

    #[unsafe(link_section = ".ram3_d2")]
    static USB_CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();
    let control_buf = USB_CONTROL_BUF.init([0; 64]);

    let mut builder = Builder::new(
        usb_driver,
        usb_config,
        config_descriptor,
        bos_descriptor,
        &mut [], // no msos descriptors
        control_buf,
    );

    // let mut midi_class: &'static mut MidiClass<'static, Driver<'static, embassy_stm32::peripherals::USB_OTG_HS>> = USB_MIDI_CLASS.init(MidiClass::new(&mut builder, 1, 1, 64));
    let midi_class = MidiClass::new(&mut builder, 1, 1, 64);

    // Split the endpoint channels into:
    //  sink - receiver for sending messages to the MIDI host from the MIDI task
    //  source - sender for receing message from the MIDI host to the MIDI task
    let (midi_sink, midi_source) = midi_endpoint.split();

    // The `MidiClass` can be split into `Sender` and `Receiver`, to be used in separate tasks.
    let (midi_sender, midi_receiver) = midi_class.split();

    // Build the builder.
    let usb = builder.build();

    // TODO: We may want to use an interrupt executor
    //  instead with a very high (low number) priority.

    info!("Spawning USB handler...");
    spawner.spawn(usb_task(usb)?);

    info!("Spawning MIDI class tasks...");
    spawner.spawn(usb_midi_receiver_task(midi_receiver, midi_source)?);
    spawner.spawn(usb_midi_sender_task(midi_sender, midi_sink)?);

    Ok(())
}

/// Task for running the USB peripheral routines.
#[embassy_executor::task]
async fn usb_task(mut usb_device: UsbDevice<'static, Driver<'static, USB_OTG_HS>>) {
    info!("Starting USB handler...");
    usb_device.run().await;
}

/// Task that waits for the USB host to enable the MIDI
/// endpoint, and the starts the MIDI event handler.
#[embassy_executor::task]
async fn usb_midi_receiver_task(
    // Receives MIDI messages from the USB host.
    mut midi_class: midi::Receiver<'static, usb::Driver<'static, USB_OTG_HS>>,
    midi_source: MIDIDestinationSender<'static>,
) {
    info!("Starting MIDI Class receiver handler...");
    loop {
        // Wait for a USB host to acknowledge
        // and enable the MIDI endpoint.
        midi_class.wait_connection().await;
        info!("Connected");

        // Clear the outgoing events channel so events added to
        // the channel before a valid connection are discarded.
        midi_source.clear();

        // Start the MIDI handler for the host.
        let _ = midi_receiver_handler(&mut midi_class, &midi_source).await;
        info!("Disconnected");
    }
}

/// Handler for receiving and processing MIDI events incoming over USB.
async fn midi_receiver_handler<'d>(
    midi_receiver: &mut midi::Receiver<'static, usb::Driver<'static, USB_OTG_HS>>,
    midi_source: &'_ MIDIDestinationSender<'static>,
) -> Result<(), Disconnected> {
    // Buffer for reading MIDI packets from the USB endpoint.
    let mut buf = [0; 64];
    loop {
        let n = midi_receiver.read_packet(&mut buf).await?;
        info!(
            "received MIDI packet {=[u8]:b} {=[u8]:X} {}",
            &buf[..n],
            &buf[..n],
            &buf[..n]
        );

        // Throw away the first byte that's the MIDI Code Index Number (CIN):
        //
        // "The first byte in each 32-bit USB-MIDI Event Packet is a Packet Header
        //  contains a Cable Number (4 bits) followed by a Code Index Number (4 bits)."
        // https://www.usb.org/sites/default/files/midi10.pdf (p.g. 16)
        //
        // Packet format see:
        //  https://www.usb.org/sites/default/files/USB%20MIDI%20v2_0.pdf
        if let Ok(event) = LiveEvent::parse(&buf[1..n]) {
            midi_source.send(decode_midi_message(event)).await;

            log_event(event);
        } else {
            error!("failed to parse midi message");
        }
    }
}

/// Task that waits for the USB host to enable the MIDI
/// endpoint, and the starts the MIDI event handler.
#[embassy_executor::task]
async fn usb_midi_sender_task(
    // Receives MIDI messages from the USB host.
    mut midi_class: midi::Sender<'static, usb::Driver<'static, USB_OTG_HS>>,
    midi_sink: MIDIDestinationReceiver<'static>,
) {
    info!("Starting MIDI Class sender handler...");
    loop {
        // Wait for a USB host to acknowledge
        // and enable the MIDI endpoint.
        midi_class.wait_connection().await;
        info!("Connected");

        // Clear the outgoing events channel so events added to
        // the channel before a valid connection are discarded.
        midi_sink.clear();

        // Start the MIDI handler for the host.
        let _ = midi_sender_handler(&mut midi_class, &midi_sink).await;
        info!("Disconnected");
    }
}

/// Handler for receiving and processing MIDI events incoming over USB.
async fn midi_sender_handler<'d>(
    midi_sender: &mut midi::Sender<'static, usb::Driver<'static, USB_OTG_HS>>,
    midi_source: &'_ MIDIDestinationReceiver<'static>,
) -> Result<(), Disconnected> {
    loop {
        // Wait for the next message produced by the MIDI
        // task that should be sent to the USB host.
        let message = midi_source.receive().await;

        // Encode the received message into MIDI formatting.
        if let Ok(mut buf) = encode_midi_message(message) {
            // Append a cable number of 0 and a code index of 1.
            //
            // Not sure why these specifically, but this seems
            // to be what works with modern DAWs (i.e. Ableton).
            //
            // "The first byte in each 32-bit USB-MIDI Event Packet is a Packet Header
            //  contains a Cable Number (4 bits) followed by a Code Index Number (4 bits)."
            // https://www.usb.org/sites/default/files/midi10.pdf (p.g. 16)
            buf.insert(0, 0b00001000);

            // Write the encoded packet to the USB host.
            midi_sender.write_packet(&buf).await?;

            info!("sent MIDI packet {=[u8]:b} {=[u8]:X} {}", &buf, &buf, &buf);
        } else {
            error!("error encoding MIDI message!");
        }
    }
}

struct Disconnected {}

impl From<EndpointError> for Disconnected {
    fn from(val: EndpointError) -> Self {
        match val {
            EndpointError::BufferOverflow => defmt::panic!("Buffer overflow"),
            EndpointError::Disabled => Disconnected {},
        }
    }
}
