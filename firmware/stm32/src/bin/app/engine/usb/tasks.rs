use alloc::boxed::Box;
use defmt::{error, info, unwrap};
use embassy_executor::{SpawnError, Spawner};
use embassy_stm32::usb::{Driver, Instance};

use embassy_usb::{
    Builder, UsbDevice,
    class::{cdc_acm::CdcAcmClass, midi::MidiClass},
    driver::EndpointError,
};
use midly::{MidiMessage, live::LiveEvent};
use static_cell::StaticCell;

use crate::engine::midi::{
    MIDIDestinationReceiver, MIDIEndpoint, MIDIMessage, MIDISourceSender, SystemCommon,
};

pub fn start_usb_tasks(
    spawner: Spawner,
    usb_driver: Driver<'static, embassy_stm32::peripherals::USB_OTG_HS>,
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

    // Create classes on the builder.
    // let mut midi_class: MidiClass<'_, Driver<'_, embassy_stm32::peripherals::USB_OTG_HS>> = MidiClass::new(&mut builder, 1, 1, 64);
    // The `MidiClass` can be split into `Sender` and `Receiver`, to be used in separate tasks.
    // let (sender, receiver) = midi_class.split();

    // let mut midi_class: &'static mut MidiClass<'static, Driver<'static, embassy_stm32::peripherals::USB_OTG_HS>> = USB_MIDI_CLASS.init(MidiClass::new(&mut builder, 1, 1, 64));
    let midi_class = MidiClass::new(&mut builder, 1, 1, 64);

    // Build the builder.
    // let mut usb = USB_DEVICE.init(builder.build());
    let usb = builder.build();

    // Run the USB device.
    // let usb_fut = usb.run();

    // Use the Midi class!
    // let midi_fut = async {
    //     loop {
    //         midi_class.wait_connection().await;
    //         info!("Connected");
    //         let _ = midi_echo(&mut midi_class).await;
    //         info!("Disconnected");
    //     }
    // };

    info!("Spawning USB handler...");
    spawner.spawn(usb_task(usb)?);

    info!("Spawning MIDI class handler...");
    spawner.spawn(usb_midi_task(midi_class, midi_endpoint)?);

    Ok(())
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

async fn echo<'d, T: Instance + 'd>(
    class: &mut CdcAcmClass<'d, Driver<'d, T>>,
) -> Result<(), Disconnected> {
    let mut buf = [0; 64];
    loop {
        let n = class.read_packet(&mut buf).await?;
        let data = &buf[..n];
        info!("data: {:x}", data);
        class.write_packet(data).await?;
    }
}

/// Task for running the USB peripheral routines.
#[embassy_executor::task]
async fn usb_task(
    mut usb_device: UsbDevice<'static, Driver<'static, embassy_stm32::peripherals::USB_OTG_HS>>,
) {
    info!("Starting USB handler...");
    usb_device.run().await;
}

/// Task that waits for the USB host to enable the MIDI
/// endpoint, and the starts the MIDI event handler.
#[embassy_executor::task]
async fn usb_midi_task(
    mut midi_class: MidiClass<'static, Driver<'static, embassy_stm32::peripherals::USB_OTG_HS>>,
    midi_endpoint: MIDIEndpoint,
) {
    let midi_endpoint = midi_endpoint;

    info!("Starting MIDI Class handler...");
    loop {
        // Wait for a USB host to acknowledge
        // and enable the MIDI endpoint.
        midi_class.wait_connection().await;
        info!("Connected");

        // Clear the outgoing events channel so events added to
        // the channel before a valid connection are discarded.
        midi_endpoint.clear();

        // Start the MIDI handler for the host.
        let _ = midi_handler(&mut midi_class, &midi_endpoint).await;
        info!("Disconnected");
    }
}

/// Handler for an incoming MIDI connection over USB.
async fn midi_handler<'d, T: Instance + 'd>(
    class: &mut MidiClass<'d, Driver<'d, T>>,
    midi_endpoint: &'_ MIDIEndpoint,
) -> Result<(), Disconnected> {
    let mut buf = [0; 64];
    loop {
        let n = class.read_packet(&mut buf).await?;
        info!(
            "received MIDI packet {=[u8]:b} {=[u8]:X} {}",
            &buf[..n],
            &buf[..n],
            &buf[..n]
        );
        // class.write_packet(&buf[..n]).await?;

        // Throw away the first byte that's the MIDI Code Index Number (CIN):
        // "The first byte in each 32-bit USB-MIDI Event Packet is a Packet Header
        //  contains a Cable Number (4 bits) followed by a Code Index Number (4 bits)."
        // See: TODO add USB MIDI10 spec link
        //
        // Packet format see:
        //  https://www.usb.org/sites/default/files/USB%20MIDI%20v2_0.pdf (p.g. 16)
        if let Ok(event) = LiveEvent::parse(&buf[1..n]) {
            // We use publish_immediate to boot the last message off the channel if
            // it's full. We don't want to wait if the channel is full, otherwise
            // we'll stall processing the messages from the USB bus.
            midi_endpoint.send_message(match event {
                LiveEvent::Midi { channel, message } => MIDIMessage::Midi { channel, message },
                LiveEvent::Common(system_common) => match system_common {
                    midly::live::SystemCommon::SysEx(u7s) => {
                        MIDIMessage::Common(SystemCommon::SysEx(u7s.to_vec().into_boxed_slice()))
                    }
                    midly::live::SystemCommon::MidiTimeCodeQuarterFrame(
                        mtc_quarter_frame_message,
                        u4,
                    ) => MIDIMessage::Common(SystemCommon::MidiTimeCodeQuarterFrame(
                        mtc_quarter_frame_message,
                        u4,
                    )),
                    midly::live::SystemCommon::SongPosition(u14) => {
                        MIDIMessage::Common(SystemCommon::SongPosition(u14))
                    }
                    midly::live::SystemCommon::SongSelect(u7) => {
                        MIDIMessage::Common(SystemCommon::SongSelect(u7))
                    }
                    midly::live::SystemCommon::TuneRequest => {
                        MIDIMessage::Common(SystemCommon::TuneRequest)
                    }
                    midly::live::SystemCommon::Undefined(n, u7s) => MIDIMessage::Common(
                        SystemCommon::Undefined(n, u7s.to_vec().into_boxed_slice()),
                    ),
                },
                LiveEvent::Realtime(system_realtime) => MIDIMessage::Realtime(system_realtime),
            });

            log_event(event);
        } else {
            error!("failed to parse midi message");
        }
    }
}

/// Logs a decoded MIDI event.
fn log_event(event: LiveEvent<'_>) {
    match event {
        LiveEvent::Midi { channel, message } => match message {
            MidiMessage::NoteOn { key, vel: _ } => info!(
                "MIDI: note on {} on channel {}",
                key.as_int(),
                channel.as_int()
            ),
            MidiMessage::NoteOff { key, vel: _ } => info!(
                "MIDI: note off {} on channel {}",
                key.as_int(),
                channel.as_int()
            ),
            MidiMessage::Aftertouch { key, vel: _ } => info!(
                "MIDI: aftertouch {} on channel {}",
                key.as_int(),
                channel.as_int()
            ),
            MidiMessage::Controller { controller, value } => info!(
                "MIDI: controller {}={} on channel {}",
                controller.as_int(),
                value.as_int(),
                channel.as_int()
            ),
            MidiMessage::ProgramChange { program } => info!(
                "MIDI: program {} on channel {}",
                program.as_int(),
                channel.as_int()
            ),
            MidiMessage::ChannelAftertouch { vel: _ } => {
                info!("MIDI: aftertouch on channel {}", channel.as_int())
            }
            MidiMessage::PitchBend { bend } => info!(
                "MIDI: pitch bend {} on channel {}",
                bend.0.as_int(),
                channel.as_int()
            ),
        },
        LiveEvent::Common(system_common) => match system_common {
            midly::live::SystemCommon::SysEx(_) => info!("MIDI: SYS: SysEx"),
            midly::live::SystemCommon::MidiTimeCodeQuarterFrame(_, _u4) => {
                info!("MIDI: SYS: quarter frame")
            }
            midly::live::SystemCommon::SongPosition(u14) => {
                info!("MIDI: SYS: song position: {}", u14.as_int())
            }
            midly::live::SystemCommon::SongSelect(u7) => {
                info!("MIDI: SYS: song select: {}", u7.as_int())
            }
            midly::live::SystemCommon::TuneRequest => info!("MIDI: SYS: tune request"),
            midly::live::SystemCommon::Undefined(_, _) => info!("MIDI: SYS: undefined"),
        },
        LiveEvent::Realtime(system_realtime) => match system_realtime {
            midly::live::SystemRealtime::TimingClock => {
                info!("MIDI: REALTIME: timing clock")
            }
            midly::live::SystemRealtime::Start => info!("MIDI: REALTIME: start"),
            midly::live::SystemRealtime::Continue => info!("MIDI: REALTIME: continue"),
            midly::live::SystemRealtime::Stop => info!("MIDI: REALTIME: stop"),
            midly::live::SystemRealtime::ActiveSensing => {
                info!("MIDI: REALTIME: active sensing")
            }
            midly::live::SystemRealtime::Reset => info!("MIDI: REALTIME: reset"),
            midly::live::SystemRealtime::Undefined(_) => {
                info!("MIDI: REALTIME: undefined message")
            }
        },
    }
}

/// Rust's `!` is unstable.  This is a locally-defined equivalent which is stable.
#[derive(Debug)]
pub enum Never {}

// struct Disconnected {}

// impl From<EndpointError> for Disconnected {
//     fn from(val: EndpointError) -> Self {
//         match val {
//             EndpointError::BufferOverflow => defmt::panic!("Buffer overflow"),
//             EndpointError::Disabled => Disconnected {},
//         }
//     }
// }

// async fn echo<'d, T: usb::Instance + 'd>(class: &mut CdcAcmClass<'d, Driver<'d, T>>) -> Result<(), Disconnected> {
//     let mut buf = [0; 64];
//     loop {
//         let n = class.read_packet(&mut buf).await?;
//         let data = &buf[..n];
//         info!("data: {:x}", data);
//         class.write_packet(data).await?;
//     }
// }

// async fn midi_echo<'d, T: usb::Instance + 'd>(class: &mut MidiClass<'d, usb::Driver<'d, T>>) -> Result<(), Disconnected> {
//     let mut buf = [0; 64];
//     loop {
//         let n = class.read_packet(&mut buf).await?;
//         let data = &buf[..n];
//         info!("data: {:x}", data);
//         class.write_packet(data).await?;
//     }
// }

// #[embassy_executor::task]
// async fn usb_midi(
//     mut midi_class: &'static mut MidiClass<'static, Driver<'static, embassy_stm32::peripherals::USB_OTG_HS>>
// ) {
//     loop {
//         midi_class.wait_connection().await;
//         info!("Connected");
//         let _ = midi_echo(&mut midi_class).await;
//         info!("Disconnected");
//     }
// }
