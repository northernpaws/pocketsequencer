#![no_std]
#![no_main]

#[cfg(feature = "alloc")]
extern crate alloc;

// https://medium.com/@carlmkadie/how-rust-embassy-shine-on-embedded-devices-part-1-9f4911c92007
// https://medium.com/@carlmkadie/aad1adfccf72

use core::fmt::Debug;

use alloc::boxed::Box;
use cortex_m_rt::{ExceptionFrame, exception};
use defmt::*;

use embassy_executor::Spawner;

use embassy_futures::yield_now;
use embassy_stm32::{
    gpio::{Input, Output},
    i2c::{self, I2c},
    mode::Async,
    rcc::clocks,
    spi::Spi,
    usart::{UartRx, UartTx},
};

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex, signal::Signal};

use embassy_time::{Delay, Timer};

use embedded_graphics::{
    mono_font::{
        MonoTextStyle,
        ascii::{FONT_6X10, FONT_10X20},
    },
    pixelcolor::Rgb565,
    primitives::{PrimitiveStyleBuilder, Rectangle},
    text::{Alignment, LineHeight, Text, TextStyleBuilder},
};

use embedded_graphics_coordinate_transform::Rotate270;

use mousefood::{EmbeddedBackend, EmbeddedBackendConfig, prelude::Rgb888};
use ratatui::Terminal;
use rgb_led_pwm_dma_maker::RGB;
use sdio_host::{
    common_cmd::{select_card, set_block_length},
    sd::BlockSize,
};
use sdspi::sd_init;

use embedded_graphics::prelude::RgbColor;
use embedded_graphics_core::draw_target::DrawTarget;

use static_cell::StaticCell;

use {
    // defmt_rtt as _,
    panic_probe as _,
};

use firmware::{
    diagnostics,
    hardware::{
        self, Irqs,
        display::{self, Display},
        drivers::stm6601::Stm6601,
        keypad::{
            self, Keypad,
            buttons::{Event, KeyCode},
        },
        preamble::*,
    },
    split_resources,
};

use engine;

// NOTE: We try to rely on heap allocations as little
// as possible for core elements of the system.
//
// Where possible, core systems should use heapless static
// allocation or allocation pools so their size is known
// and we avoid running into OOM problems.
#[cfg(feature = "alloc")]
use embedded_alloc::LlffHeap as Heap;
#[cfg(feature = "alloc")]
#[global_allocator]
static HEAP: Heap = Heap::empty();

static SHUTDOWN_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

// Audio I2C bus.
//
// This communicates with the NAU88C22YG Audio Codec, FM SI4703-C19-GMR RX / SI4710-B30-GMR TX.
// static I2C1_BUS: <NoopMutex<RefCell<I2c<'static, Async, i2c::Master>>>> = StaticCell::new();

// Power management bus.
//
// This communicates with the BQ24193 battery charger, BQ27531YZFR-G1 fuel gauge, and FUSB302B USB-PD manager.
// static I2C2_BUS: StaticCell<NoopMutex<RefCell<I2c<'static, Async, i2c::Master>>>> = StaticCell::new();
static I2C2_BUS: StaticCell<Mutex<CriticalSectionRawMutex, I2c<'static, Async, i2c::Master>>> =
    StaticCell::new();

// Input bus.
//
// This communicates with the FT6206 Capacitive Touch sensor, TCA8418RTWR Keypad matrix,
// ADS7128IRTER GPIO Breakout for Velocity Grid, DA7280 Haptics driver, and MMA8653FCR1 9DOF.
// static I2C4_BUS: StaticCell<NoopMutex<RefCell<I2c<'static, Async, i2c::Master>>>> = StaticCell::new();
static I2C4_BUS: StaticCell<Mutex<CriticalSectionRawMutex, I2c<'static, Async, i2c::Master>>> =
    StaticCell::new();

/// SPI bus for internal and SD card storage.
static SPI_BUS: StaticCell<Mutex<CriticalSectionRawMutex, Spi<'static, Async>>> = StaticCell::new();
// static SPI_BUS: StaticCell<blocking_mutex::Mutex<CriticalSectionRawMutex, RefCell<Spi<'static, Async>>>> = StaticCell::new();

/// Custom HardFault handler to improve debugging.
///
/// ref: https://doc.rust-lang.org/beta/embedded-book/start/exceptions.html#the-hard-fault-handler
#[exception]
unsafe fn HardFault(ef: &ExceptionFrame) -> ! {
    // Check `SCB -> CFSR_UFSR_BFSR_MMFSR` register
    // to see the fault and then reference:
    //  https://community.st.com/t5/stm32-mcus/how-to-debug-a-hardfault-on-an-arm-cortex-m-stm32/ta-p/672235#toc-hId--1471762501

    error!("HARD FAULT:");
    error!("r0  = 0x{:X}", ef.r0());
    error!("r1  = 0x{:X}", ef.r1());
    error!("r2  = 0x{:X}", ef.r2());
    error!("r3  = 0x{:X}", ef.r3());
    error!("r12 = 0x{:X}", ef.r12());
    error!("lr  = 0x{:X}", ef.lr());

    if (ef.lr() & 0x8) == 1 {
        error!("LR indicates Main Stack Pointer (MSP) TODO double check this is correct")
    } else {
        error!("LR indicates Process Stack Pointer (PSP) TODO double check this is correct")
    }

    // cargo objdump -- -d --no-show-raw-insn --print-imm-hex | grep <PC_HEX_WITHOUT_0x> -A 10 -B 10
    let pc = ef.pc();
    error!("pc: 0x{:X}", pc);
    error!("  Inspect assembled program with:");
    error!(
        "  $ cargo objdump -- -d --no-show-raw-insn --print-imm-hex | grep {:x} -A 10 -B 10",
        pc
    );
    error!("xpsr: {:b}", ef.xpsr());

    // Attempt to read the cortex registers to get more info.
    unsafe {
        let mut core_peri: cortex_m::Peripherals = cortex_m::Peripherals::steal();
        let scb = &mut core_peri.SCB;
        error!("hfsr: {:b}", scb.hfsr.read());

        // Chek HFSR bit 30 to see if we need to check other registers for details.
        if (scb.hfsr.read() & (1 << 30)) != 0 {
            error!("Forced Hard Fault!");
            error!(" HFSR FORCED bit set, see CFSR for details.");

            // NOTE: CFSR contains the UFSR, BFSR, and MMFSR registers.
            error!("  SCB->CFSR: {:b}", scb.cfsr.read());

            // Mask out the UFSR section of the CFSR register (first half).
            if (scb.cfsr.read() & 0xFFFF0000) != 0 {
                let ufsr: u16 = (scb.cfsr.read() >> 16) as u16;
                error!("  Usage Fault Indicator set!");
                error!("    SCB->CFSR->UFSR={:b}", ufsr);

                // TODO: error messages https://developer.arm.com/documentation/dui0552/a/cortex-m3-peripherals/system-control-block/configurable-fault-status-register
            }

            // BFSR
            if (scb.cfsr.read() & 0xFF00) != 0 {
                error!("  Bus Fault Indicator set!");

                let bsfr = (scb.cfsr.read() & 0xFF00) >> 8;
                error!("    SCB->CSFR->BSFR={:b}", bsfr);

                if (bsfr & 0b1) != 0 {
                    // 0
                    error!("    IBUSERR: Instruction bus error.")
                } else if (bsfr & 0b10) != 0 {
                    // 1
                    error!("    PRECISERR: Precise data bus error.");
                    error!("      Offending address (BFAR): 0x{:x}", scb.bfar.read());
                } else if (bsfr & 0b100) != 0 {
                    // 2
                    error!("    IMPRECISERR: Imprecise data bus error.");
                    error!("      Due to an Imprecise data bus error, the return address in the ");
                    error!("       stack does not point to the instruction that caused the fault!");
                } else if (bsfr & 0b1000) != 0 {
                    // 3
                    error!("    UNSTKERR: BusFault on unstacking for a return from exception.");
                } else if (bsfr & 0b10000) != 0 {
                    // 4
                    error!("    STKERR: BusFault on stacking for exception entry.");
                } else {
                    error!("    Unknown bus fault! See register for details.");
                }
            }

            // MMFSR
            if (scb.cfsr.read() & 0xFF) != 0 {
                error!("  Memory Management Indicator set!");
                error!("    Memory fault location: 0x{:X}", scb.mmfar.read());
                // TODO: error messages https://developer.arm.com/documentation/dui0552/a/cortex-m3-peripherals/system-control-block/configurable-fault-status-register
            }
        }
    }

    // TODO: attempt hard fault recovery https://github.com/kendiser5000/Hardfault-Recovery-Cortex-M4/blob/master/hardfault_handler.c

    loop {}
}

// TODO: add general exception handlers with keypad access to bink error LEDs.
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("program start!");
    // If it returns, something went wrong.
    if let Err(err) = inner_main(spawner).await {
        defmt::panic!("{}", err);
    }
}

static DEBUG_SERIAL_TX: StaticCell<UartTx<'static, Async>> = StaticCell::new();
static DEBUG_SERIAL_RX: StaticCell<UartRx<'static, Async>> = StaticCell::new();

// Inner-main allows us us to returns errors via Result instead of relying on panics.
#[expect(
    clippy::future_not_send,
    reason = "Safe in single-threaded, bare-metal embedded context"
)]
#[expect(clippy::items_after_statements, reason = "Keeps related code together")]
async fn inner_main(spawner: Spawner) -> Result<(), ()> {
    // TODO: add error type
    info!("initializing clocks and PLL to 480Mhz");
    let p = hardware::init();

    info!("getting handles to hardware peripherals");
    let r = split_resources!(p);

    // Get a handle to the STM6601 power manager and enable PS_HOLD as early as
    // possible to make sure the STM6601 won't power the system regulator off.
    //
    // This MUST be configured as early as possible to ensure that the
    // PS_HOLD pin is held high by the MCU to keep the power enabled.
    //
    // NOTE: PLL errors before this point will cause the power controller to shut
    // off the paniced MCU after 5 seconds, leaving little time for debugging.
    info!("Configuring stm6601...");
    let mut stm6601 = hardware::get_stm6601(r.power);
    stm6601.power_enable().unwrap(); // TODO: change to result error

    info!("Power good confirmed!");

    // Initialize the debug UART connection to the BMP
    // header and configure it for use with defmt.
    //
    // Baud: 115200
    // DataBits8
    // ParityNone
    // Stop1
    let mut debug_uart = hardware::get_uart_debug(r.uart_debug);
    debug_uart.blocking_write(b"debug serial ready").unwrap();
    let (debug_tx_raw, debug_rx_raw) = debug_uart.split();
    let debug_tx = DEBUG_SERIAL_TX.init(debug_tx_raw);
    let _debug_rx = DEBUG_SERIAL_RX.init(debug_rx_raw);
    defmt_serial::defmt_serial(debug_tx);

    let clocks = clocks(&p.RCC);
    println!("System clock speed: {}", clocks.sys);
    println!("APB1  clock speed: {}", clocks.pclk1);
    println!("APB1 Timer clock speed: {}", clocks.pclk1_tim);
    println!("APB2  clock speed: {}", clocks.pclk2);
    println!("APB2 Timer clock speed: {}", clocks.pclk2_tim);

    // Initialize the bus for the I2C4 peripheral.
    //
    // This communicates with the FT6206 Capacitive Touch sensor,
    // TCA8418RTWR Keypad matrix, ADS7128IRTER GPIO Breakout for
    // Velocity Grid, DA7280 Haptics driver, and MMA8653FCR1 9DOF.
    info!("initializing input I2C4 bus");
    let i2c4 = hardware::get_i2c4(r.i2c4);
    let i2c4_bus = I2C4_BUS.init(Mutex::new(i2c4));

    // Initialize the keypad fairly early so that we can
    // use the keypad LEDs as status indicators and use
    // the buttons to alter the boot sequence.
    info!("Initializing keypad...");
    static KEYPAD: StaticCell<Keypad> = StaticCell::new();
    let mut keypad: &mut Keypad = KEYPAD.init(
        hardware::get_keypad(spawner, r.keypad, i2c4_bus)
            .await
            .unwrap(),
    );

    // Spawn the task that handles the power button.
    //
    // TODO: move this to an EXTI interrupt so that we're
    // not relying on the task eventually being called.
    spawner.spawn(unwrap!(power_button_task(stm6601)));

    // Next, we configure the memory buses for the SDRAM and the display.
    //
    // We initialize the display as soon as possible to display any boot errors.
    info!("Configuring memory controller...");
    keypad.set_led(keypad::Button::Trig1, Rgb888::YELLOW);
    let Ok((mut sdram, display)) =
        hardware::get_memory_devices(r.fmc, r.display, embassy_time::Delay).await
    else {
        loop {
            // Blink an LED to indicate an error.
            keypad.set_led(keypad::Button::Trig1, Rgb888::RED);
            Timer::after_millis(25).await;
            keypad.set_led(keypad::Button::Trig1, Rgb888::BLACK);
            Timer::after_millis(25).await;
        }
    };
    keypad.set_led(keypad::Button::Trig1, Rgb888::GREEN);

    // Wrap the display in a translation layer that rotates it
    // 270 degrees into landscape with the correct orientation.
    //
    // We need to use software rotation instead of rotation in
    // the LCD driver because the LCD driver doesn't rotate the
    // scanning direction, causing nasty diagonal tearing.
    let mut display_rot = Rotate270::new(display);

    // Next, handle boot diagnostics.
    //
    // This runs in a loop so that the diagnostics menu
    // can be closed and reopened within a short period.
    let Ok(mut subscriber) = keypad.subscribe() else {
        defmt::panic!("Failed to get a keypad button subscriber")
    };

    // Before we continue, force a scan of the keypad to
    // ensure we have a current map of the key states.
    //
    // This is important so we can catch any special key states
    // for purposes such as diagnostics, bootloader triggers, and
    // special setting modes.
    info!("Scanning keypad for boot interruption keybinds..");

    // Immediately flush the button FIFO from the TCA8418
    // so that we have an up-to-date button state.
    //
    // Flush awaits until the end of the next keypad scan, which
    // is usually the scan manually triggered by the flush.
    keypad.flush().await;

    loop {
        // Imeddiatly receive pending button events until none are left.
        let Some(message) = subscriber.try_next_message() else {
            break;
        };

        use embassy_sync::pubsub::WaitResult::Message;

        let Message(event) = message else {
            continue;
        };

        let Event::KeyPress(keycode) = event else {
            continue;
        };

        // Match the button to a diagnostic or boot setting.
        match keycode {
            KeyCode::Trig1 => {
                // TODO: load diagnostics depending on keypad state
                info!("Entering diagnostics menu...");
                diagnostics::run_diagnostics(&mut display_rot, &mut keypad).await;
                info!("Exited diagnostics menu.");
            }
            _ => {
                info!("Ignoring unbound boot hotkey: {}", keycode);
            }
        }
    }

    info!("Running fast memory check...");

    draw_boot_screen(&mut display_rot, "Memory test..", &[])
        .await
        .unwrap();

    // After we've got the display initialized so we can show status, perform a very
    // very brief memory test to see if we can write and read from the SDRAM.

    info!("RAM contents before writing: {:x}", sdram[..10]);
    sdram[0] = 1;
    sdram[1] = 2;
    sdram[2] = 3;
    sdram[3] = 4;
    info!("RAM contents after writing: {:x}", sdram[..10]);

    if sdram[0] != 1 || sdram[1] != 2 || sdram[2] != 3 || sdram[3] != 4 {
        error!("Brief memory test failed!");
        draw_boot_screen(&mut display_rot, "Memory test failed!", &[])
            .await
            .unwrap();

        loop {
            // Blink an LED to indicate an error.
            keypad.set_led(keypad::Button::Trig2, Rgb888::RED);
            Timer::after_millis(25).await;
            keypad.set_led(keypad::Button::Trig2, Rgb888::GREEN);
            Timer::after_millis(25).await;
        }
    }

    info!("Memory test succeeded.");

    info!("Initializing heap allocator...");

    // Next we need to initialize the heap allocator for the program.
    //
    // Note that this needs to be done before drawing to the display,
    // as ratataui uses the heap allocator for it's widgets.
    //
    // NOTE: We try to rely on heap allocations as little
    // as possible for core elements of the system.
    //
    // Where possible, core systems should use heapless static
    // allocation or allocation pools so their size is known
    // and we avoid running into OOM problems.
    {
        const HEAP_SIZE: usize = 1024 * 128;

        // Split off a section of the SDRAM memory for the heap allocator.
        //
        // This syntax lets us use the SDRAM slice to keep track of how much
        // of it we can allocate to other parts of the system without needing
        // to remeber where pointers have been used.
        let Some((heap_slice, new_sdram)) = sdram.split_at_mut_checked(HEAP_SIZE) else {
            error!("No space left in SDRAM for heap!");

            draw_boot_screen(
                &mut display_rot,
                "No space in SDRAM for heap!",
                &["Memory test"],
            )
            .await
            .unwrap();

            loop {}
        };

        // Assign the split-off portion back to our original slice
        // reference so that we can track the remaining amount of
        // SDRAM available to allocate to other systems.
        sdram = new_sdram;

        // Place the heap allocation pool in the external SDRAM.
        unsafe { HEAP.init(heap_slice.as_ptr().addr(), heap_slice.len()) }

        // TODO: we need an exception handler for over-allocation exceptions to handle them gracefully.
        info!("Heap successfully initialized.");
    }

    draw_boot_screen(&mut display_rot, "SD Card...", &["Memory test"])
        .await
        .unwrap();

    // Initialize the bus for the SPI1 peripheral.
    //
    // This communicates with the internal storage and Micro SD card.
    info!("initializing storage SPI1 bus");
    keypad.set_led(keypad::Button::Trig2, Rgb888::RED);
    let (spi1, sd_cs, mut xtsdg_cs) = hardware::get_spi1(r.spi_storage);

    // Convert the SPI1 peripheral handle into a bus handle that can be consumed by multiple devices.
    let spi_bus = SPI_BUS.init(Mutex::new(spi1));

    // Initialize the internal and SD card storage next.
    //
    // We also want to initialize storage fairly early in the startup process so that
    // we can check for key files on the device, such as firmware update indicators.

    // SD cards need to be clocked with a at least 74 cycles
    // on their SPI clock with the CS pin held HIGH.
    //
    // sd_init is a helper function that does this for us.
    info!("Clocking SPI1 74 cyles before initializing SD devices...");
    loop {
        keypad.set_led(keypad::Button::Trig2, Rgb888::BLACK);
        match sd_init(spi_bus.get_mut(), &mut xtsdg_cs).await {
            Ok(_) => break,
            Err(_e) => {
                keypad.set_led(keypad::Button::Trig2, Rgb888::RED);
                error!("SPI init error!");
                embassy_time::Timer::after_millis(10).await;
            }
        }
    }

    // Now we can initialize the Micro SD card driver.
    info!("Initializing SD card device..");
    let mut sd_card = hardware::init_sdcard_async2(spi_bus, sd_cs, embassy_time::Delay)
        .await
        .unwrap();
    keypad.set_led(keypad::Button::Trig2, Rgb888::GREEN);

    draw_boot_screen(
        &mut display_rot,
        "Internal Storage...",
        &["Memory test", "SD Card"],
    )
    .await
    .unwrap();

    // sd_card.list_filesystem().await.unwrap();

    // Initialize the internal storage.
    //
    // The device internal storage uses an XTSDG IC
    // that acts as a soldered SD card.
    /*info!("Constructing internal storage device...");
    keypad.set_led(keypad::Button::Trig4, RGB::new(255, 0, 0));
    let mut internal_storage = hardware::get_internal_storage(spi_bus, xtsdg_cs, embassy_time::Delay);
    keypad.set_led(keypad::Button::Trig4, RGB::new(0, 255, 0));*/

    draw_boot_screen(
        &mut display_rot,
        "Battery...",
        &["Memory test", "SD Card", "TODO: Internal Storage"],
    )
    .await
    .unwrap();

    /*
        // Next initialize the bus for the I2C2 peripheral that
        // has all the power management peripherals attatched.
        //
        // This communicates with the BQ24193 battery charger,
        // BQ27531YZFR-G1 fuel gauge, FUSB302B USB-PD.
        //
        // Note that the communication with the BQ24193 charger happens
        // THROUGH the BQ27531YZFR-G1 fuel gauge. They're interconnected
        // on their own I2C bus with a special set of registers on the
        // fuel gauge to interact with the charger.
        info!("Initializing power i2c2 bus...");
        let i2c2_bus = I2C2_BUS.init(Mutex::new(hardware::get_i2c2(r.i2c2)));

        // Get a handle to the battery fuel gauge peripheral.
        let mut fuel_gauge = hardware::get_bq27531_g1_async(
            r.fuel_gauge.int,
            r.fuel_gauge.int_exti,
            i2c2_bus,
            embassy_time::Delay);

        info!("Attempting to read battery charger internal temp...");
        if let Ok(temp) = fuel_gauge.read_internal_temperature().await {
            info!("Battery charger internal temp: {}", temp);
        } else {
            error!("Failed to read battery charger temp!");
        }

        // Get a handle to the FUSB302B device for managing the USB-PD interface.
        let mut fusb302b = hardware::get_fusb302b_async(r.usb_pd.int, r.usb_pd.int_exti, i2c2_bus);

        // Initialize the bus for the I2C1 peripheral.
        //
        // This communicates with the NAU88C22YG Audio Codec,
        // FM SI4703-C19-GMR RX / SI4710-B30-GMR TX.
        info!("initializing audio i2c1 bus");
        let _i2c1_bus = I2C1_BUS.init(hardware::get_i2c1(r.i2c1));
    */

    // info!("Starting USB device...");
    // hardware::usb::start_usb(spawner, r.usb).await;

    // We're basically booted at this point and past any critical
    // error stages, so clear boot status from the keypad LEDs.
    keypad.set_leds(Rgb888::BLACK);

    // Clear the display once more before starting the ratataui.
    info!("Configuring display engine...");
    display_rot.clear(Rgb565::BLACK).unwrap();
    display_rot.push_buffer_dma().await.unwrap();

    // A Ratatui wrapper for embedded-graphics.
    //
    // Will be replaced later with our own embedded-optimized
    // UI framework, but for now it'll work great for testing.
    // let backend_config = EmbeddedBackendConfig {
    //     // Define how to display newly rendered widgets to the simulator window
    //     flush_callback: Box::new(
    //         move |display: &mut embedded_graphics_coordinate_transform::CoordinateTransform<
    //             display::Display<'_, embassy_time::Delay>,
    //             false,
    //             true,
    //             true,
    //         >| {
    //             display.push_buffer_dma().await;
    //             // TODO: dispatch queue in async method for DMA.
    //             // async {
    //             //     display.push_buffer_dma().await.unwrap();
    //             // };
    //         },
    //     ),
    //     ..Default::default()
    // };
    // let backend = EmbeddedBackend::new(&mut display_rot, backend_config);
    // let mut terminal = Terminal::new(backend).unwrap();

    loop {
        // terminal.draw(draw).unwrap();
        // display_rot.push_buffer_dma().await.unwrap();

        // Give other tasks time to do work.
        // Timer::after_millis(10).await;

        SHUTDOWN_SIGNAL.wait().await;
        keypad.set_leds(Rgb888::BLACK);
    }

    // info!("Initializing engine..");
    // let mut engine_instance = engine::Engine::new(keypad);

    // engine_instance.start().await;

    /*info!("Starting wireless UART peripheral...");
        let wireless_uart = hardware::get_uart_rf(r.uart_rf);
        let (mut wireless_tx, mut wireless_rx) = wireless_uart.split();

        let wireless_read = async {
            let mut buf = [0u8; 16];
            loop {
                unwrap!(wireless_rx.read(&mut buf).await);
                info!("received wireless uart message: {} {=[u8]:a}", buf, buf);
            }
        };

        let wireless_write = async {
            let mut buf = [0u8; 16];
            loop {
                unwrap!(debug_rx.read(&mut buf).await);
                info!("received wireless uart message: {} {=[u8]:a}", buf, buf);
                unwrap!(wireless_tx.write(&mut buf).await);
            }
        };

        join(wireless_read, wireless_write).await;
    */
}

async fn draw_boot_screen<'a>(
    display: &'_ mut Rotate270<Display<'_, Delay>>,
    component: &'a str,
    success_components: &[&'a str],
) -> Result<(), <Rotate270<Display<'a, Delay>> as DrawTarget>::Error> {
    use embedded_graphics::prelude::*;

    // First, we need to clear the framebuffer to black
    // to overwrite any previously drawn elements.
    display.clear(Rgb565::BLACK)?;

    let border_style = PrimitiveStyleBuilder::new()
        .stroke_color(Rgb565::YELLOW)
        .stroke_width(3)
        .reset_fill_color()
        .build();

    let display_size = display.bounding_box().size;

    Rectangle::new(
        Point::new(20, 20),
        Size::new(display_size.width - 40, display_size.height - 40),
    )
    .into_styled(border_style)
    .draw(display)?;

    let large_character_style = MonoTextStyle::new(&FONT_10X20, Rgb565::WHITE);
    let small_character_style = MonoTextStyle::new(&FONT_6X10, Rgb565::WHITE);
    let small_character_style_success = MonoTextStyle::new(&FONT_6X10, Rgb565::GREEN);

    let text_style = TextStyleBuilder::new()
        .alignment(Alignment::Left)
        .line_height(LineHeight::Percent(150))
        .build();

    let left_offset = 40;

    Text::with_text_style(
        "Booting...",
        Point::new(left_offset, 50),
        large_character_style,
        text_style,
    )
    .draw(display)?;

    let mut offset = 0;
    for comp in success_components {
        Text::with_text_style(
            comp,
            Point::new(left_offset, 80 + offset),
            small_character_style_success,
            text_style,
        )
        .draw(display)?;
        offset += 15;
    }

    Text::with_text_style(
        component,
        Point::new(left_offset, 80 + offset),
        small_character_style,
        text_style,
    )
    .draw(display)?;

    display.push_buffer_dma().await.unwrap();

    Ok(())
}

#[embassy_executor::task]
pub async fn power_button_task(
    mut stm6601: Stm6601<'static, Output<'static>, Input<'static>>,
) -> ! {
    trace!("Starting power button listener...");
    loop {
        // Wait for the power button to be pressed.
        stm6601.wait_for_button_press().await;

        info!("Power button pressed!");

        // Signal that there has been a shutdown button press.
        SHUTDOWN_SIGNAL.signal(());

        // TODO: actual shutdown confirmation signal
        yield_now().await;

        // Trigger shutdown routine.
        // TODO: clear keypad LEDs since they don't have a power switch..
        // keypad.set_all(RGB::new(0, 0, 0));
        // keypad.wait_for_led_update().await;
        // // TODO:
        stm6601.power_disable().unwrap();
    }
}

// fn draw(frame: &mut ratatui::Frame) {
//     let text = "Ratatui on embedded devices!";
//     let paragraph = ratatui::widgets::Paragraph::new(ratatui::style::Stylize::dark_gray(text))
//         .wrap(ratatui::widgets::Wrap { trim: true });
//     let bordered_block = ratatui::widgets::Block::bordered()
//         .border_style(ratatui::style::Style::new().yellow())
//         .title("Mousefood");
//     frame.render_widget(paragraph.block(bordered_block), frame.area());
// }

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
