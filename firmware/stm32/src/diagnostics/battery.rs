use defmt::info;
use embassy_embedded_hal::shared_bus::asynch;
use embassy_stm32::{
    i2c::{I2c, Master},
    mode::Async,
};
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, RawMutex};
use embassy_time::{Delay, Timer};
use embedded_graphics::{
    mono_font::{MonoTextStyle, ascii::FONT_6X10, ascii::FONT_9X18_BOLD},
    pixelcolor::{Rgb565, Rgb888},
    prelude::*,
    primitives::{PrimitiveStyleBuilder, Rectangle},
    text::{Alignment, LineHeight, Text, TextStyleBuilder},
};
use embedded_graphics_coordinate_transform::Rotate270;
use heapless::String;

use crate::hardware::{
    display::Display,
    drivers::bq27531_g1::{
        Bq27531,
        command::CONTROL_IT_ENABLE_SUBCOMMAND,
        flash::{CHARGER_SUBCLASS_CHARGER_CONTROL_CONFIGURATION, FUEL_GAUGING_SUBCLASS_STATE},
    },
    keypad::{Button, Keypad, buttons::KeyCode},
    power::Power,
};

pub async fn run_diagnostics(
    display: &'_ mut Rotate270<Display<'_, Delay>>,
    keypad: &'_ mut Keypad,
    power: &'_ mut Power<'_>,
) {
    let mut channel = keypad.subscribe().unwrap();

    let mut on = true;

    let mut learning = false;

    loop {
        // We use *_raw and the flush methods to incrementally update the buffer
        // and push the update all at once instead of repeatedly signaling the
        // LED update task.
        keypad.clear_leds_raw();
        keypad.set_led_raw(Button::Trig8, Rgb888::WHITE);
        keypad.set_led_raw(Button::Trig12, Rgb888::RED);
        keypad.set_led_raw(Button::Trig13, Rgb888::RED);
        keypad.flush_leds();

        if on {
            keypad.set_led_raw(Button::Trig4, Rgb888::BLACK);
            on = false;
        } else {
            if let Ok(learning_state) = power
                .fuel_gauge_ref()
                .data_flash()
                .read_byte(FUEL_GAUGING_SUBCLASS_STATE, 6)
                .await
            {
                if learning_state == 0x00 {
                    keypad.set_led_raw(Button::Trig4, Rgb888::GREEN);
                    on = true;
                } else if learning_state != 0x06 {
                    keypad.set_led_raw(Button::Trig4, Rgb888::RED);
                    on = true;
                }
            } else {
                keypad.set_led_raw(Button::Trig4, Rgb888::RED);
                on = true;
            }
        }

        draw_diagnostics(display, power).await.unwrap();
        display.push_buffer_dma().await.unwrap();

        // Process input events.
        loop {
            // Immediately process any button events in the keypad channel.
            let Some(message) = channel.try_next_message() else {
                break;
            };

            use embassy_sync::pubsub::WaitResult::Message;

            let Message(event): embassy_sync::pubsub::WaitResult<
                crate::hardware::keypad::buttons::Event,
            > = message
            else {
                continue;
            };

            match event {
                crate::hardware::keypad::buttons::Event::KeyRelease(key_code) => {
                    if let Ok(button) = <KeyCode as TryInto<Button>>::try_into(key_code) {
                        match button {
                            Button::Trig4 => {
                                // Enable the Impedance Track algorithm to run a learn cycle.
                                info!("Starting Impedance Track battery pack learning cycle...");
                                power
                                    .fuel_gauge_ref()
                                    .write_control_command(CONTROL_IT_ENABLE_SUBCOMMAND)
                                    .await
                                    .unwrap();

                                // Reset to enable it
                                power.fuel_gauge_ref().write_command(0x41).await.unwrap();
                            }
                            Button::Trig8 => {
                                info!(
                                    "Enabling all LEDs to use as battery drain for learning discharge..."
                                );
                                keypad.set_leds(Rgb888::WHITE);
                            }
                            Button::Trig12 => {
                                info!("Disabling all LEDs...");
                                keypad.set_leds(Rgb888::BLACK);
                            }
                            // Exit from diagnostics
                            Button::Trig13 => return,

                            _ => {}
                        }
                    }
                }
                _ => {}
            }
        }

        // We don't need to update the statistics super frequently.
        Timer::after_secs(1).await;
    }
}

async fn draw_diagnostics<'a>(
    display: &'_ mut Rotate270<Display<'_, Delay>>,
    power: &'_ mut Power<'_>,
) -> Result<(), <Rotate270<Display<'a, Delay>> as DrawTarget>::Error> {
    let fuel_gauge = power.fuel_gauge_ref();

    display.clear(Rgb565::BLACK)?;

    let display_size = display.size();

    let padding = Point::new(10, 15);

    let mut y = 0;

    if let Ok(it_enable) = fuel_gauge
        .data_flash()
        .read_byte(FUEL_GAUGING_SUBCLASS_STATE, 0)
        .await
    {
        if it_enable == 0x00 {
            draw_ok_graphic(display, padding + Point::new(0, y), "IT Enable", "No")?;
        } else if it_enable == 0x02 {
            draw_ok_graphic(display, padding + Point::new(0, y), "IT Enable", "Yes")?;
        } else {
            draw_ok_graphic(display, padding + Point::new(0, y), "IT Enable", it_enable)?;
        }
    } else {
        draw_err_graphic(display, padding + Point::new(0, y), "Learning")?;
    }
    y += 10;

    if let Ok(learning_state) = fuel_gauge
        .data_flash()
        .read_byte(FUEL_GAUGING_SUBCLASS_STATE, 6)
        .await
    {
        if learning_state == 0x00 {
            draw_ok_graphic(display, padding + Point::new(0, y), "Learning", "Unknown")?;
        } else if learning_state == 0x01 {
            draw_ok_graphic(display, padding + Point::new(0, y), "Learning", "Init QMAX")?;
        } else if learning_state == 0x04 {
            draw_ok_graphic(display, padding + Point::new(0, y), "Learning", "Charge")?;
        } else if learning_state == 0x05 {
            draw_ok_graphic(display, padding + Point::new(0, y), "Learning", "Discharge")?;
        } else if learning_state == 0x06 {
            draw_ok_graphic(display, padding + Point::new(0, y), "Learning", "Complete")?;
        } else if learning_state == 0x02 {
            draw_ok_graphic(display, padding + Point::new(0, y), "Learning", "Optimized")?;
        } else {
            draw_ok_graphic(
                display,
                padding + Point::new(0, y),
                "Learning",
                learning_state,
            )?;
        }
    } else {
        draw_err_graphic(display, padding + Point::new(0, y), "Learning")?;
    }
    y += 10;

    if let Ok(cycle_count_1) = fuel_gauge
        .data_flash()
        .read_byte(FUEL_GAUGING_SUBCLASS_STATE, 4)
        .await
    {
        if let Ok(cycle_count_2) = fuel_gauge
            .data_flash()
            .read_byte(FUEL_GAUGING_SUBCLASS_STATE, 5)
            .await
        {
            draw_ok_graphic(
                display,
                padding + Point::new(0, y),
                "Cycle Count",
                u16::from_le_bytes([cycle_count_1, cycle_count_2]),
            )?;
        } else {
            draw_err_graphic(display, padding + Point::new(0, y), "Cycle Count 2")?;
        }
    } else {
        draw_err_graphic(display, padding + Point::new(0, y), "Cycle Count 1")?;
    }
    y += 10;

    if let Ok(design_capacity) = fuel_gauge.read_extended_design_capacity().await {
        draw_ok_graphic(
            display,
            padding + Point::new(0, y),
            "Design Capacity (mAh)",
            design_capacity,
        )?;
    } else {
        draw_err_graphic(display, padding + Point::new(0, y), "Design Capacity")?;
    }
    y += 10;

    if let Ok(voltage) = fuel_gauge.read_voltage().await {
        draw_ok_graphic(
            display,
            padding + Point::new(0, y),
            "Cell voltage (mV)",
            voltage,
        )?;
    } else {
        draw_err_graphic(display, padding + Point::new(0, y), "Cell voltage")?;
    }
    y += 10;

    if let Ok(remaining_capacity) = fuel_gauge.read_remaining_capacity().await {
        draw_ok_graphic(
            display,
            padding + Point::new(0, y),
            "Remaining capacity (mAh)",
            remaining_capacity,
        )?;
    } else {
        draw_err_graphic(display, padding + Point::new(0, y), "Remaining capacity")?;
    }
    y += 10;

    if let Ok(full_charge_capacity) = fuel_gauge.read_full_charge_capacity_filtered().await {
        draw_ok_graphic(
            display,
            padding + Point::new(0, y),
            "Full charge capacity (f, mAh)",
            full_charge_capacity,
        )?;
    } else {
        draw_err_graphic(display, padding + Point::new(0, y), "Full charge capacity")?;
    }

    y += 10;

    if let Ok(time_to_empty) = fuel_gauge.read_time_to_empty().await {
        draw_ok_graphic(
            display,
            padding + Point::new(0, y),
            "Time to empty (min)",
            time_to_empty,
        )?;
    } else {
        draw_err_graphic(display, padding + Point::new(0, y), "Time to empty")?;
    }

    y += 10;

    if let Ok(average_power) = fuel_gauge.read_average_power().await {
        draw_ok_graphic(
            display,
            padding + Point::new(0, y),
            "Average power (mW)",
            average_power,
        )?;
    } else {
        draw_err_graphic(display, padding + Point::new(0, y), "Average power")?;
    }

    y += 10;

    if let Ok(state_of_charge) = fuel_gauge.read_state_of_charge().await {
        draw_ok_graphic(
            display,
            padding + Point::new(0, y),
            "State of charge",
            state_of_charge,
        )?;
    } else {
        draw_err_graphic(display, padding + Point::new(0, y), "State of charge")?;
    }

    y += 10;

    if let Ok(state_of_charge) = fuel_gauge.read_true_soc().await {
        draw_ok_graphic(
            display,
            padding + Point::new(0, y),
            "True stage of charge (%)",
            state_of_charge,
        )?;
    } else {
        draw_err_graphic(display, padding + Point::new(0, y), "True state of charge")?;
    }

    y += 10;

    if let Ok(internal_temp) = fuel_gauge.read_internal_temperature().await {
        draw_ok_graphic(
            display,
            padding + Point::new(0, y),
            "Internal temp (c)",
            (internal_temp as f32 * 0.1) - 273.15,
        )?;
    } else {
        draw_err_graphic(display, padding + Point::new(0, y), "Internal temp")?;
    }

    y += 10;

    if let Ok(temp) = fuel_gauge.read_temperature().await {
        draw_ok_graphic(
            display,
            padding + Point::new(0, y),
            "Thermistor (c)",
            (temp as f32 * 0.1) - 273.15,
        )?;
    } else {
        draw_err_graphic(display, padding + Point::new(0, y), "Thermistor")?;
    }

    y += 10;

    if let Ok(average_current) = fuel_gauge.read_average_current().await {
        draw_ok_graphic(
            display,
            padding + Point::new(0, y),
            "Average current (mA)",
            average_current,
        )?;
    } else {
        draw_err_graphic(display, padding + Point::new(0, y), "Average current")?;
    }

    y += 20;

    if let Ok(operation_config_register) =
        fuel_gauge.data_flash().read_operation_configuration().await
    {
        draw_ok_graphic(
            display,
            padding + Point::new(0, y),
            "Operation Config H",
            operation_config_register[0],
        )?;
        y += 10;

        draw_ok_graphic(
            display,
            padding + Point::new(0, y),
            "Operation Config L",
            operation_config_register[1],
        )?;
        y += 10;
    } else {
        draw_err_graphic(display, padding + Point::new(0, y), "Operation Config")?;
    }

    if let Ok(reg) = fuel_gauge
        .data_flash()
        .read_byte(CHARGER_SUBCLASS_CHARGER_CONTROL_CONFIGURATION, 0)
        .await
    {
        draw_ok_graphic(display, padding + Point::new(0, y), "Charger Config", reg)?;
    } else {
        draw_err_graphic(display, padding + Point::new(0, y), "Charger Config")?;
    }

    y += 20;

    // if let Ok(fault) = fuel_gauge.read_charger_fault_register().await {
    //     draw_ok_graphic(display, padding + Point::new(0, y), "Charger Config", reg)?;
    // } else {
    //     draw_err_graphic(display, padding + Point::new(0, y), "Charger Fault")?;
    // }

    // y += 20;

    if let Ok(charger_status) = fuel_gauge.read_charger_status_register().await {
        if charger_status.wait_cmd() {
            draw_err_graphic(
                display,
                padding + Point::new(0, y),
                "Waiting for charger CMD",
            )?;
        } else if charger_status.denied() {
            draw_err_graphic(display, padding + Point::new(0, y), "Charger access denied")?;
        } else if charger_status.err() {
            draw_err_graphic(display, padding + Point::new(0, y), "Charger I2C")?;
        } else if charger_status.authfail() {
            draw_err_graphic(display, padding + Point::new(0, y), "Charger auth fail")?;
        } else if charger_status.wfail() {
            draw_err_graphic(display, padding + Point::new(0, y), "Charger write failed")?;
        } else if charger_status.init() {
            draw_err_graphic(display, padding + Point::new(0, y), "Charger initializing")?;
        } else {
            if let Ok(charger_system_status) =
                fuel_gauge.read_charger_system_status_register().await
            {
                if charger_system_status.vbus_stat() == 0 {
                    draw_ok_graphic(
                        display,
                        padding + Point::new(0, y),
                        "VBUS Status",
                        "Unknown",
                    )?;
                } else if charger_system_status.vbus_stat() == 1 {
                    draw_ok_graphic(
                        display,
                        padding + Point::new(0, y),
                        "VBUS Status",
                        "USB Host",
                    )?;
                } else if charger_system_status.vbus_stat() == 2 {
                    draw_ok_graphic(
                        display,
                        padding + Point::new(0, y),
                        "VBUS Status",
                        "Adapter",
                    )?;
                } else if charger_system_status.vbus_stat() == 3 {
                    draw_ok_graphic(display, padding + Point::new(0, y), "VBUS Status", "OTG")?;
                }

                y += 10;

                if charger_system_status.chrg_stat() == 0 {
                    draw_ok_graphic(
                        display,
                        padding + Point::new(0, y),
                        "Charger Status",
                        "Unknown",
                    )?;
                } else if charger_system_status.chrg_stat() == 1 {
                    draw_ok_graphic(
                        display,
                        padding + Point::new(0, y),
                        "Charger Status",
                        "Precharge",
                    )?;
                } else if charger_system_status.chrg_stat() == 2 {
                    draw_ok_graphic(
                        display,
                        padding + Point::new(0, y),
                        "Charger Status",
                        "Fast Charging",
                    )?;
                } else if charger_system_status.chrg_stat() == 3 {
                    draw_ok_graphic(
                        display,
                        padding + Point::new(0, y),
                        "Charger Status",
                        "Charge Done",
                    )?;
                }
            } else {
                draw_err_graphic(display, padding + Point::new(0, y), "Charger system status")?;
            }
        }
    } else {
        draw_err_graphic(display, padding + Point::new(0, y), "Charger connection")?;
    }

    display.push_buffer_dma().await.unwrap();

    Ok(())
}

pub fn draw_err_graphic<'a>(
    display: &'_ mut Rotate270<Display<'_, Delay>>,
    point: Point,
    label: &'static str,
) -> Result<(), <Rotate270<Display<'a, Delay>> as DrawTarget>::Error> {
    let err_character_style = MonoTextStyle::new(&FONT_6X10, Rgb565::RED);
    let err_text_style = TextStyleBuilder::new().alignment(Alignment::Left).build();

    Text::with_text_style(label, point, err_character_style, err_text_style).draw(display)?;

    Ok(())
}

pub fn draw_ok_graphic<'a, T: core::fmt::Display>(
    display: &'_ mut Rotate270<Display<'_, Delay>>,
    point: Point,
    label: &'static str,
    value: T,
) -> Result<(), <Rotate270<Display<'a, Delay>> as DrawTarget>::Error> {
    use core::fmt::Write;

    let ok_character_style = MonoTextStyle::new(&FONT_6X10, Rgb565::WHITE);
    let ok_text_style = TextStyleBuilder::new().alignment(Alignment::Left).build();

    // String on the stack for integer conversions.
    let mut intermeddiate_string = String::<64>::new();

    let _ = write!(intermeddiate_string, "{label}: {value}");

    Text::with_text_style(
        intermeddiate_string.as_str(),
        point,
        ok_character_style,
        ok_text_style,
    )
    .draw(display)?;

    Ok(())
}
