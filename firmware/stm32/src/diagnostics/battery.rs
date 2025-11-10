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
    drivers::bq27531_g1::Bq27531,
    keypad::{Button, Keypad, buttons::KeyCode},
};

pub async fn run_diagnostics(
    display: &'_ mut Rotate270<Display<'_, Delay>>,
    keypad: &'_ mut Keypad,
    fuel_gauge: &'_ mut Bq27531<
        '_,
        asynch::i2c::I2cDevice<'_, CriticalSectionRawMutex, I2c<'static, Async, Master>>,
        Delay,
    >,
) {
    let mut channel = keypad.subscribe().unwrap();

    // We use *_raw and the flush methods to incrementally update the buffer
    // and push the update all at once instead of repeatedly signaling the
    // LED update task.
    keypad.clear_leds_raw();
    keypad.set_led_raw(Button::Trig13, Rgb888::RED);
    keypad.flush_leds();

    loop {
        draw_diagnostics(display, fuel_gauge).await.unwrap();
        display.push_buffer_dma().await.unwrap();
        // TODO: select diagnostics like input

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
                crate::hardware::keypad::buttons::Event::KeyPress(key_code) => {
                    if let Ok(button) = <KeyCode as TryInto<Button>>::try_into(key_code) {
                        match button {
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
    fuel_gauge: &'_ mut Bq27531<
        '_,
        asynch::i2c::I2cDevice<'_, CriticalSectionRawMutex, I2c<'static, Async, Master>>,
        Delay,
    >,
) -> Result<(), <Rotate270<Display<'a, Delay>> as DrawTarget>::Error> {
    display.clear(Rgb565::BLACK)?;

    let display_size = display.size();

    let padding = Point::new(10, 15);

    if let Ok(remaining_capacity) = fuel_gauge.read_remaining_capacity().await {
        draw_ok_graphic(
            display,
            padding + Point::new(0, 0),
            "Remaining capacity (mAh)",
            remaining_capacity,
        )?;
    } else {
        draw_err_graphic(display, padding + Point::new(0, 0), "Remaining capacity")?;
    }

    if let Ok(full_charge_capacity) = fuel_gauge.read_full_charge_capacity_filtered().await {
        draw_ok_graphic(
            display,
            padding + Point::new(0, 10),
            "Full charge capacity (f, mAh)",
            full_charge_capacity,
        )?;
    } else {
        draw_err_graphic(display, padding + Point::new(0, 10), "Full charge capacity")?;
    }

    if let Ok(time_to_empty) = fuel_gauge.read_time_to_empty().await {
        draw_ok_graphic(
            display,
            padding + Point::new(0, 20),
            "Time to empty (min)",
            time_to_empty,
        )?;
    } else {
        draw_err_graphic(display, padding + Point::new(0, 20), "Time to empty")?;
    }

    if let Ok(average_power) = fuel_gauge.read_average_power().await {
        draw_ok_graphic(
            display,
            padding + Point::new(0, 30),
            "Average power (mW)",
            average_power,
        )?;
    } else {
        draw_err_graphic(display, padding + Point::new(0, 30), "Average power")?;
    }

    if let Ok(state_of_charge) = fuel_gauge.read_state_of_charge().await {
        draw_ok_graphic(
            display,
            padding + Point::new(0, 40),
            "State of charge",
            state_of_charge,
        )?;
    } else {
        draw_err_graphic(display, padding + Point::new(0, 40), "State of charge")?;
    }

    if let Ok(state_of_charge) = fuel_gauge.read_true_soc().await {
        draw_ok_graphic(
            display,
            padding + Point::new(0, 50),
            "True stage of charge (%)",
            state_of_charge,
        )?;
    } else {
        draw_err_graphic(display, padding + Point::new(0, 50), "True state of charge")?;
    }

    if let Ok(internal_temp) = fuel_gauge.read_internal_temperature().await {
        draw_ok_graphic(
            display,
            padding + Point::new(0, 60),
            "Internal temp (c)",
            (internal_temp as f32 * 0.1) - 273.15,
        )?;
    } else {
        draw_err_graphic(display, padding + Point::new(0, 60), "Internal temp")?;
    }

    if let Ok(average_current) = fuel_gauge.read_average_current().await {
        draw_ok_graphic(
            display,
            padding + Point::new(0, 70),
            "Average current",
            average_current,
        )?;
    } else {
        draw_err_graphic(display, padding + Point::new(0, 70), "Average current")?;
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
