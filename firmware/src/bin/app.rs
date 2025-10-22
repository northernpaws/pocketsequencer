#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

use firmware::{
    split_resources,
    hardware::{
        self,
        preamble::*
    }
};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = hardware::init();

    info!("Hello World!");

    let r = split_resources!(p);

    loop {}

    // let mut led = Output::new(p.PB14, Level::High, Speed::Low);

    // loop {
    //     info!("high");
    //     led.set_high();
    //     Timer::after_millis(500).await;

    //     info!("low");
    //     led.set_low();
    //     Timer::after_millis(500).await;
    // }
}
