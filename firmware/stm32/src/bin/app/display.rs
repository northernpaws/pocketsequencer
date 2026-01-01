use defmt::{info, warn};

use embassy_time::{Instant, Timer};

use firmware::hardware::RotatedDisplay;

#[embassy_executor::task]
pub async fn display_task(display: RotatedDisplay<'static>) -> ! {
    // should never return
    let err = inner_display_task(display).await;
    panic!("{:?}", err);
}

#[derive(Debug)]
enum Never {}

async fn inner_display_task(mut display: RotatedDisplay<'static>) -> Result<(), Never> {
    info!("starting display task");

    loop {
        // Record the time at the start of the frame so that we can
        // try to keep the frames-per-second around a known value.
        let start = Instant::now();

        // Render a frame to the framebuffer.

        // TODO: rendering

        // Push the frame to the display.
        display.push_buffer_dma().await.unwrap();

        // Dynamically clock the loop to run at roughly 30fps.
        //
        // This both makes the refresh of the display consistant,
        // but also makes sure we're not wasting unessessary time
        // on rendering the display.
        let time_ms = Instant::now().as_micros() - start.as_micros();
        if time_ms < 33_333 {
            Timer::after_micros(33_333 - time_ms);
        } else {
            warn!("display rendering behind {}us", 33_333 - time_ms);
        }
    }
}
