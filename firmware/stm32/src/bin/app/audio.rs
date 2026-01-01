use defmt::info;

use crate::hardware::audio::Audio;

#[embassy_executor::task]
pub async fn audio_task(audio: Audio<'static>) -> ! {
    // should never return
    let err = inner_audio_task(audio).await;
    panic!("{:?}", err);
}

#[derive(Debug)]
enum Never {}

async fn inner_audio_task(audio: Audio<'static>) -> Result<(), Never> {
    info!("starting audio task");

    loop {}
}
