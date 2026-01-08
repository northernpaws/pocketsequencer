use embassy_embedded_hal::shared_bus::I2cDeviceError;
use embassy_executor::{SpawnError, Spawner};
use embassy_stm32::i2c;
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    channel::{Channel, Receiver, Sender},
};
use firmware::hardware::{self, audio::AudioParameters};

mod task;

/// A table that defines how the codec's audio inputs and outputs are routed.
pub struct AudioRoutingTable {
    pub line_out_enabled: bool,
    pub speakers_enabled: bool,
}

impl Default for AudioRoutingTable {
    fn default() -> Self {
        Self {
            line_out_enabled: true,
            speakers_enabled: true,
        }
    }
}

impl AudioRoutingTable {
    /// Apply the routing table to the codec by updating the relevant registers.
    pub async fn apply(
        &self,
        codec: &'_ mut hardware::AudioCodec,
    ) -> Result<(), I2cDeviceError<i2c::Error>> {
        // Power down the AUX1/2 output mixer if not in use.
        codec
            .modify_powermanagement1(|reg| {
                reg.with_aux1mxen(self.line_out_enabled)
                    .with_aux2mxen(self.line_out_enabled)
            })
            .await?;

        // Power the enabled outputs.
        codec
            .modify_powermanagement3(|reg| {
                reg.with_auxout1en(self.line_out_enabled)
                    .with_auxout2en(self.line_out_enabled)
                    .with_lspken(self.speakers_enabled)
                    .with_rspken(self.speakers_enabled)
            })
            .await?;

        Ok(())
    }
}

pub enum AudioCommand {
    /// Updates the codec settings with the new audio routing table.
    UpdateRoutingTable(AudioRoutingTable),
}

/// Alias type for a channel that sends audio commands to the audio task.
pub type AudioCommandChannel = Channel<CriticalSectionRawMutex, AudioCommand, 2>;
pub type AudioCommandReceiver = Receiver<'static, CriticalSectionRawMutex, AudioCommand, 2>;
pub type AudioCommandSender = Sender<'static, CriticalSectionRawMutex, AudioCommand, 2>;

pub fn start(
    spawner: Spawner,
    codec: hardware::AudioCodec,
    params: AudioParameters,
) -> Result<(), SpawnError> {
    static AUDIO_COMMAND_CHANNEL: AudioCommandChannel = Channel::new();

    // Manages the codec over it's i2c control interface.
    task::spawn(spawner, codec, AUDIO_COMMAND_CHANNEL.receiver(), params)?;

    Ok(())
}
