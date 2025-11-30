use defmt::{error, info, warn};

use embassy_embedded_hal::shared_bus::{I2cDeviceError, asynch};
use embassy_stm32::{
    i2c::{self, I2c},
    mode::Async,
};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::Delay;
use proc_bitfield::Bitfield;

use crate::hardware::drivers::bq27531_g1::{
    Bq27531,
    command::CONTROL_GG_CHGRCTL_ENABLE_SUBCOMMAND,
    flash::{
        CHARGER_SUBCLASS_CHARGER_CONTROL_CONFIGURATION,
        class::configuration::{SUBCLASS_DATA, SUBCLASS_REGISTERS},
    },
};

pub struct Power<'a> {
    fuel_gauge: Bq27531<
        'a,
        asynch::i2c::I2cDevice<'a, CriticalSectionRawMutex, I2c<'static, Async, i2c::Master>>,
        Delay,
    >,
}

impl<'a> Power<'a> {
    pub async fn new(
        mut fuel_gauge: Bq27531<
            'a,
            asynch::i2c::I2cDevice<'a, CriticalSectionRawMutex, I2c<'static, Async, i2c::Master>>,
            Delay,
        >,
    ) -> Result<Self, I2cDeviceError<embassy_stm32::i2c::Error>> {
        // Instruct the fuel gauge to take control of the connected charger.
        //
        // TODO: This can be programed as a default into flash, see:
        //  https://e2e.ti.com/support/power-management-group/power-management/f/power-management-forum/560822/how-to-configure-write-to-data-memory-of-bq27531-g1-is-my-understanding-of-the-process-correct
        fuel_gauge
            .write_control_command(CONTROL_GG_CHGRCTL_ENABLE_SUBCOMMAND)
            .await?;

        let reg = fuel_gauge
            .data_flash()
            .read_operation_configuration()
            .await?;
        info!(
            "Fuel gauge operation configuration: [{=u8:b}, {=u8:b}]",
            reg[0], reg[1]
        );

        let temps = fuel_gauge
            .data_flash()
            .read_byte_bit(SUBCLASS_REGISTERS, 1, 0)
            .await
            .unwrap();
        if temps {
            // Required because no external thermistor was attatched.
            warn!(
                "Fuel gauge is currently configured to use external thermistor, updating to internal..."
            );

            fuel_gauge
                .data_flash()
                .write_byte(SUBCLASS_REGISTERS, 1, 0b01100110)
                .await
                .unwrap();

            let reg = fuel_gauge
                .data_flash()
                .read_operation_configuration()
                .await
                .unwrap();
            info!(
                "Updated fuel gauge operation configuration: [{=u8:b}, {=u8:b}]",
                reg[0], reg[1]
            );
        }

        // Defualt: 0x1C
        let charge_cmd = fuel_gauge
            .data_flash()
            .read_byte_bit(CHARGER_SUBCLASS_CHARGER_CONTROL_CONFIGURATION, 0, 5)
            .await
            .unwrap();
        if !charge_cmd {
            // Required because charger doesn't have it's own thermistor,
            // so it requires the fuel gauge connection.
            warn!(
                "Fuel gauge currently requires a command to contact the charger, updating to not require command..."
            );

            fuel_gauge
                .data_flash()
                .write_byte(
                    CHARGER_SUBCLASS_CHARGER_CONTROL_CONFIGURATION,
                    0,
                    // RSVD USB_IN_DEF CMD_NOT_REQ CHGTRM_HIZ STEP_EN SOH_EN DEFAULT_OVRD BYPASS
                    0b01111100,
                )
                .await
                .unwrap();

            let reg = fuel_gauge
                .data_flash()
                .read_byte(CHARGER_SUBCLASS_CHARGER_CONTROL_CONFIGURATION, 0)
                .await
                .unwrap();
            info!("Updated fuel gauge charger configuration: [{=u8:b}]", reg);
        }

        if let Ok(design_capacity) = fuel_gauge.read_extended_design_capacity().await {
            info!("Current design capacity: {}mAh", design_capacity);

            if design_capacity != 4200 {
                warn!(
                    "Incorrect design capacity {}mAh, updating to 6000mAh",
                    design_capacity
                );

                let bytes: [u8; 2] = 4200_u16.to_le_bytes();

                fuel_gauge
                    .data_flash()
                    .write_byte(SUBCLASS_DATA, 8, bytes[0])
                    .await?;
                fuel_gauge
                    .data_flash()
                    .write_byte(SUBCLASS_DATA, 8, bytes[1])
                    .await?;
            }
        }

        // Print some debug info to the log in case it's required later.

        info!("Attempting to read fuel gauge flags...");
        if let Ok(flags) = fuel_gauge.read_flags().await {
            info!(
                "Fuel gauge flags: {:b} {:b} ({:b})",
                flags.to_le_bytes()[0],
                flags.to_le_bytes()[1],
                flags
            );
        } else {
            error!("Failed to read fuel gauge flags!");
        }

        info!("Attempting to read fuel gauge flags...");
        if let Ok(design_capacity) = fuel_gauge.read_extended_design_capacity().await {
            info!("Design capacity: {}", design_capacity);
        } else {
            error!("Failed to read design capacity!");
        }

        info!("Attempting to read fuel gauge internal temp...");
        if let Ok(temp) = fuel_gauge.read_internal_temperature().await {
            info!("Fuel gauge internal temp: {}", temp);
        } else {
            error!("Failed to read fuel gauge temp!");
        }

        info!("Attempting to read fuel gauge remaining capacity...");
        if let Ok(temp) = fuel_gauge.read_remaining_capacity().await {
            info!("Fuel gauge remaining capacity: {}mah", temp);
        } else {
            error!("Failed to read fuel gauge remaining capacity!");
        }

        if let Ok(temp) = fuel_gauge.read_instantaneous_current_reading().await {
            info!("Current current draw: {}mA", temp);
        } else {
            error!("Failed to read fuel gauge current draw!");
        }

        if let Ok(charger_fault) = fuel_gauge.read_charger_fault_register().await {
            let raw = charger_fault.into_storage();
            if raw != 0 {
                error!("Charger fault detected!")
            }

            info!("Charger fault register: {:b}", raw);
        } else {
            error!("Failed to read charger fault register!");
        }

        if let Ok(charger_status) = fuel_gauge.read_charger_status_register().await {
            let raw = charger_status.into_storage();

            info!("Charger status register: {:b}", raw);
        } else {
            error!("Failed to read charger status register!");
        }

        Ok(Self { fuel_gauge })
    }

    pub fn fuel_gauge_ref(
        &mut self,
    ) -> &'_ mut Bq27531<
        'a,
        asynch::i2c::I2cDevice<'a, CriticalSectionRawMutex, I2c<'static, Async, i2c::Master>>,
        Delay,
    > {
        &mut self.fuel_gauge
    }
}
