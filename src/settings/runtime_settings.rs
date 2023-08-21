//! Booster NGFW runtime settings

use super::channel_settings::ChannelSettings;
use crate::{
    hardware::{self, platform, Channel},
    net,
};
use miniconf::Miniconf;

#[derive(Clone, Miniconf)]
pub struct RuntimeSettings {
    #[miniconf(defer(3))]
    pub channel: [Option<ChannelSettings>; 8],

    /// The normalized fan speed. 1.0 corresponds to 100% on and 0.0 corresponds to completely
    /// off.
    pub fan_speed: f32,

    /// The configured telemetry period in seconds.
    pub telemetry_period: u64,
}

impl Default for RuntimeSettings {
    fn default() -> Self {
        Self {
            channel: [None; 8],
            fan_speed: hardware::chassis_fans::DEFAULT_FAN_SPEED,
            telemetry_period: net::mqtt_control::DEFAULT_TELEMETRY_PERIOD_SECS,
        }
    }
}

impl RuntimeSettings {
    pub fn handle_update(
        _: &str,
        settings: &mut Self,
        new_settings: &Self,
    ) -> Result<(), &'static str> {
        for idx in enum_iterator::all::<Channel>() {
            if let Some(settings) = new_settings.channel[idx as usize].as_ref() {
                // Check that the interlock thresholds are sensible.
                if settings.output_interlock_threshold > platform::MAX_OUTPUT_POWER_DBM {
                    return Err("Interlock threshold too high");
                }

                // Validate bias voltage.
                if !(0.0..=platform::BIAS_DAC_VCC).contains(&(-1.0 * settings.bias_voltage)) {
                    return Err("Bias voltage out of range");
                }

                // Validate that the output interlock threshold voltage (after mapping) is actually
                // configurable on the DAC.
                let output_interlock_voltage = settings
                    .output_power_transform
                    .invert(settings.output_interlock_threshold);
                if !(0.00..=ad5627::MAX_VOLTAGE).contains(&output_interlock_voltage) {
                    return Err("Output interlock threshold voltage out of range");
                }
            }
        }

        if !(0.0..=1.0).contains(&new_settings.fan_speed) {
            return Err("Invalid fan speed");
        }

        *settings = new_settings.clone();
        Ok(())
    }
}
