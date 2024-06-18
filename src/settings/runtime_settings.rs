//! Booster NGFW runtime settings

use super::eeprom::rf_channel::ChannelSettings;
use crate::{hardware, net};
use miniconf::Tree;

#[derive(Clone, Debug, Tree)]
pub struct RuntimeSettings {
    #[tree(depth = 3)]
    pub channel: [Option<ChannelSettings>; 8],

    /// The normalized fan speed. 1.0 corresponds to 100% on and 0.0 corresponds to completely
    /// off.
    #[tree(validate=Self::validate_fans)]
    pub fan_speed: f32,

    /// The configured telemetry period in seconds.
    pub telemetry_period: u32,
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
    fn validate_fans(&mut self, new: f32) -> Result<f32, &'static str> {
        if (0.0..=1.0).contains(&new) {
            Ok(new)
        } else {
            Err("Invalid fan speed. Must be within range [0, 1.0]")
        }
    }
}
