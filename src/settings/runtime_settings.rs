//! Booster NGFW runtime settings

use super::eeprom::rf_channel::ChannelSettings;
use miniconf::{Leaf, Tree};

use crate::hardware::chassis_fans::DEFAULT_FAN_SPEED;
use crate::net::mqtt_control::DEFAULT_TELEMETRY_PERIOD_SECS;

#[derive(Clone, Debug, Tree)]
pub struct RuntimeSettings {
    pub channel: [Option<ChannelSettings>; 8],

    /// The normalized fan speed. 1.0 corresponds to 100% on and 0.0 corresponds to completely
    /// off.
    #[tree(validate=self.validate_fan_speed)]
    pub fan_speed: Leaf<f32>,

    /// The configured telemetry period in seconds.
    pub telemetry_period: Leaf<u32>,
}

impl Default for RuntimeSettings {
    fn default() -> Self {
        Self {
            channel: [None; 8],
            fan_speed: Leaf::from(DEFAULT_FAN_SPEED),
            telemetry_period: Leaf::from(DEFAULT_TELEMETRY_PERIOD_SECS),
        }
    }
}

impl RuntimeSettings {
    fn validate_fan_speed(&mut self, depth: usize) -> Result<usize, &'static str> {
        *self.fan_speed = self.fan_speed.clamp(0.0, 1.0);
        Ok(depth)
    }

    pub fn reset(&mut self) {
        for channel in self.channel.iter_mut().flatten() {
            *channel = ChannelSettings::default();
        }

        *self = Self {
            channel: self.channel,
            ..Default::default()
        }
    }
}
