//! Booster NGFW runtime settings

use super::eeprom::rf_channel::ChannelSettings;
use miniconf::{Leaf, Tree};

use crate::hardware::chassis_fans::DEFAULT_FAN_SPEED;
use crate::net::mqtt_control::DEFAULT_TELEMETRY_PERIOD_SECS;

mod validate_fan_speed {
    pub use miniconf::{
        leaf::{self, *},
        Keys, SerdeError,
    };
    use serde::Deserializer;

    pub fn deserialize_by_key<'de, D: Deserializer<'de>>(
        value: &mut f32,
        keys: impl Keys,
        de: D,
    ) -> Result<(), SerdeError<D::Error>> {
        leaf::deserialize_by_key(value, keys, de)?;

        *value = value.clamp(0.0, 1.0);

        Ok(())
    }
}

#[derive(Clone, Debug, Tree)]
pub struct RuntimeSettings {
    pub channel: [Option<ChannelSettings>; 8],

    /// The normalized fan speed. 1.0 corresponds to 100% on and 0.0 corresponds to completely
    /// off.
    #[tree(with=validate_fan_speed)]
    pub fan_speed: Leaf<f32>,

    /// The configured telemetry period in seconds.
    pub telemetry_period: Leaf<u32>,
}

impl Default for RuntimeSettings {
    fn default() -> Self {
        Self {
            channel: [None; 8],
            fan_speed: Leaf(DEFAULT_FAN_SPEED),
            telemetry_period: Leaf(DEFAULT_TELEMETRY_PERIOD_SECS),
        }
    }
}

impl RuntimeSettings {
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
