//! Booster NGFW NVM channel settings
//!
//! # Copyright
//! Copyright (C) 2020 QUARTIQ GmbH - All Rights Reserved
//! Unauthorized usage, editing, or copying is strictly prohibited.
//! Proprietary and confidential.

use super::{SemVersion, SinaraBoardId, SinaraConfiguration};
use crate::{hardware::I2cProxy, linear_transformation::LinearTransformation, Error};
use microchip_24aa02e48::Microchip24AA02E48;
use miniconf::Miniconf;

/// The expected semver of the BoosterChannelSettings. This version must be updated whenever the
/// `VersionedChannelData` layout is updated.
const EXPECTED_VERSION: SemVersion = SemVersion {
    major: 1,
    minor: 0,
    patch: 1,
};

/// Represents booster channel-specific configuration values.
#[derive(Miniconf, serde::Serialize, serde::Deserialize, Clone, PartialEq)]
pub struct ChannelSettings {
    pub output_interlock_threshold: f32,
    pub bias_voltage: f32,
    pub enabled: bool,
    pub input_power_transform: LinearTransformation,
    pub output_power_transform: LinearTransformation,
    pub reflected_power_transform: LinearTransformation,

    // Note: This field is not persisted to external memory.
    #[serde(skip)]
    pub output_disable: bool,
}

impl Default for ChannelSettings {
    /// Generate default booster channel data.
    fn default() -> Self {
        Self {
            output_interlock_threshold: 0.0,
            bias_voltage: -3.2,
            enabled: false,
            output_disable: false,

            // When operating at 100MHz, the power detectors specify the following output
            // characteristics for -10 dBm to 10 dBm (the equation uses slightly different coefficients
            // for different power levels and frequencies):
            //
            // dBm = V(Vout) / .035 V/dB - 35.6 dBm
            //
            // All of the power meters are preceded by attenuators which are incorporated in
            // the offset.
            output_power_transform: LinearTransformation::new(1.0 / 0.035, -35.6 + 19.8 + 10.0),
            // The input power and reflected power detectors are then passed through an
            // op-amp with gain 1.5x - this modifies the slope from 35mV/dB to 52.5mV/dB
            reflected_power_transform: LinearTransformation::new(
                1.0 / 1.5 / 0.035,
                -35.6 + 19.8 + 10.0,
            ),
            input_power_transform: LinearTransformation::new(1.0 / 1.5 / 0.035, -35.6 + 8.9),
        }
    }
}

/// Represents versioned channel-specific configuration values.
#[derive(serde::Serialize, serde::Deserialize)]
struct VersionedChannelData {
    version: SemVersion,
    settings: ChannelSettings,
}

impl Default for VersionedChannelData {
    fn default() -> Self {
        Self {
            version: EXPECTED_VERSION,
            settings: ChannelSettings::default(),
        }
    }
}

impl VersionedChannelData {
    /// Construct booster configuration data from serialized `board_data` from a
    /// SinaraConfiguration.
    ///
    /// # Args
    /// * `data` - The data to deserialize from.
    ///
    /// # Returns
    /// The configuration if deserialization was successful. Otherwise, returns an error.
    pub fn deserialize(data: &[u8; 64]) -> Result<Self, Error> {
        let data: VersionedChannelData = postcard::from_bytes(data).or(Err(Error::Invalid))?;

        // Validate configuration parameters.
        if data.settings.bias_voltage < -3.3 || data.settings.bias_voltage > 0.0 {
            return Err(Error::Invalid);
        }

        // Validate the version of the settings.
        if !EXPECTED_VERSION.is_compatible(&data.version) {
            return Err(Error::Invalid);
        }

        Ok(data)
    }

    /// Serialize the booster config into a sinara configuration for storage into EEPROM.
    ///
    /// # Args
    /// * `config` - The sinara configuration to serialize the booster configuration into.
    pub fn serialize_into(&self, config: &mut SinaraConfiguration) {
        let mut buffer: [u8; 64] = [0; 64];
        let serialized = postcard::to_slice(self, &mut buffer).unwrap();
        config.board_data[..serialized.len()].copy_from_slice(serialized);
    }
}

/// Represents the booster RF channel settings.
pub struct BoosterChannelSettings {
    eeprom: Microchip24AA02E48<I2cProxy>,
    data: VersionedChannelData,
}

impl BoosterChannelSettings {
    /// Construct the booster RF channel settings from the RF module EEPROM.
    ///
    /// # Args
    /// * `eeprom` - The EEPROM installed on the RF module.
    ///
    /// # Returns
    /// The settings found on the RF module.
    pub fn new(eeprom: Microchip24AA02E48<I2cProxy>) -> Self {
        let mut settings = Self {
            eeprom,
            data: VersionedChannelData::default(),
        };

        match settings.load_config() {
            Ok(config) => {
                // If we loaded sinara configuration, deserialize the board data.
                match VersionedChannelData::deserialize(&config.board_data) {
                    Ok(data) => settings.data = data,

                    Err(_) => {
                        settings.data = VersionedChannelData::default();
                        settings.save();
                    }
                }
            }

            // If we failed to load configuration, use a default config.
            Err(_) => {
                settings.data = VersionedChannelData::default();
                settings.save();
            }
        };

        settings
    }

    /// Save the configuration settings to EEPROM for retrieval.
    pub fn save(&mut self) {
        let mut config = match self.load_config() {
            Err(_) => SinaraConfiguration::default(SinaraBoardId::RfChannel),
            Ok(config) => config,
        };

        self.data.serialize_into(&mut config);
        config.update_crc32();
        self.save_config(&config);
    }

    /// Mutably borrow the channel settings.
    pub fn settings_mut(&mut self) -> &mut ChannelSettings {
        &mut self.data.settings
    }

    pub fn settings(&self) -> &ChannelSettings {
        &self.data.settings
    }

    /// Load device settings from EEPROM.
    ///
    /// # Returns
    /// The loaded sinara configuration.
    fn load_config(&mut self) -> Result<SinaraConfiguration, Error> {
        // Read the sinara-config from memory.
        let mut sinara_config: [u8; 256] = [0; 256];
        self.eeprom.read(0, &mut sinara_config).unwrap();

        SinaraConfiguration::try_deserialize(sinara_config)
    }

    fn save_config(&mut self, config: &SinaraConfiguration) {
        // Save the updated configuration to EEPROM.
        let mut serialized = [0u8; 128];
        config.serialize_into(&mut serialized);
        self.eeprom.write(0, &serialized).unwrap();
    }
}
