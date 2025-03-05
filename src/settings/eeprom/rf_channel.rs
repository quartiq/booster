//! Booster NGFW NVM channel settings

use super::{
    sinara::{BoardId as SinaraBoardId, SinaraConfiguration},
    SemVersion,
};
use crate::{
    hardware::platform, hardware::I2cProxy, linear_transformation::LinearTransformation, Error,
};
use encdec::{Decode, DecodeOwned, Encode};
use enum_iterator::Sequence;
use microchip_24aa02e48::Microchip24AA02E48;
use miniconf::{Leaf, Tree};
use serde::{Deserialize, Serialize};

/// The expected semver of the BoosterChannelSettings. This version must be updated whenever the
/// `VersionedChannelData` layout is updated.
const EXPECTED_VERSION: SemVersion = SemVersion {
    major: 1,
    minor: 0,
    patch: 1,
};

/// Indicates the desired state of a channel.
#[derive(Serialize, Deserialize, Debug, Copy, Clone, PartialEq, Sequence)]
#[repr(u8)]
pub enum ChannelState {
    /// The channel should be turned off and power should be disconnected.
    Off = 0,

    /// The channel stages are powered and the RF switch is enabled.
    // For compatibility reasons, Enabled is stored with the value equivalent to "true"
    Enabled = 1,

    /// Stages are powered but RF switch is disabled. Used for bias current tuning.
    Powered = 2,
}

impl Encode for ChannelState {
    type Error = encdec::Error;

    fn encode_len(&self) -> Result<usize, Self::Error> {
        Ok(1)
    }

    fn encode(&self, buff: &mut [u8]) -> Result<usize, Self::Error> {
        if buff.is_empty() {
            return Err(encdec::Error::Length);
        }

        buff[0] = *self as u8;

        Ok(1)
    }
}

impl DecodeOwned for ChannelState {
    type Output = ChannelState;

    type Error = encdec::Error;

    fn decode_owned(buff: &[u8]) -> Result<(Self::Output, usize), Self::Error> {
        if buff.is_empty() {
            return Err(encdec::Error::Length);
        }

        for state in enum_iterator::all::<ChannelState>() {
            if state as u8 == buff[0] {
                return Ok((state, 1));
            }
        }

        Err(encdec::Error::Utf8)
    }
}

/// Represents booster channel-specific configuration values.
#[derive(Tree, Encode, Debug, Copy, Clone, PartialEq)]
pub struct ChannelSettings {
    // dBm
    #[tree(validate=self.validate_output_interlock_threshold)]
    pub output_interlock_threshold: Leaf<f32>,

    // V
    #[tree(validate=self.validate_bias_voltage)]
    pub bias_voltage: Leaf<f32>,

    pub state: Leaf<ChannelState>,

    pub input_power_transform: Leaf<LinearTransformation>,

    pub output_power_transform: Leaf<LinearTransformation>,

    pub reflected_power_transform: Leaf<LinearTransformation>,
}

impl DecodeOwned for ChannelSettings {
    type Output = ChannelSettings;

    type Error = encdec::Error;

    fn decode_owned(buff: &[u8]) -> Result<(Self::Output, usize), Self::Error> {
        #[derive(Debug, DecodeOwned)]
        struct ChannelSettingsDecoder {
            pub output_interlock_threshold: f32,
            pub bias_voltage: f32,
            pub state: ChannelState,
            pub input_power_transform: LinearTransformation,
            pub output_power_transform: LinearTransformation,
            pub reflected_power_transform: LinearTransformation,
        }

        let (inner, inner_len) = ChannelSettingsDecoder::decode_owned(buff)?;
        Ok((
            ChannelSettings {
                output_interlock_threshold: Leaf::from(inner.output_interlock_threshold),
                bias_voltage: Leaf::from(inner.bias_voltage),
                state: Leaf::from(inner.state),
                input_power_transform: Leaf::from(inner.input_power_transform),
                output_power_transform: Leaf::from(inner.output_power_transform),
                reflected_power_transform: Leaf::from(inner.reflected_power_transform),
            },
            inner_len,
        ))
    }
}

impl Default for ChannelSettings {
    /// Generate default booster channel data.
    fn default() -> Self {
        Self {
            // dBm
            output_interlock_threshold: Leaf::from(20.0),

            // V
            bias_voltage: Leaf::from(-3.2),

            state: Leaf::from(ChannelState::Off),

            // When operating at 100MHz, the power detectors specify the following output
            // characteristics for -10 dBm to 10 dBm:
            //
            // dBm = V(Vout) / .035 V/dB - 35.6 dBm
            //
            // All of the power meters are preceded by attenuators which are incorporated in
            // the offset.
            output_power_transform: Leaf::from(LinearTransformation::new(
                1.0 / 0.035,
                -35.6 + 19.8 + 10.0,
            )),

            // The input power and reflected power detectors have an op-amp gain of 1.5
            reflected_power_transform: Leaf::from(LinearTransformation::new(
                1.0 / 1.5 / 0.035,
                -35.6 + 19.8 + 10.0,
            )),

            input_power_transform: Leaf::from(LinearTransformation::new(
                1.0 / 1.5 / 0.035,
                -35.6 + 8.9,
            )),
        }
    }
}

impl ChannelSettings {
    fn validate_bias_voltage(&mut self, depth: usize) -> Result<usize, &'static str> {
        if (0.0..=platform::BIAS_DAC_VCC).contains(&-(*self.bias_voltage)) {
            Ok(depth)
        } else {
            Err("Bias voltage out of range")
        }
    }

    fn validate_output_interlock_threshold(&mut self, depth: usize) -> Result<usize, &'static str> {
        // Verify the output interlock is within acceptable values.
        if *self.output_interlock_threshold >= platform::MAX_OUTPUT_POWER_DBM {
            return Err("Output interlock threshold too high");
        }

        // Verify the interlock is mappable to a DAC threshold.
        if !(0.0..=ad5627::MAX_VOLTAGE).contains(
            &self
                .output_power_transform
                .invert(*self.output_interlock_threshold),
        ) {
            return Err("Output interlock threshold voltage out of range");
        }

        Ok(depth)
    }
}

/// Represents versioned channel-specific configuration values.
#[derive(Encode, DecodeOwned, Debug, Copy, Clone)]
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
        let (data, _) = VersionedChannelData::decode_owned(data).or(Err(Error::Invalid))?;

        // Validate configuration parameters.
        if *data.settings.bias_voltage < -3.3 || *data.settings.bias_voltage > 0.0 {
            return Err(Error::Invalid);
        }

        // Validate the version of the settings.
        if !EXPECTED_VERSION.is_compatible_with(&data.version) {
            return Err(Error::Invalid);
        }

        Ok(data)
    }

    /// Serialize the booster config into a sinara configuration for storage into EEPROM.
    ///
    /// # Args
    /// * `config` - The sinara configuration to serialize the booster configuration into.
    pub fn serialize_into(&self, config: &mut SinaraConfiguration) {
        // We will never store `Powered` in EEPROM, since this is never desired. Cache the current
        // power state while we serialize to ensure we only serialize Enabled and Off.
        let mut versioned_copy = *self;
        if *versioned_copy.settings.state == ChannelState::Powered {
            *versioned_copy.settings.state = ChannelState::Off;
        }

        let mut buffer: [u8; 64] = [0; 64];
        let len = versioned_copy.encode(&mut buffer).unwrap();
        config.board_data[..len].copy_from_slice(&buffer[..len]);
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

        settings.data = settings
            .load_config()
            .and_then(|config|
                // If we loaded sinara configuration, deserialize the board data.
                VersionedChannelData::deserialize(&config.board_data))
            .unwrap_or_default();

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
