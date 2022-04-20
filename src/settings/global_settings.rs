//! Booster NGFW NVM settings
//!
//! # Copyright
//! Copyright (C) 2020 QUARTIQ GmbH - All Rights Reserved
//! Unauthorized usage, editing, or copying is strictly prohibited.
//! Proprietary and confidential.
use crate::{hardware::Eeprom, Error};
use heapless::String;
use minimq::embedded_nal::Ipv4Addr;
use serde::{Deserialize, Serialize};

use crate::hardware::chassis_fans::DEFAULT_FAN_SPEED;

#[cfg(feature = "phy_w5500")]
use w5500::MacAddress;

#[cfg(feature = "phy_enc424j600")]
use smoltcp_nal::smoltcp::wire::EthernetAddress as MacAddress;

use super::{SemVersion, SinaraBoardId, SinaraConfiguration};

use core::fmt::Write;

/// The expected semver of the BoosterChannelSettings. This version must be updated whenever the
/// `BoosterMainBoardData` layout is updated.
const EXPECTED_VERSION: SemVersion = SemVersion {
    major: 1,
    minor: 1,
    patch: 0,
};

fn array_to_addr(addr: &[u8; 4]) -> Ipv4Addr {
    Ipv4Addr::new(addr[0], addr[1], addr[2], addr[3])
}

fn identifier_is_valid(id: &str) -> bool {
    id.len() <= 23 && id.chars().all(|x| x.is_alphanumeric() || x == '-')
}

/// Represents booster mainboard-specific configuration values.
#[derive(Serialize, Deserialize)]
struct BoosterMainBoardData {
    version: SemVersion,

    // Note: The IP address, gateway, and netmask are unused, but left here to maintain backwards compatibility with
    // settings version v1.0.0
    _unused_ip_address: [u8; 4],
    broker_address: [u8; 4],
    _unused_gateway_address: [u8; 4],
    _unused_netmask: [u8; 4],
    identifier: [u8; 23],
    id_size: usize,
    fan_speed: f32,
}

impl BoosterMainBoardData {
    /// Generate default booster configuration data given the device EUI48.
    ///
    /// # Args
    /// * `eui48` - The EUI48 identifier of the booster mainboard.
    pub fn default(eui48: &[u8; 6]) -> Self {
        let mut name: String<23> = String::new();
        write!(
            &mut name,
            "{:02x}-{:02x}-{:02x}-{:02x}-{:02x}-{:02x}",
            eui48[0], eui48[1], eui48[2], eui48[3], eui48[4], eui48[5]
        )
        .unwrap();

        let mut id: [u8; 23] = [0; 23];
        id[..name.len()].copy_from_slice(name.as_str().as_bytes());

        Self {
            version: EXPECTED_VERSION,
            _unused_ip_address: [10, 0, 0, 1],
            broker_address: [10, 0, 0, 2],
            _unused_gateway_address: [10, 0, 0, 0],
            _unused_netmask: [255, 255, 255, 0],
            identifier: id,
            id_size: name.len(),
            fan_speed: DEFAULT_FAN_SPEED,
        }
    }

    /// Construct booster configuration data from serialized `board_data` from a
    /// SinaraConfiguration.
    ///
    /// # Args
    /// * `data` - The data to deserialize from.
    ///
    /// # Returns
    /// The configuration if deserialization was successful along with a bool indicating if the
    /// configuration was automatically upgraded. Otherwise, returns an error.
    pub fn deserialize(data: &[u8; 64]) -> Result<(Self, bool), Error> {
        let mut config: BoosterMainBoardData = postcard::from_bytes(data).unwrap();
        let mut modified = false;

        // Check if the stored EEPROM version is older (or incompatible)
        if !EXPECTED_VERSION.is_compatible_with(&config.version) {
            // If the stored config is compatible with the new version (e.g. older), we can upgrade
            // the config version in a backward compatible manner by adding in new parameters and
            // writing it back.
            if config.version.is_compatible_with(&EXPECTED_VERSION) {
                log::info!("Adding default fan speed setting");
                config.fan_speed = DEFAULT_FAN_SPEED;
                config.version = EXPECTED_VERSION;
                modified = true;
            } else {
                // The version stored in EEPROM is some future version that we don't understand.
                return Err(Error::Invalid);
            }
        }

        // Validate configuration parameters.
        let identifier = core::str::from_utf8(config.id()).map_err(|_| Error::Invalid)?;
        if !identifier_is_valid(identifier) {
            return Err(Error::Invalid);
        }

        Ok((config, modified))
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

    /// Get the MQTT broker address of Booster.
    pub fn broker(&self) -> Ipv4Addr {
        array_to_addr(&self.broker_address)
    }

    /// Get the MQTT identifier of Booster.
    pub fn id(&self) -> &[u8] {
        &self.identifier[..self.id_size]
    }

    /// Set the MQTT ID of Booster.
    ///
    /// # Args
    /// * `id` - The new MQTT id. This must conform with MQTT identifier standards. That means that
    ///   it must be 23 characters or shorter and contain only alphanumeric values.
    ///
    /// # Returns
    /// Ok if the update was successful. Otherwise, returns an error.
    pub fn set_id(&mut self, id: &str) -> Result<(), Error> {
        if !identifier_is_valid(id) {
            return Err(Error::Invalid);
        }

        let len = id.as_bytes().len();
        self.identifier[..len].copy_from_slice(id.as_bytes());
        self.id_size = len;

        Ok(())
    }

    /// Update the MQTT broker IP address of Booster.
    pub fn set_broker(&mut self, addr: Ipv4Addr) {
        self.broker_address = addr.octets();
    }
}

/// Booster device-wide configurable settings.
pub struct BoosterSettings {
    board_data: BoosterMainBoardData,
    eui48: [u8; 6],
    eeprom: Eeprom,
    dirty: bool,
}

impl BoosterSettings {
    /// Load booster settings from external EEPROM storage.
    ///
    /// # Args
    /// * `eeprom` - The EEPROM used to store the configuration.
    pub fn new(mut eeprom: Eeprom) -> Self {
        let mut mac: [u8; 6] = [0; 6];
        eeprom.read_eui48(&mut mac).unwrap();

        // Load the sinara configuration from EEPROM.
        let (board_data, write_back) = Self::load_config(&mut eeprom)
            .and_then(|config| BoosterMainBoardData::deserialize(&config.board_data))
            .unwrap_or((BoosterMainBoardData::default(&mac), true));

        let mut settings = Self {
            board_data,
            eui48: mac,
            dirty: false,
            eeprom,
        };

        if write_back {
            settings.save();
        }

        settings
    }

    /// Save the configuration settings to EEPROM for retrieval.
    pub fn save(&mut self) {
        let mut config = match Self::load_config(&mut self.eeprom) {
            Err(_) => SinaraConfiguration::default(SinaraBoardId::Mainboard),
            Ok(config) => config,
        };

        self.board_data.serialize_into(&mut config);
        config.update_crc32();
        self.save_config(&config);
    }

    /// Load device settings from EEPROM.
    ///
    /// # Returns
    /// Ok(settings) if the settings loaded successfully. Otherwise, Err(settings), where `settings`
    /// are default values.
    fn load_config(eeprom: &mut Eeprom) -> Result<SinaraConfiguration, Error> {
        // Read the sinara-config from memory.
        let mut sinara_config: [u8; 256] = [0; 256];
        eeprom.read(0, &mut sinara_config).unwrap();

        SinaraConfiguration::try_deserialize(sinara_config)
    }

    fn save_config(&mut self, config: &SinaraConfiguration) {
        // Save the updated configuration to EEPROM.
        let mut serialized = [0u8; 128];
        config.serialize_into(&mut serialized);
        self.eeprom.write(0, &serialized).unwrap();
    }

    /// Get the Booster unique identifier.
    pub fn id(&self) -> &str {
        core::str::from_utf8(self.board_data.id()).unwrap()
    }

    /// Get the Booster MAC address.
    pub fn mac(&self) -> MacAddress {
        MacAddress { octets: self.eui48 }
    }

    /// Get the saved Booster fan speed.
    pub fn fan_speed(&self) -> f32 {
        self.board_data.fan_speed
    }

    /// Set the default fan speed of the device.
    pub fn set_fan_speed(&mut self, fan_speed: f32) {
        self.board_data.fan_speed = fan_speed.clamp(0.0, 1.0);
        self.dirty = true;
        self.save();
    }

    /// Get the MQTT broker IP address.
    pub fn broker(&self) -> Ipv4Addr {
        self.board_data.broker()
    }

    /// Check if current settings differ from active (executing) settings.
    pub fn are_dirty(&self) -> bool {
        self.dirty
    }

    /// Update the broker IP address.
    pub fn set_broker(&mut self, addr: Ipv4Addr) {
        self.dirty = true;
        self.board_data.set_broker(addr);
        self.save();
    }

    /// Update the booster MQTT client identifier.
    pub fn set_id(&mut self, id: &str) -> Result<(), Error> {
        self.board_data.set_id(id).map(|_| {
            self.dirty = true;
            self.save()
        })
    }
}
