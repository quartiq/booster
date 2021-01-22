//! Booster NGFW NVM settings
//!
//! # Copyright
//! Copyright (C) 2020 QUARTIQ GmbH - All Rights Reserved
//! Unauthorized usage, editing, or copying is strictly prohibited.
//! Proprietary and confidential.
use crate::{Eeprom, Error};
use heapless::{consts, String};
use minimq::embedded_nal::Ipv4Addr;
use serde::{Deserialize, Serialize};

#[cfg(feature = "phy_w5500")]
use w5500::MacAddress;

#[cfg(feature = "phy_enc424j600")]
use smoltcp::wire::EthernetAddress as MacAddress;

use super::{SemVersion, SinaraBoardId, SinaraConfiguration};

use core::fmt::Write;

/// The expected semver of the BoosterChannelSettings. This version must be updated whenever the
/// `BoosterMainBoardData` layout is updated.
const EXPECTED_VERSION: SemVersion = SemVersion {
    major: 1,
    minor: 0,
    patch: 1,
};

fn array_to_addr(addr: &[u8; 4]) -> Ipv4Addr {
    Ipv4Addr::new(addr[0], addr[1], addr[2], addr[3])
}

fn identifier_is_valid<'a>(id: &'a str) -> bool {
    id.len() <= 23 && id.chars().all(|x| x.is_alphanumeric())
}

/// Represents booster mainboard-specific configuration values.
#[derive(Serialize, Deserialize)]
struct BoosterMainBoardData {
    version: SemVersion,
    ip_address: [u8; 4],
    broker_address: [u8; 4],
    gateway_address: [u8; 4],
    netmask: [u8; 4],
    identifier: [u8; 23],
    id_size: usize,
}

impl BoosterMainBoardData {
    /// Generate default booster configuration data given the device EUI48.
    ///
    /// # Args
    /// * `eui48` - The EUI48 identifier of the booster mainboard.
    pub fn default(eui48: &[u8; 6]) -> Self {
        let mut name: String<consts::U23> = String::new();
        write!(
            &mut name,
            "booster{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}",
            eui48[0], eui48[1], eui48[2], eui48[3], eui48[4], eui48[5]
        )
        .unwrap();

        let mut id: [u8; 23] = [0; 23];
        id[..name.len()].copy_from_slice(name.as_str().as_bytes());

        Self {
            version: EXPECTED_VERSION,
            ip_address: [10, 0, 0, 1],
            broker_address: [10, 0, 0, 2],
            gateway_address: [10, 0, 0, 0],
            netmask: [255, 255, 255, 0],
            identifier: id,
            id_size: name.len(),
        }
    }

    /// Construct booster configuration data from serialized `board_data` from a
    /// SinaraConfiguration.
    ///
    /// # Args
    /// * `data` - The data to deserialize from.
    ///
    /// # Returns
    /// The configuration if deserialization was successful. Otherwise, returns an error.
    pub fn deserialize(data: &[u8; 64]) -> Result<Self, Error> {
        let config: BoosterMainBoardData = postcard::from_bytes(data).unwrap();

        // Validate the version of the settings.
        if !EXPECTED_VERSION.is_compatible(&config.version) {
            return Err(Error::Invalid);
        }

        // Validate configuration parameters.
        let identifier = core::str::from_utf8(config.id()).map_err(|_| Error::Invalid)?;
        if identifier_is_valid(identifier) == false {
            return Err(Error::Invalid);
        }

        Ok(config)
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

    /// Get the IP address of the Booster.
    pub fn ip(&self) -> Ipv4Addr {
        array_to_addr(&self.ip_address)
    }

    /// Get the MQTT broker address of Booster.
    pub fn broker(&self) -> Ipv4Addr {
        array_to_addr(&self.broker_address)
    }

    /// Get the gateway address of Booster.
    pub fn gateway(&self) -> Ipv4Addr {
        array_to_addr(&self.gateway_address)
    }

    /// Get the subnet mask of Booster.
    pub fn subnet(&self) -> Ipv4Addr {
        array_to_addr(&self.netmask)
    }

    /// Get the MQTT identifier of Booster.
    pub fn id<'a>(&'a self) -> &'a [u8] {
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
    pub fn set_id<'a>(&mut self, id: &'a str) -> Result<(), Error> {
        // TODO: Verify the ID is valid.
        if identifier_is_valid(id) == false {
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

    /// Update the IP address of Booster.
    pub fn set_ip_address(&mut self, addr: Ipv4Addr) {
        self.ip_address = addr.octets();
    }

    /// Update the IP address of the gateway.
    pub fn set_gateway(&mut self, addr: Ipv4Addr) {
        self.gateway_address = addr.octets();
    }

    /// Update the subnet mask of Booster.
    pub fn set_netmask(&mut self, addr: Ipv4Addr) {
        self.netmask = addr.octets();
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

        let mut settings = Self {
            board_data: BoosterMainBoardData::default(&mac),
            eui48: mac,
            dirty: false,
            eeprom,
        };

        // Load the sinara configuration from EEPROM.
        match settings.load_config() {
            Ok(config) => match BoosterMainBoardData::deserialize(&config.board_data) {
                Ok(data) => settings.board_data = data,

                Err(_) => {
                    settings.board_data = BoosterMainBoardData::default(&settings.eui48);
                    settings.save();
                }
            },

            // If we failed to load configuration, use a default config.
            Err(_) => {
                settings.board_data = BoosterMainBoardData::default(&settings.eui48);
                settings.save();
            }
        };

        settings
    }

    /// Override MAC address during initialization
    pub fn set_mac(mut self, mac: [u8; 6]) -> Self {
        // MAC address is designed to be unchanged in EEPROM
        self.eui48.copy_from_slice(&mac);
        self
    }

    /// Save the configuration settings to EEPROM for retrieval.
    pub fn save(&mut self) {
        let mut config = match self.load_config() {
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

    /// Get the Booster unique identifier.
    pub fn id<'a>(&'a self) -> &'a str {
        core::str::from_utf8(self.board_data.id()).unwrap()
    }

    /// Get the Booster MAC address.
    pub fn mac(&self) -> MacAddress {
        #[cfg(feature = "phy_w5500")]
        {
            MacAddress::from_bytes(self.eui48)
        }

        #[cfg(feature = "phy_enc424j600")]
        {
            MacAddress::from_bytes(&self.eui48)
        }
    }

    /// Get the Booster IP address.
    pub fn ip(&self) -> Ipv4Addr {
        self.board_data.ip()
    }

    /// Get the MQTT broker IP address.
    pub fn broker(&self) -> Ipv4Addr {
        self.board_data.broker()
    }

    /// Get the network gateway.
    pub fn gateway(&self) -> Ipv4Addr {
        self.board_data.gateway()
    }

    /// Get the network subnet.
    pub fn subnet(&self) -> Ipv4Addr {
        self.board_data.subnet()
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

    /// Update the Booster IP address.
    pub fn set_ip_address(&mut self, addr: Ipv4Addr) {
        self.dirty = true;
        self.board_data.set_ip_address(addr);
        self.save();
    }

    /// Update the booster gateway.
    pub fn set_gateway(&mut self, addr: Ipv4Addr) {
        self.dirty = true;
        self.board_data.set_gateway(addr);
        self.save();
    }

    /// Update the booster net mask.
    pub fn set_netmask(&mut self, addr: Ipv4Addr) {
        self.dirty = true;
        self.board_data.set_netmask(addr);
        self.save();
    }

    /// Update the booster MQTT client identifier.
    pub fn set_id<'a>(&mut self, id: &'a str) -> Result<(), Error> {
        self.board_data.set_id(id).and_then(|_| {
            self.dirty = true;
            self.save();
            Ok(())
        })
    }
}
