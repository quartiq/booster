//! Booster NGFW Application
//!
//! # Copyright
//! Copyright (C) 2020 QUARTIQ GmbH - All Rights Reserved
//! Unauthorized usage, editing, or copying is strictly prohibited.
//! Proprietary and confidential.
use super::{Eeprom, Error};
use heapless::{consts, String};
use serde::{Deserialize, Serialize};
use serde_big_array::big_array;
use w5500::{Ipv4Addr, MacAddress};

use core::fmt::Write;

big_array! {
    BigArray;
    +122,
}

/// The standard EEPROM layout for all Sinara hardware. Booster-specific configuration is stored in
/// `board_data`.
#[derive(Serialize, Deserialize)]
struct SinaraConfiguration {
    crc32: u32,
    magic: u16,
    pub name: [u8; 10],
    pub board_id: u16,
    pub format_rev: u8,
    pub major: u8,
    pub minor: u8,
    pub variant: u8,
    pub port: u8,
    pub vendor: u8,
    pub vendor_data: [u8; 8],
    pub project_data: [u8; 16],
    pub user_data: [u8; 16],
    #[serde(with = "BigArray")]
    pub board_data: [u8; 64],
    #[serde(with = "BigArray")]
    _padding: [u8; 122],
    pub eui48: [u8; 6],
}

impl SinaraConfiguration {
    /// Attempt to deserialize sinara configuration data from the raw EEPROM content.
    ///
    /// # Returns
    /// The configuration if it was properly decoded. Otherwise, an error.
    pub fn deserialize(data: [u8; 256]) -> Result<SinaraConfiguration, Error> {
        let config: SinaraConfiguration = postcard::from_bytes(&data).unwrap();

        if config.crc32 != config.calculate_crc32() || config.magic != 0x391e {
            Err(Error::Invalid)
        } else {
            Ok(config)
        }
    }

    /// Generate a default sinara EEPROM configuration.
    ///
    /// # Args
    /// * `name` - The name to store in the EEPOM configuration.
    pub fn default(name: &[u8]) -> SinaraConfiguration {
        assert!(name.len() <= 10);

        let mut name_info: [u8; 10] = [0; 10];
        name_info[..name.len()].copy_from_slice(name);

        let mut config = SinaraConfiguration {
            // Will be updated later.
            crc32: 0,

            magic: 0x391e,
            name: name_info,
            board_id: 0,
            format_rev: 0,
            major: 1,
            minor: 0,
            variant: 0,
            port: 0,

            // Specifies QUARTIQ
            vendor: 3,
            vendor_data: [0; 8],

            project_data: [0; 16],
            user_data: [0; 16],
            board_data: [0; 64],

            // Padding and EUI48 are DONT-CARE - this is a read-only memory region.
            _padding: [0xFF; 122],
            eui48: [0xFF; 6],
        };

        config.update_crc32();

        config
    }

    /// Serialize the configuration into an EEPROM buffer.
    ///
    /// # Args
    /// * `buf` - The buffer to serialize into.
    ///
    /// # Returns
    /// The 128 byte writable portion of the configuration. This should be written to the first page
    /// of EEPROM.
    pub fn serialize_into<'a>(&self, buf: &'a mut [u8; 256]) -> &'a [u8] {
        // The configuration only actually allows the first 128 bytes to be programmed. The second
        // 128 bytes are ROM.
        &postcard::to_slice(&self, buf).unwrap()[..128]
    }

    /// Update the internal CRC32 of the configuration.
    pub fn update_crc32(&mut self) {
        self.crc32 = self.calculate_crc32()
    }

    fn calculate_crc32(&self) -> u32 {
        // TODO: Calculate using the zlib.crc32 algorithm at
        // https://github.com/madler/zlib/blob/master/crc32.c#L202-L234
        0xAAAA_BBBB
    }
}

fn array_to_addr(addr: &[u8; 4]) -> Ipv4Addr {
    Ipv4Addr::new(addr[0], addr[1], addr[2], addr[3])
}

fn identifier_is_valid<'a>(id: &'a str) -> bool {
    id.len() == 23 && id.chars().all(|x| x.is_alphanumeric())
}

/// Represents booster mainboard-specific configuration values.
#[derive(serde::Serialize, serde::Deserialize)]
struct BoosterMainBoardData {
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
            "booster{}{}{}{}{}{}",
            eui48[0], eui48[1], eui48[2], eui48[3], eui48[4], eui48[5]
        )
        .unwrap();

        let mut id: [u8; 23] = [0; 23];
        id[..name.len()].copy_from_slice(name.as_str().as_bytes());

        Self {
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
        let sinara_config = match settings.load_config() {
            Ok(config) => config,

            // If we failed to load configuration, we got a default. Save the default and continue
            // using it.
            Err(config) => {
                settings.save_config(&config);
                config
            }
        };

        // Next, deserialize the board configuration from the sinara configuration.
        match BoosterMainBoardData::deserialize(&sinara_config.board_data) {
            Ok(data) => settings.board_data = data,

            // If the deserialization fails, use default board data and update the sinara
            // configuration in memory.
            Err(_) => {
                settings.board_data = BoosterMainBoardData::default(&settings.eui48);
                settings.save();
            }
        };

        settings
    }

    /// Save the configuration settings to EEPROM for retrieval.
    pub fn save(&mut self) {
        let mut config = match self.load_config() {
            Err(config) => config,
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
    fn load_config(&mut self) -> Result<SinaraConfiguration, SinaraConfiguration> {
        // Read the sinara-config from memory.
        let mut sinara_config: [u8; 256] = [0; 256];
        self.eeprom.read(0, &mut sinara_config).unwrap();

        match SinaraConfiguration::deserialize(sinara_config) {
            Ok(config) => Ok(config),

            // If we failed to load data, provide default values.
            Err(_) => {
                let board_data = BoosterMainBoardData::default(&self.eui48);
                let mut config = SinaraConfiguration::default("Booster".as_bytes());
                board_data.serialize_into(&mut config);
                Err(config)
            }
        }
    }

    fn save_config(&mut self, config: &SinaraConfiguration) {
        // Save the updated configuration to EEPROM.
        let mut serialized: [u8; 256] = [0; 256];
        self.eeprom
            .write(0, config.serialize_into(&mut serialized))
            .unwrap();
    }

    /// Get the Booster unique identifier.
    pub fn id<'a>(&'a self) -> &'a str {
        core::str::from_utf8(self.board_data.id()).unwrap()
    }

    /// Get the Booster MAC address.
    pub fn mac(&self) -> MacAddress {
        MacAddress::from_bytes(self.eui48)
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
