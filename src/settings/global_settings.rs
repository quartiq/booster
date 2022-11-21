//! Booster NGFW NVM settings

use crate::{hardware::Eeprom, Error};
use encdec::{Decode, DecodeOwned, Encode};
use heapless::String;
use minimq::embedded_nal::Ipv4Addr;
use smoltcp_nal::smoltcp;

use crate::hardware::chassis_fans::DEFAULT_FAN_SPEED;

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

fn identifier_is_valid(id: &str) -> bool {
    id.len() <= 23 && id.chars().all(|x| x.is_alphanumeric() || x == '-')
}

/// Represents booster mainboard-specific configuration values.
#[derive(Debug, Encode, DecodeOwned)]
struct BoosterMainBoardData {
    version: SemVersion,
    ip_address: [u8; 4],
    broker_address: [u8; 4],
    gateway: [u8; 4],
    netmask: [u8; 4],
    identifier: [u8; 23],
    id_size: u32,
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
            ip_address: [0, 0, 0, 0],
            broker_address: [10, 0, 0, 2],
            gateway: [0, 0, 0, 0],
            netmask: [0, 0, 0, 0],
            identifier: id,
            id_size: name.len() as u32,
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
        let (mut config, _) = BoosterMainBoardData::decode_owned(data).unwrap();
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

    /// Get the MQTT identifier of Booster.
    pub fn id(&self) -> &[u8] {
        &self.identifier[..self.id_size as usize]
    }

    /// Serialize the booster config into a sinara configuration for storage into EEPROM.
    ///
    /// # Args
    /// * `config` - The sinara configuration to serialize the booster configuration into.
    pub fn serialize_into(&self, config: &mut SinaraConfiguration) {
        let mut buffer: [u8; 64] = [0; 64];
        let len = self.encode(&mut buffer).unwrap();
        config.board_data[..len].copy_from_slice(&buffer[..len]);
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
        //self.eeprom.write(0, &serialized).unwrap();
    }

    /// Get the Booster unique identifier.
    pub fn id(&self) -> &str {
        core::str::from_utf8(self.board_data.id()).unwrap()
    }

    /// Get the Booster MAC address.
    pub fn mac(&self) -> MacAddress {
        MacAddress(self.eui48)
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
        let addr = &self.board_data.broker_address;
        Ipv4Addr::new(addr[0], addr[1], addr[2], addr[3])
    }

    /// Check if current settings differ from active (executing) settings.
    pub fn are_dirty(&self) -> bool {
        self.dirty
    }

    /// Update the broker IP address.
    pub fn set_broker(&mut self, addr: Ipv4Addr) {
        self.dirty = true;
        self.board_data.broker_address = addr.octets();
        self.save();
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
        self.board_data.identifier[..len].copy_from_slice(id.as_bytes());
        self.board_data.id_size = len as u32;

        self.dirty = true;
        self.save();

        Ok(())
    }

    /// Get the IP address of the device.
    ///
    /// # Note
    /// The IP address will be unspecified if DHCP is to be used.
    pub fn ip_address(&self) -> smoltcp::wire::IpCidr {
        let ip_addr = smoltcp::wire::Ipv4Address::from_bytes(&self.board_data.ip_address);

        let prefix = if !ip_addr.is_unspecified() {
            let netmask = smoltcp::wire::IpAddress::Ipv4(smoltcp::wire::Ipv4Address::from_bytes(
                &self.board_data.netmask,
            ));

            netmask.prefix_len().unwrap_or_else(|| {
                log::error!("Invalid netmask found. Assuming no mask.");
                0
            })
        } else {
            0
        };

        smoltcp::wire::IpCidr::new(smoltcp::wire::IpAddress::Ipv4(ip_addr), prefix)
    }

    /// Set the static IP address of the device.
    ///
    /// # Args
    /// * `addr` - The address to set
    pub fn set_ip_address(&mut self, addr: Ipv4Addr) {
        self.board_data.ip_address = addr.octets();
        self.dirty = true;
        self.save();
    }

    /// Get the netmask of the device.
    pub fn netmask(&self) -> smoltcp::wire::Ipv4Address {
        smoltcp::wire::Ipv4Address::from_bytes(&self.board_data.netmask)
    }

    /// Set the netmask of the static IP address.
    pub fn set_netmask(&mut self, mask: Ipv4Addr) {
        let netmask = smoltcp::wire::IpAddress::Ipv4(smoltcp::wire::Ipv4Address::from_bytes(
            &mask.octets()[..],
        ));

        if netmask.prefix_len().is_none() {
            log::error!("Netmask is invalid. Ignoring");
            return;
        }

        self.board_data.netmask = mask.octets();
        self.dirty = true;
        self.save();
    }

    /// Get the default gateway IP address for the interface.
    pub fn gateway(&self) -> smoltcp::wire::Ipv4Address {
        smoltcp::wire::Ipv4Address::from_bytes(&self.board_data.gateway)
    }

    /// Set the gateway of the device.
    ///
    /// # Note
    pub fn set_gateway(&mut self, gateway: Ipv4Addr) {
        self.board_data.gateway = gateway.octets();
        self.dirty = true;
        self.save();
    }
}
