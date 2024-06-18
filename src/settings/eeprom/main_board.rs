//! Booster NGFW NVM settings
//!
//! # Design
//! Booster firmware maintains mainboard settings in an EEPROM on the main board until v0.5.0. After
//! v0.5.0, main board settings are loaded from and stored to internal flash storage.
//!
//! In order to maintain backwards compatibility with existing Booster devices that are upgraded
//! from v0.5.0 firmware and earlier, the settings loading process occurs as follows:
//! 1. Settings are loaded from Booster mainboard EEPROM
//! 2. Settings are then loaded from flash (if possible)
//!
//! Any further saves to settings are persisted to device flash, which will result in overriding
//! the older EEPROM settings. This essentially freezes the EEPROM-based settings in time from the
//! switch over.
//!
//! Settings are stored in flash because of the restrictive size of EEPROM on the device making it
//! impossible to save domain names for a named broker into EEPROM, as the available board data storage is only 64
//! bytes, but a domain name can be up to 255 characters.

use crate::{hardware::Eeprom, Error};
use core::str::FromStr;
use encdec::{Decode, DecodeOwned, Encode};
use heapless::String;
use smoltcp_nal::smoltcp;

use crate::hardware::chassis_fans::DEFAULT_FAN_SPEED;

use super::{
    sinara::{BoardId as SinaraBoardId, SinaraConfiguration},
    SemVersion,
};
use serde::{Deserialize, Serialize};

use core::fmt::Write;
use miniconf::Tree;
use serde_with::DeserializeFromStr;

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

#[derive(DeserializeFromStr, Copy, Clone, Debug)]
pub struct IpAddr(pub smoltcp_nal::smoltcp::wire::Ipv4Address);

impl Serialize for IpAddr {
    fn serialize<S: serde::Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        let mut display: String<16> = String::new();
        write!(&mut display, "{}", self).unwrap();
        serializer.serialize_str(&display)
    }
}

impl IpAddr {
    pub fn new(bytes: &[u8]) -> Self {
        Self(smoltcp::wire::Ipv4Address::from_bytes(bytes))
    }
}

impl core::str::FromStr for IpAddr {
    type Err = &'static str;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        let addr = smoltcp::wire::Ipv4Address::from_str(s).map_err(|_| "Invalid IP format")?;
        Ok(Self(addr))
    }
}

impl core::fmt::Display for IpAddr {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        self.0.fmt(f)
    }
}

impl encdec::Encode for IpAddr {
    type Error = encdec::Error;

    fn encode_len(&self) -> Result<usize, Self::Error> {
        Ok(self.0 .0.len())
    }

    fn encode(&self, buff: &mut [u8]) -> Result<usize, Self::Error> {
        self.0 .0.encode(buff)
    }
}

impl encdec::DecodeOwned for IpAddr {
    type Output = IpAddr;
    type Error = encdec::Error;

    fn decode_owned(buff: &[u8]) -> Result<(Self::Output, usize), Self::Error> {
        let (data, size) = <[u8; 4]>::decode_owned(buff)?;
        Ok((Self::new(&data[..]), size))
    }
}

#[derive(Serialize, Deserialize, Clone, Debug)]
#[serde(transparent)]
pub struct MqttIdentifier(pub String<23>);

impl encdec::Encode for MqttIdentifier {
    type Error = encdec::Error;

    fn encode_len(&self) -> Result<usize, Self::Error> {
        Ok(self.0.capacity() + core::mem::size_of::<u32>())
    }

    fn encode(&self, buff: &mut [u8]) -> Result<usize, Self::Error> {
        if buff.len() < 27 {
            return Err(encdec::Error::Length);
        }
        let id_bytes = self.0.as_bytes();
        buff[..id_bytes.len()].copy_from_slice(id_bytes);
        let len = id_bytes.len() as u32;
        len.encode(&mut buff[23..])?;
        Ok(27)
    }
}

impl encdec::DecodeOwned for MqttIdentifier {
    type Output = MqttIdentifier;
    type Error = encdec::Error;

    fn decode_owned(buff: &[u8]) -> Result<(Self::Output, usize), Self::Error> {
        if buff.len() < 27 {
            return Err(encdec::Error::Length);
        }

        let len = u32::decode_owned(&buff[23..])?.0 as usize;
        let string = core::str::from_utf8(&buff[..len]).map_err(|_| encdec::Error::Utf8)?;

        Ok((MqttIdentifier(String::from(string)), 27))
    }
}

#[derive(Debug, Clone, Encode, DecodeOwned)]
pub struct SerializedMainBoardData {
    version: SemVersion,
    pub ip: IpAddr,
    pub broker: IpAddr,
    pub gateway: IpAddr,
    pub netmask: IpAddr,
    pub id: MqttIdentifier,
    pub fan_speed: f32,
}

impl From<BoosterMainBoardData> for SerializedMainBoardData {
    fn from(d: BoosterMainBoardData) -> Self {
        Self {
            version: d.version,
            ip: d.ip,
            broker: d
                .broker
                .parse()
                .unwrap_or_else(|_| IpAddr::new(&[10, 0, 0, 2])),
            gateway: d.gateway,
            netmask: d.netmask,
            id: MqttIdentifier(d.id),
            fan_speed: d.fan_speed,
        }
    }
}

impl SerializedMainBoardData {
    fn with_mac(self, eui48: &[u8; 6]) -> BoosterMainBoardData {
        let mut broker = String::new();
        write!(&mut broker, "{}", self.broker.0).unwrap();
        BoosterMainBoardData {
            mac: smoltcp_nal::smoltcp::wire::EthernetAddress(*eui48),
            version: self.version,
            ip: self.ip,
            broker,
            gateway: self.gateway,
            netmask: self.netmask,
            id: self.id.0,
            fan_speed: self.fan_speed,
        }
    }
}

/// Represents booster mainboard-specific configuration values.
#[derive(Debug, Clone, Tree, Serialize, Deserialize)]
pub struct BoosterMainBoardData {
    #[tree(skip)]
    version: SemVersion,

    #[tree(skip)]
    #[serde(skip)]
    pub mac: smoltcp_nal::smoltcp::wire::EthernetAddress,

    pub ip: IpAddr,
    pub broker: heapless::String<255>,
    pub gateway: IpAddr,
    pub netmask: IpAddr,
    pub id: heapless::String<23>,
    pub fan_speed: f32,
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
            mac: smoltcp_nal::smoltcp::wire::EthernetAddress(*eui48),
            version: EXPECTED_VERSION,
            ip: IpAddr::new(&[0, 0, 0, 0]),
            broker: String::from_str("10.0.0.2").unwrap(),
            gateway: IpAddr::new(&[0, 0, 0, 0]),
            netmask: IpAddr::new(&[0, 0, 0, 0]),
            id: name,
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
    pub fn deserialize(eui48: &[u8; 6], data: &[u8; 64]) -> Result<(Self, bool), Error> {
        let (mut config, _) = SerializedMainBoardData::decode_owned(data).unwrap();
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
        if !identifier_is_valid(&config.id.0) {
            return Err(Error::Invalid);
        }

        log::info!("Loaded settings from EEPROM");
        Ok((config.with_mac(eui48), modified))
    }

    /// Serialize the booster config into a sinara configuration for storage into EEPROM.
    ///
    /// # Args
    /// * `config` - The sinara configuration to serialize the booster configuration into.
    pub fn serialize_into(&self, config: &mut SinaraConfiguration) {
        let mut buffer: [u8; 64] = [0; 64];
        let serialized: SerializedMainBoardData = self.clone().into();
        let len = serialized.encode(&mut buffer).unwrap();
        config.board_data[..len].copy_from_slice(&buffer[..len]);
    }
}

/// Booster device-wide configurable settings.
pub struct BoosterSettings {
    pub properties: BoosterMainBoardData,
    eeprom: Eeprom,
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
            .and_then(|config| BoosterMainBoardData::deserialize(&mac, &config.board_data))
            .unwrap_or((BoosterMainBoardData::default(&mac), true));

        let mut settings = Self {
            properties: board_data,
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

        let board_data: BoosterMainBoardData = self.properties.clone();
        board_data.serialize_into(&mut config);
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
}
