//! Booster NGFW NVM settings

use core::fmt::Write;
use core::str::FromStr;
use heapless::String;
use miniconf::Tree;

pub mod eeprom;
pub mod flash;
pub mod runtime_settings;

use eeprom::main_board::IpAddr;
use runtime_settings::RuntimeSettings;
use smoltcp_nal::smoltcp;

#[derive(Clone, Debug, Tree)]
pub struct Settings {
    #[tree(depth = 4)]
    pub booster: RuntimeSettings,

    #[tree(skip)]
    pub mac: smoltcp_nal::smoltcp::wire::EthernetAddress,

    #[tree(validate=Self::validate_ip)]
    pub ip: String<18>,
    pub broker: String<255>,
    pub gateway: IpAddr,
    pub id: String<23>,
}

impl Settings {
    /// Get the IP address of the device.
    ///
    /// # Note
    /// The IP address will be unspecified if DHCP is to be used.
    pub fn ip_cidr(&self) -> smoltcp::wire::IpCidr {
        self.ip.parse().unwrap()
    }

    fn validate_ip(&mut self, new: String<18>) -> Result<String<18>, &'static str> {
        match smoltcp::wire::IpCidr::from_str(&new) {
            Ok(smoltcp::wire::IpCidr::Ipv4(_)) => Ok(new),
            Ok(_) => Err("IPv6 addresses are not supported"),
            Err(_) => Err("Please provide a valid IPv4 CIDR (i.e. 192.168.1.1/12"),
        }
    }
}

impl serial_settings::Settings<5> for Settings {
    fn reset(&mut self) {
        self.id.clear();
        write!(
            &mut self.id,
            "{:02x}-{:02x}-{:02x}-{:02x}-{:02x}-{:02x}",
            self.mac.0[0],
            self.mac.0[1],
            self.mac.0[2],
            self.mac.0[3],
            self.mac.0[4],
            self.mac.0[5]
        )
        .unwrap();

        self.booster.reset();
        self.ip = "0.0.0.0/0".parse().unwrap();
        self.broker = "mqtt".parse().unwrap();
        self.gateway = IpAddr::new(&[0, 0, 0, 0]);
    }
}
