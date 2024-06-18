//! Booster NGFW NVM settings

use crate::hardware::chassis_fans::DEFAULT_FAN_SPEED;
use crate::net::mqtt_control::DEFAULT_TELEMETRY_PERIOD_SECS;
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

    pub ip: IpAddr,
    pub broker: heapless::String<255>,
    pub gateway: IpAddr,
    pub netmask: IpAddr,
    pub id: heapless::String<23>,
}

impl Settings {
    /// Get the IP address of the device.
    ///
    /// # Note
    /// The IP address will be unspecified if DHCP is to be used.
    pub fn ip_cidr(&self) -> smoltcp::wire::IpCidr {
        let ip_addr = self.ip.0;

        let prefix = if !ip_addr.is_unspecified() {
            let netmask = smoltcp::wire::IpAddress::Ipv4(self.netmask.0);

            netmask.prefix_len().unwrap_or_else(|| {
                log::error!("Invalid netmask found. Assuming no mask.");
                0
            })
        } else {
            0
        };

        smoltcp::wire::IpCidr::new(smoltcp::wire::IpAddress::Ipv4(ip_addr), prefix)
    }
}

impl serial_settings::Settings<5> for Settings {
    fn reset(&mut self) {
        let mut name: String<23> = String::new();
        write!(
            &mut name,
            "{:02x}-{:02x}-{:02x}-{:02x}-{:02x}-{:02x}",
            self.mac.0[0],
            self.mac.0[1],
            self.mac.0[2],
            self.mac.0[3],
            self.mac.0[4],
            self.mac.0[5]
        )
        .unwrap();

        *self = Self {
            mac: self.mac,
            ip: IpAddr::new(&[0, 0, 0, 0]),
            broker: heapless::String::from_str("10.0.0.2").unwrap(),
            gateway: IpAddr::new(&[0, 0, 0, 0]),
            netmask: IpAddr::new(&[0, 0, 0, 0]),
            id: name,
            booster: RuntimeSettings {
                fan_speed: DEFAULT_FAN_SPEED,
                telemetry_period: DEFAULT_TELEMETRY_PERIOD_SECS,
                channel: self.booster.channel,
            },
        };
        // TODO: Reset each channel config.
    }
}
