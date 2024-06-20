//! Booster NGFW NVM settings

use core::fmt::Write;
use heapless::String;
use miniconf::Tree;

pub mod eeprom;
pub mod flash;
pub mod runtime_settings;

use eeprom::main_board::{Cidr, IpAddr};
use runtime_settings::RuntimeSettings;

#[derive(Clone, Debug, Tree)]
pub struct Settings {
    #[tree(depth = 4)]
    pub booster: RuntimeSettings,

    #[tree(skip)]
    pub mac: smoltcp_nal::smoltcp::wire::EthernetAddress,

    pub ip: Cidr,
    pub broker: String<255>,
    pub gateway: IpAddr,
    pub id: String<23>,
}

impl serial_settings::Settings<5> for Settings {
    fn reset(&mut self) {
        self.id.clear();
        write!(&mut self.id, "{}", self.mac).unwrap();

        self.booster.reset();
        self.ip = "0.0.0.0/0".parse().unwrap();
        self.broker = "mqtt".parse().unwrap();
        self.gateway = "0.0.0.0".parse().unwrap();
    }
}
