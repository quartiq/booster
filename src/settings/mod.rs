//! Booster NGFW NVM settings

use core::fmt::Write;
use core::str::FromStr;
use heapless::String;
use miniconf::{Leaf, StrLeaf, Tree};

pub mod eeprom;
pub mod flash;
pub mod runtime_settings;

use eeprom::main_board::{Cidr, IpAddr};
use runtime_settings::RuntimeSettings;

#[derive(Clone, Debug, Tree)]
pub struct Settings {
    pub booster: RuntimeSettings,

    #[tree(skip)]
    pub mac: smoltcp_nal::smoltcp::wire::EthernetAddress,

    pub ip: Leaf<Cidr>,
    pub broker: StrLeaf<String<255>>,
    pub gateway: Leaf<IpAddr>,
    pub id: StrLeaf<String<23>>,
}

impl serial_settings::Settings for Settings {
    fn reset(&mut self) {
        self.id.clear();
        write!(&mut self.id, "{}", self.mac).unwrap();

        self.booster.reset();
        self.ip = Leaf(Cidr::from_str("0.0.0.0/0").unwrap());
        self.broker = StrLeaf(String::from_str("mqtt").unwrap());
        self.gateway = Leaf(IpAddr::from_str("0.0.0.0").unwrap());
    }
}
