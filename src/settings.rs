use super::Eeprom;
use w5500::{Ipv4Addr, MacAddress};

pub struct Settings {
    pub mac_address: w5500::MacAddress,
    pub ip_address: Ipv4Addr,
    pub broker_address: Ipv4Addr,
    pub gateway: Ipv4Addr,
    pub netmask: Ipv4Addr,
    pub identifier: &'static str,
    _eeprom: Eeprom,
    dirty: bool,
}

impl Settings {
    pub fn load(mut eeprom: Eeprom) -> Settings {
        let mut mac: [u8; 6] = [0; 6];
        eeprom.read_eui48(&mut mac).unwrap();
        Self {
            mac_address: MacAddress::from_bytes(mac),
            ip_address: Ipv4Addr::new(10, 0, 0, 1),
            gateway: Ipv4Addr::new(10, 0, 0, 0),
            netmask: Ipv4Addr::new(255, 255, 255, 0),
            broker_address: Ipv4Addr::new(10, 0, 0, 2),
            identifier: "booster",
            dirty: false,
            _eeprom: eeprom,
        }
    }

    pub fn are_dirty(&self) -> bool {
        self.dirty
    }

    pub fn set_broker(&mut self, addr: Ipv4Addr) {
        self.dirty = true;
        self.broker_address = addr;
    }

    pub fn set_ip_address(&mut self, addr: Ipv4Addr) {
        self.dirty = true;
        self.ip_address = addr;
    }
}
