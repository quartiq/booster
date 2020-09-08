//! Booster NGFW Application
//!
//! # Copyright
//! Copyright (C) 2020 QUARTIQ GmbH - All Rights Reserved
//! Unauthorized usage, editing, or copying is strictly prohibited.
//! Proprietary and confidential.
use super::Eeprom;
use heapless::{consts, String};
use w5500::{Ipv4Addr, MacAddress};

/// Booster device-wide configurable settings.
pub struct BoosterSettings {
    mac_address: w5500::MacAddress,
    ip_address: Ipv4Addr,
    broker_address: Ipv4Addr,
    gateway_address: Ipv4Addr,
    netmask: Ipv4Addr,
    identifier: String<consts::U32>,
    _eeprom: Eeprom,
    dirty: bool,
}

impl BoosterSettings {
    /// Load device settings from EEPROM.
    pub fn load(mut eeprom: Eeprom) -> BoosterSettings {
        let mut mac: [u8; 6] = [0; 6];
        eeprom.read_eui48(&mut mac).unwrap();
        let mut string = String::<consts::U32>::new();
        string.push_str("booster").unwrap();
        Self {
            mac_address: MacAddress::from_bytes(mac),
            ip_address: Ipv4Addr::new(10, 0, 0, 1),
            gateway_address: Ipv4Addr::new(10, 0, 0, 0),
            netmask: Ipv4Addr::new(255, 255, 255, 0),
            broker_address: Ipv4Addr::new(10, 0, 0, 2),
            identifier: string,
            dirty: false,
            _eeprom: eeprom,
        }
    }

    /// Get the Booster unique identifier.
    pub fn id(&self) -> String<consts::U32> {
        self.identifier.clone()
    }

    /// Get the Booster MAC address.
    pub fn mac(&self) -> w5500::MacAddress {
        self.mac_address
    }

    /// Get the Booster IP address.
    pub fn ip(&self) -> Ipv4Addr {
        self.ip_address
    }

    /// Get the MQTT broker IP address.
    pub fn broker(&self) -> Ipv4Addr {
        self.broker_address
    }

    /// Get the network gateway.
    pub fn gateway(&self) -> Ipv4Addr {
        self.gateway_address
    }

    /// Get the network subnet.
    pub fn subnet(&self) -> Ipv4Addr {
        self.netmask
    }

    /// Check if current settings differ from active (executing) settings.
    pub fn are_dirty(&self) -> bool {
        self.dirty
    }

    /// Update the broker IP address.
    pub fn set_broker(&mut self, addr: Ipv4Addr) {
        self.dirty = true;
        self.broker_address = addr;
    }

    /// Update the Booster IP address.
    pub fn set_ip_address(&mut self, addr: Ipv4Addr) {
        self.dirty = true;
        self.ip_address = addr;
    }

    /// Update the booster gateway.
    pub fn set_gateway(&mut self, addr: Ipv4Addr) {
        self.dirty = true;
        self.gateway_address = addr;
    }

    /// Update the booster net mask.
    pub fn set_netmask(&mut self, addr: Ipv4Addr) {
        self.dirty = true;
        self.netmask = addr;
    }

    /// Update the booster MQTT client identifier.
    pub fn set_id<'a>(&mut self, id: &'a str) -> bool {
        // TODO: Verify the ID is valid.
        if id.as_bytes().len() > 23 {
            return false;
        }

        if id.chars().all(|x| x.is_alphanumeric()) == false {
            return false;
        }

        self.dirty = true;
        self.identifier = String::new();
        self.identifier.push_str(id).unwrap();

        true
    }
}
