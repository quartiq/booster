//! Booster module-level hardware definitions
//!
//! # Copyright
///! Copyright (C) 2020-2022 QUARTIQ GmbH

use core::fmt::Write;
use enum_iterator::IntoEnumIterator;
use serde::{Deserialize, Serialize};
use stm32f4xx_hal as hal;

pub mod booster_channels;
pub mod chassis_fans;
pub mod delay;
pub mod external_mac;
pub mod metadata;
pub mod net_interface;
pub mod platform;
pub mod rf_channel;
pub mod setup;
pub mod user_interface;

pub const MONOTONIC_FREQUENCY: u32 = 1_000;
pub type Systick = systick_monotonic::Systick<MONOTONIC_FREQUENCY>;
pub type SystemTimer = mono_clock::MonoClock<u32, MONOTONIC_FREQUENCY>;

pub const CPU_FREQ: u32 = 168_000_000;

// Convenience type definition for the I2C bus used for booster RF channels.
pub type I2C = hal::i2c::I2c<
    hal::stm32::I2C1,
    (
        hal::gpio::gpiob::PB6<hal::gpio::AlternateOD<hal::gpio::AF4>>,
        hal::gpio::gpiob::PB7<hal::gpio::AlternateOD<hal::gpio::AF4>>,
    ),
>;

pub type I2C2 = hal::i2c::I2c<
    hal::stm32::I2C2,
    (
        hal::gpio::gpiob::PB10<hal::gpio::AlternateOD<hal::gpio::AF4>>,
        hal::gpio::gpiob::PB11<hal::gpio::AlternateOD<hal::gpio::AF4>>,
    ),
>;

pub type SpiCs = hal::gpio::gpioa::PA4<hal::gpio::Output<hal::gpio::PushPull>>;

pub type Spi = hal::spi::Spi<
    hal::stm32::SPI1,
    (
        hal::gpio::gpioa::PA5<hal::gpio::Alternate<hal::gpio::AF5>>,
        hal::gpio::gpioa::PA6<hal::gpio::Alternate<hal::gpio::AF5>>,
        hal::gpio::gpioa::PA7<hal::gpio::Alternate<hal::gpio::AF5>>,
    ),
>;

pub type Led1 = hal::gpio::gpioc::PC8<hal::gpio::Output<hal::gpio::PushPull>>;
pub type Led2 = hal::gpio::gpioc::PC9<hal::gpio::Output<hal::gpio::PushPull>>;
pub type Led3 = hal::gpio::gpioc::PC10<hal::gpio::Output<hal::gpio::PushPull>>;
pub type MainboardLeds = (Led1, Led2, Led3);

#[cfg(feature = "phy_w5500")]
pub type ExternalMac = w5500::raw_device::RawDevice<w5500::bus::FourWire<Spi, SpiCs>>;

#[cfg(feature = "phy_enc424j600")]
pub type ExternalMac = enc424j600::Enc424j600<Spi, SpiCs>;

pub type NetworkManager = external_mac::Manager<'static, ExternalMac>;

pub type NetworkStack =
    smoltcp_nal::NetworkStack<'static, external_mac::SmoltcpDevice<'static>, SystemTimer>;

pub type I2cBusManager = shared_bus::BusManagerAtomicCheck<I2C>;
pub type I2cProxy = shared_bus::I2cProxy<'static, shared_bus::AtomicCheckMutex<I2C>>;
pub type I2cError = hal::i2c::Error;

pub type UsbBus = hal::otg_fs::UsbBus<hal::otg_fs::USB>;
pub type Eeprom = microchip_24aa02e48::Microchip24AA02E48<I2C2>;

/// Indicates a booster RF channel.
#[derive(IntoEnumIterator, Copy, Clone, Debug, Serialize, Deserialize)]
pub enum Channel {
    Zero = 0,
    One = 1,
    Two = 2,
    Three = 3,
    Four = 4,
    Five = 5,
    Six = 6,
    Seven = 7,
}

pub enum HardwareVersion {
    Rev1_2OrEarlier,
    Rev1_3,
    Rev1_5,
    Unknown(u8),
}

impl From<u8> for HardwareVersion {
    fn from(bitfield: u8) -> Self {
        match bitfield {
            0b000 => HardwareVersion::Rev1_2OrEarlier,
            0b011 => HardwareVersion::Rev1_3,
            0b100 => HardwareVersion::Rev1_5,
            other => HardwareVersion::Unknown(other),
        }
    }
}

impl core::fmt::Display for HardwareVersion {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            HardwareVersion::Rev1_2OrEarlier => write!(f, "<= v1.2"),
            HardwareVersion::Rev1_3 => write!(f, "v1.3"),
            HardwareVersion::Rev1_5 => write!(f, "v1.5"),
            HardwareVersion::Unknown(other) => write!(f, "Unknown ({:#b})", other),
        }
    }
}

impl serde::Serialize for HardwareVersion {
    fn serialize<S: serde::Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        let mut version_string: heapless::String<32> = heapless::String::new();
        write!(&mut version_string, "{}", self).unwrap();
        serializer.serialize_str(&version_string)
    }
}
