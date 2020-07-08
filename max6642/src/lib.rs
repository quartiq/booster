//! Implements a driver for the MAX6642 temperature sensor.
//!
//! # Copyright
//! Copyright (C) 2020 QUARTIQ GmbH - All Rights Reserved
//! Unauthorized usage, editing, or copying is strictly prohibited.
//! Proprietary and confidential.
#![no_std]
#![deny(warnings)]

use bit_field::BitField;
use embedded_hal::blocking::i2c::{Write, WriteRead};

#[allow(dead_code)]
#[doc(hidden)]
enum Command {
    ReadLocalTemperature = 0x00,
    ReadRemoteTemperature = 0x01,
    ReadStatusByte = 0x02,
    ReadConfigurationByte = 0x03,
    ReadLocalHighLimit = 0x05,
    ReadRemoteHighLimit = 0x07,
    WriteConfigurationByte = 0x09,
    WriteLocalHighLimit = 0x0b,
    WriteRemoteHighLimit = 0x0d,
    SingleShot = 0x0f,
    ReadRemoteExtendedTemperature = 0x10,
    ReadInternalExtendedTemperature = 0x11,
    ReadManufacturerId = 0xfe,
}

impl Command {
    /// Check if a command should be followed by writable data.
    fn is_writable(&self) -> bool {
        match self {
            Command::WriteConfigurationByte
            | Command::WriteLocalHighLimit
            | Command::WriteRemoteHighLimit
            | Command::SingleShot => true,
            _ => false,
        }
    }
}

/// Represents possible errors from the temperature sensor.
pub enum Error {
    Interface,
    DiodeFault,
    InvalidCommand,
}

/// The temperature sensor driver.
pub struct Max6642<I2C> {
    i2c: I2C,
    address: u8,
}

impl<I2C> Max6642<I2C>
where
    I2C: Write + WriteRead,
{
    /// Construct a new driver for the MAX6642-ATT94 variant.
    ///
    /// # Args
    /// * `i2c` - The I2C driver to use to communicate with the device.
    pub fn att94(i2c: I2C) -> Self {
        Max6642::new(i2c, 0x4A)
    }

    /// Construct a new driver for the MAX6642 temperature sensor.
    ///
    /// # Args
    /// * `i2c` - The I2C driver to use to communicate with the device.
    /// * `address` - The I2C address of the device.
    pub fn new(i2c: I2C, address: u8) -> Self {
        Max6642 { i2c, address }
    }

    fn read(&mut self, command: Command) -> Result<u8, Error> {
        let mut result: [u8; 1] = [0; 1];
        self.i2c
            .write_read(self.address, &[command as u8], &mut result)
            .map_err(|_| Error::Interface)?;

        Ok(result[0])
    }

    #[allow(dead_code)]
    fn write(&mut self, command: Command, value: u8) -> Result<(), Error> {
        if command.is_writable() == false {
            return Err(Error::InvalidCommand);
        }

        self.i2c
            .write(self.address, &[command as u8, value])
            .map_err(|_| Error::Interface)?;

        Ok(())
    }

    /// Get the temperature of the remote diode.
    ///
    /// # Returns
    /// The temperature of the remote diode in degrees celsius.
    pub fn get_remote_temperature(&mut self) -> Result<f32, Error> {
        let temp_c = self.read(Command::ReadRemoteTemperature)?;
        if temp_c > 130 {
            return Err(Error::DiodeFault);
        }

        // 0.25C temperature is stored in the top 2 bits of the extended data register.
        let temp_c_4ths = self
            .read(Command::ReadRemoteExtendedTemperature)?
            .get_bits(6..8);

        let temp_c = (temp_c as f32) + (temp_c_4ths as f32) * 0.25;

        Ok(temp_c)
    }
}
