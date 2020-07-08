//! Driver for the microchip 24AA02E48 Eeprom with EUI-48.
//!
//! # Copyright
//! Copyright (C) 2020 QUARTIQ GmbH - All Rights Reserved
//! Unauthorized usage, editing, or copying is strictly prohibited.
//! Proprietary and confidential.
//!
//! # Description
//! The 24AA02E48 provides a globally unique EUI-48 (or EUI-64) address as well as 256 bytes of
//! EEPROM.
#![no_std]
#![deny(warnings)]

use embedded_hal::blocking::i2c::{Write, WriteRead};

const DEVICE_ADDRESS: u8 = 0b101_0000;

struct Microchip24AA02E48<I2C>
where
    I2C: WriteRead + Write,
{
    i2c: I2C,
}

pub enum Error {
    Interface,
    PageFault,
    Bounds,
}

impl<I2C> Micropchip24AA02E48<I2C>
where
    I2C: WriteRead + Write,
{
    /// Construct a driver for the EUI-48.
    ///
    /// # Args
    /// * `i2c` - The I2C bus to communicate with the DAC.
    pub fn new(i2c: I2C) -> Self {
        Microchip24AA02E48 { i2c }
    }

    fn write_page(&mut self, data: &[u8], address: u8) {
        let end_address: usize = address as usize + data.len() - 1;

        // The EEPROM only supports writing to the first page of 128 bytes.
        if end_address > 128 {
            return Err(Error::Bounds);
        }

        // Verify the write does not cross a page boundary.
        let start_page = address >> 3;
        let end_page = (address + data.len() - 1) >> 3;

        if start_page != end_page {
            return Err(Error::PageFault);
        }

        let mut write_data: [u8; 9] = [0; 9];
        write_data[0] = address;
        write_data[1..][..data.len()].copy_from_slice(data);

        self.i2c.write(DEVICE_ADDRESS, &write_data).map_err(|_| Err::Interface)?;

        Ok(())
    }

    fn read_page(&mut self, data: &mut [u8], address: u8) -> Result<(), Error> {
        let end_address: usize = address as usize + data.len() - 1;

        // The EEPROM only contains 256 bytes.
        if end_address > 256 {
            return Err(Error::Bounds);
        }

        // Verify the read does not cross a page boundary.
        let start_page = address >> 3;
        let end_page = (address + data.len() - 1) >> 3;

        if start_page != end_page {
            return Err(Error::PageFault);
        }

        self.i2c.write_read(DEVICE_ADDRESS, &[address], &mut data).map_err(|_| Err::Interface)?;

        Ok(())
    }

    pub fn write(&mut self, data: & [u8], address: u8) -> Result<(), Error> {
    }

    pub fn read(&mut self, data: &mut [u8], address: u8) -> Result<(), Error> {
    }

    pub fn read_eui48(&mut self, data: &mut [u8]) -> Result<(), Error> {
    }

    pub fn read_eui64(&mut self, data: &mut [u8]) -> Result<(), Error> {
    }
}
