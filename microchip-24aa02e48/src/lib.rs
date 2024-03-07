//! # Description
//! Driver for the microchip 24AA02E48 Eeprom with EUI-48.
//! The 24AA02E48 provides a globally unique EUI-48 (or EUI-64) address as well as 256 bytes of
//! EEPROM.
#![no_std]
#![deny(warnings)]

use embedded_hal::i2c::{I2c, ErrorType};

/// The default I2C device address of the EUI-48.
const DEVICE_ADDRESS: u8 = 0b101_0000;

/// The number of bytes within a single EEPROM page.
const PAGE_SIZE: usize = 8;

/// The number of pages contained within EEPROM.
const TOTAL_PAGES: usize = 32;

/// The number of writable pages at the beginning of EEPROM.
const WRITABLE_PAGES: usize = 16;

/// A driver for the 24AA02E48 EUI-48 2Kb EEPROM.
pub struct Microchip24AA02E48<I2C>
where
    I2C: I2c,
{
    i2c: I2C,
}

/// Represents various errors that may be encountered by the EEPROM driver.
#[derive(Debug)]
pub enum Error<E> {
    Interface(E),
    PageFault,
    Bounds,
}

impl<E> From<E> for Error<E> {
    fn from(err: E) -> Error<E> {
        Error::Interface(err)
    }
}

impl<I2C> Microchip24AA02E48<I2C>
where
    I2C: I2c,
    <I2C as ErrorType>::Error: Into<<I2C as ErrorType>::Error>,
{
    /// Construct a driver for the EUI-48.
    ///
    /// # Args
    /// * `i2c` - The I2C bus to communicate with the DAC.
    pub fn new(i2c: I2C) -> Result<Self, Error<<I2C as ErrorType>::Error>> {
        let mut eeprom = Microchip24AA02E48 { i2c };

        // In case we are initializing the device while a write sequence is in progress, wait for
        // the write sequence to complete so we don't unintentionally use the device while it is not
        // available.
        eeprom.wait_for_write_sequence()?;

        Ok(eeprom)
    }

    fn wait_for_write_sequence(&mut self) -> Result<(), Error<<I2C as ErrorType>::Error>> {
        loop {
            match self.i2c.write(DEVICE_ADDRESS, &[]) {
                Ok(_) => return Ok(()),
                Err(_) => {}
            }
        }
    }

    fn write_page(
        &mut self,
        address: u8,
        data: &[u8],
    ) -> Result<(), Error<<I2C as ErrorType>::Error>> {
        let end_address: usize = address as usize + data.len() - 1;

        // The EEPROM only supports writing to the first 16 pages (128 bytes).
        if end_address > PAGE_SIZE * WRITABLE_PAGES {
            return Err(Error::Bounds);
        }

        // Verify the write does not cross a page boundary.
        let start_page = (address >> 3) as usize;
        let end_page = (end_address >> 3) as usize;

        if start_page != end_page {
            return Err(Error::PageFault);
        }

        let mut write_data: [u8; PAGE_SIZE + 1] = [0; PAGE_SIZE + 1];
        write_data[0] = address;
        write_data[1..][..data.len()].copy_from_slice(data);

        self.i2c
            .write(DEVICE_ADDRESS, &write_data[..data.len() + 1])
            .map_err(|err| err.into())?;

        // Wait for the internal page write sequence to complete.
        self.wait_for_write_sequence()?;

        Ok(())
    }

    /// Write data to the EEPROM in the chip.
    ///
    /// # Note
    /// The chip only supports writing to the first 128 bytes.
    ///
    /// # Args
    /// * `address` - The address to write data to.
    /// * `data` - The data to write into EEPROM.
    pub fn write(
        &mut self,
        address: u8,
        data: &[u8],
    ) -> Result<(), Error<<I2C as ErrorType>::Error>> {
        let final_address = address as usize + data.len() - 1;
        if final_address > PAGE_SIZE * WRITABLE_PAGES {
            return Err(Error::Bounds);
        }

        // Determine the number of bytes in the first page.
        let start_page = (address >> 3) as usize;
        let next_page_address = ((start_page + 1) << 3) as usize;

        let mut bytes_in_first_page = next_page_address - address as usize;
        if bytes_in_first_page > data.len() {
            bytes_in_first_page = data.len();
        }

        // First, write the data in the initial page.
        self.write_page(address, &data[..bytes_in_first_page])?;

        // Continue writing pages until data is exhausted.
        let mut address = address + bytes_in_first_page as u8;
        for chunk in data[bytes_in_first_page..].chunks(PAGE_SIZE) {
            self.write_page(address, chunk)?;
            address += chunk.len() as u8;
        }

        Ok(())
    }

    /// Rea data from the EEPROM in the chip.
    ///
    /// # Args
    /// * `address` - The address to read data from.
    /// * `data` - The location to place read data into.
    pub fn read(
        &mut self,
        address: u8,
        data: &mut [u8],
    ) -> Result<(), Error<<I2C as ErrorType>::Error>> {
        let final_address = address as usize + data.len() - 1;
        if final_address > PAGE_SIZE * TOTAL_PAGES {
            return Err(Error::Bounds);
        }

        self.i2c.write_read(DEVICE_ADDRESS, &[address], data)?;

        Ok(())
    }

    /// Read the unique EUI-48 identifier from the chip.
    ///
    /// # Args
    /// * `data` - An array of 6 bytes to store the EUI-48 into.
    pub fn read_eui48(
        &mut self,
        data: &mut [u8; 6],
    ) -> Result<(), Error<<I2C as ErrorType>::Error>> {
        self.read(0xFA, data)?;

        Ok(())
    }

    /// Read the unique EUI-64 identifier from the chip.
    ///
    /// # Note
    /// The chip does not support a unique EUI-64 intrinisically, so an EUI-64 is created using
    /// the internal EUI-48.
    ///
    /// # Args
    /// * `data` - An array of 8 bytes to store the EUI-64 into.
    pub fn read_eui64(
        &mut self,
        data: &mut [u8; 8],
    ) -> Result<(), Error<<I2C as ErrorType>::Error>> {
        // To support a 64-bit EUI, the OUI (Organizationally unique identifier) and the 24-bit EI
        // (extension identifier) have 0xFFFE placed between them.
        self.read(0xFA, &mut data[..3])?;
        data[3] = 0xFF;
        data[4] = 0xFE;
        self.read(0xFD, &mut data[5..])?;

        Ok(())
    }
}
