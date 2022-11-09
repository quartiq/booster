//! Driver for the AD5627R 2-output programmable reference generator (2-channel DAC).
//!
//! # Description
//! This driver allows for configuring either or both DAC output voltages. It assumes that the DAC
//! is using an internal 1.25V reference (AD5627R variant).
#![no_std]
#![deny(warnings)]

use embedded_hal::blocking::i2c::Write;

/// The maximum voltage that the DAC can output.
pub const MAX_VOLTAGE: f32 = 2.5;

/// The driver representing the programmable reference generator.
pub struct Ad5627<I2C>
where
    I2C: Write,
{
    i2c: I2C,
    address: u8,
}

#[doc(hidden)]
#[allow(dead_code)]
/// Represents various commands that can be sent to the DAC during a write.
enum Command {
    WriteInput = 0b000,
    WriteDac = 0b001,
    WriteInputUpdate = 0b010,
    WriteDacUpdate = 0b011,
    PowerUpdate = 0b100,
    Reset = 0b101,
    LdacSetup = 0b110,
    InternalRefSetup = 0b111,
}

/// Represents which DAC output to update.
pub enum Dac {
    A = 0b000,
    B = 0b001,
    Both = 0b111,
}

/// Represents possible errors from the DAC driver.
pub enum Error<E> {
    Range,
    I2c(E),
}

impl<E> From<E> for Error<E> {
    fn from(e: E) -> Self {
        Error::I2c(e)
    }
}

impl<I2C> Ad5627<I2C>
where
    I2C: Write,
{
    /// Construct a driver for the DAC.
    ///
    /// # Args
    /// * `i2c` - The I2C bus to communicate with the DAC.
    /// * `address` - The 7-bit I2C address of the device.
    pub fn new(i2c: I2C, address: u8) -> Result<Self, I2C::Error> {
        let mut device = Ad5627 { i2c, address };

        // Reset the DAC outputs.
        device.write(Command::Reset, Dac::Both, [0, 0])?;

        // Power up the internal reference.
        device.write(Command::InternalRefSetup, Dac::Both, [0, 1])?;

        Ok(device)
    }

    fn write(&mut self, command: Command, dac: Dac, payload: [u8; 2]) -> Result<(), I2C::Error> {
        // Construct the command byte.
        let write: [u8; 3] = [((command as u8) << 3) | dac as u8, payload[0], payload[1]];

        self.i2c.write(self.address, &write)
    }

    /// Construct a driver for the DAC.
    ///
    /// # Note
    /// This constructs a driver with a default address when the address pin is not connected.
    ///
    /// # Args
    /// * `i2c` - The I2C bus to communicate with the DAC.
    pub fn default(i2c: I2C) -> Result<Self, I2C::Error> {
        Ad5627::new(i2c, 0b0001110)
    }

    /// Set the output voltage for a specific DAC channel.
    ///
    /// # Args
    /// * `voltage` - The desired output voltage to configure the DAC to.
    /// * `dac` - Specifies which DAC (or both) should be configured.
    ///
    /// # Returns
    /// The actual voltage programmed into the DAC after digitization.
    pub fn set_voltage(&mut self, voltage: f32, dac: Dac) -> Result<f32, Error<I2C::Error>> {
        // Assuming a 1.25V internal reference with a 2x output stage gain, our full scale range is
        // 2.5V.
        if !(0.0..=crate::MAX_VOLTAGE).contains(&voltage) {
            return Err(Error::Range);
        }

        // The DAC has a 12-bit DAC output. Full scale is 0xFFF.
        let code = ((voltage / 2.5) * (0x1000 as f32)) as u16;

        // Check that the DAC code has not overflown.
        if code > 0xFFF {
            return Err(Error::Range);
        }

        // The 12-bit code must be stored MSB-aligned.
        let code = code << 4;

        // Write the dac level to the output.
        self.write(Command::WriteInput, dac, code.to_be_bytes())?;

        let programmed_voltage = ((code >> 4) as f32) / (0x1000 as f32) * 2.5;
        Ok(programmed_voltage)
    }
}
