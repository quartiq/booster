//! Driver for the MCP3221 external analog-to-digital converter.
#![no_std]
#![deny(warnings)]

use embedded_hal::i2c::{ErrorType, I2c};

// The default address of the ADC.
const DEVICE_ADDRESS: u8 = 0x4D;

/// A driver for the MCP3221 external analog-to-digital converter.
pub struct Mcp3221<I2C> {
    i2c: I2C,
    supply_voltage: f32,
}

impl<I2C> Mcp3221<I2C>
where
    I2C: I2c,
{
    /// Construct a MCP3221 driver.
    ///
    /// # Args
    /// * `i2c` - The I2C bus used to communicate with the device.
    /// * `vdd` - The supply voltage to the VDD pin of the chip.
    pub fn new(i2c: I2C, vdd: f32) -> Self {
        Mcp3221 {
            i2c,
            supply_voltage: vdd,
        }
    }

    /// Construct a default MCP3221 driver.
    ///
    /// # Note:
    /// A default configuration assumes 3.3V
    ///
    /// # Args
    /// * `i2c` - The I2C bus used to communicate with the device.
    pub fn default(i2c: I2C) -> Self {
        Mcp3221::new(i2c, 3.3)
    }

    /// Measure the analog voltage from the device.
    ///
    /// # Returns
    /// The analog measurement of the conversion in volts.
    pub fn get_voltage(&mut self) -> Result<f32, <I2C as ErrorType>::Error> {
        let mut conversion: [u8; 2] = [0; 2];
        self.i2c.read(DEVICE_ADDRESS, &mut conversion)?;

        let analog_code = u16::from_be_bytes(conversion) & 0xFFF;
        Ok(analog_code as f32 / 4096.0 * self.supply_voltage)
    }
}
