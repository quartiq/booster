//! Driver for the TCA9548 I2C bus multiplexer
//!
//! This minimal driver allows for intiailizing the I2C bus multiplexer as well as configuring a
//! single bus connection for the output mux.
use embedded_hal::{
    blocking::{delay::DelayUs, i2c::Write},
    digital::v2::OutputPin,
};

/// The driver for the TCA9548 I2C bus multiplexer.
pub struct Tca9548<I2C>
where
    I2C: Write,
{
    i2c: I2C,
    address: u8,
}

/// Represents a bus connection on the I2C fanout.
pub enum Bus {
    Zero = 0x01,
    One = 0x02,
    Two = 0x04,
    Three = 0x08,
    Four = 0x10,
    Five = 0x20,
    Six = 0x40,
    Seven = 0x80,
}

impl<I2C> Tca9548<I2C>
where
    I2C: Write,
{
    /// Construct a new I2C bus mux.
    ///
    /// # Args
    /// * `i2c` - The I2C bus to use for communication with the MUX.
    /// * `address` - The 7-bit I2C address of the device.
    /// * `reset` - A pin connected to the RST input of the device.
    /// * `delay` - A means of delaying for a specific amount of time.
    pub fn new<RST, DELAY>(
        i2c: I2C,
        address: u8,
        reset: &mut RST,
        delay: &mut DELAY,
    ) -> Result<Self, I2C::Error>
    where
        RST: OutputPin,
        DELAY: DelayUs<u8>,
        RST::Error: core::fmt::Debug,
    {
        let mut device = Tca9548 { i2c, address };

        // Reset the device.
        reset.set_low().unwrap();
        delay.delay_us(10_u8);
        reset.set_high().unwrap();

        // Select none of the I2C buses.
        device.select_bus(None)?;

        Ok(device)
    }

    /// Construct a new I2C bus mux with a default address.
    ///
    /// # Args
    /// * `i2c` - The I2C bus to use for communication with the MUX.
    /// * `reset` - A pin connected to the RST input of the device.
    /// * `delay` - A means of delaying for a specific amount of time.
    pub fn default<RST, DELAY>(
        i2c: I2C,
        reset: &mut RST,
        delay: &mut DELAY,
    ) -> Result<Self, I2C::Error>
    where
        RST: OutputPin,
        DELAY: DelayUs<u8>,
        RST::Error: core::fmt::Debug,
    {
        Tca9548::new(i2c, 0x70, reset, delay)
    }

    /// Select an I2C bus to connect.
    ///
    /// # Args
    /// * `bus` - An optional bus to connect. If None, all buses will be disconnected.
    pub fn select_bus(&mut self, bus: Option<Bus>) -> Result<(), I2C::Error> {
        if let Some(bus) = bus {
            self.i2c.write(self.address, &[bus as u8])
        } else {
            self.i2c.write(self.address, &[0u8])
        }
    }
}
