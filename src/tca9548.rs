
use embedded_hal::blocking::i2c::Write;

pub struct Tca9548<I2C>
where
    I2C: Write,
{
    i2c: I2C,
    address: u8,
}

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
    pub fn new(i2c: I2C, address: u8) -> Result<Self, I2C::Error> {
        let mut device = Tca9548 {
            i2c,
            address,
        };

        device.select_bus(None)?;

        Ok(device)
    }

    pub fn default(i2c: I2C) -> Result<Self, I2C::Error> {
        Tca9548::new(i2c, 0x70)
    }

    pub fn select_bus(&mut self, bus: Option<Bus>) -> Result<(), I2C::Error> {
        if let Some(bus) = bus {
            self.i2c.write(self.address, &[bus as u8])
        } else {
            self.i2c.write(self.address, &[0u8])
        }
    }
}
