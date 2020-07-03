
use embedded_hal::{
    digital::v2::OutputPin,
    blocking::{
        delay::DelayUs,
        i2c::Write,
    },
};

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
    pub fn new<RST, DELAY>(i2c: I2C, address: u8, reset: &mut RST, delay: &mut DELAY) -> Self
    where
        RST: OutputPin,
        DELAY: DelayUs<u8>,
        RST::Error: core::fmt::Debug,
    {
        reset.set_low().unwrap();
        delay.delay_us(1u8);
        reset.set_high().unwrap();

        Tca9548 {
            i2c,
            address,
        }
    }

    //pub fn default<RST, DELAY>(i2c: I2C, reset: &mut RST, delay: &mut DELAY) -> Self
    //where
    //    RST: OutputPin,
    //    DELAY: DelayUs<u8>,
    //    RST::Error: core::fmt::Debug,
    //{
    //    Tca9548::new(i2c, 0x70, reset, delay)
    //}
    pub fn default(i2c: I2C) -> Self {
        Tca9548 {
            address: 0x70,
            i2c: i2c,
        }
    }

    pub fn select_bus(&mut self, bus: Bus) -> Result<(), I2C::Error> {
        self.i2c.write(self.address, &[bus as u8])
    }
}
