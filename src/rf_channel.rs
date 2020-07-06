use ad5627::{self, Ad5627};

use super::{BusProxy, I2C};
use crate::error::Error;

pub struct RfChannel {
    interlock_thresholds: Ad5627<BusProxy<I2C>>,
}

impl RfChannel {
    pub fn new(ad5627: Ad5627<BusProxy<I2C>>) -> Self {
        Self {
            interlock_thresholds: ad5627,
        }
    }

    pub fn set_interlock_thresholds(&mut self, output: f32, reflected: f32) -> Result<(), Error> {
        // From the power detectors, dBm = Vout / 0.035 - 35.7;
        let voltage = (reflected + 35.6) * 0.055;
        match self
            .interlock_thresholds
            .set_voltage(voltage, ad5627::Dac::A)
        {
            Err(ad5627::Error::Range) => return Err(Error::Bounds),
            Err(ad5627::Error::I2c(_)) => return Err(Error::Interface),
            Ok(_) => {}
        }

        let voltage = (output + 35.6) * 0.055;
        match self
            .interlock_thresholds
            .set_voltage(voltage, ad5627::Dac::B)
        {
            Err(ad5627::Error::Range) => return Err(Error::Bounds),
            Err(ad5627::Error::I2c(_)) => return Err(Error::Interface),
            Ok(_) => {}
        }

        Ok(())
    }
}
