//! Driver for the MAX6639 fan speed controller
//!
//! # Copyright
//! Copyright (C) 2020 QUARTIQ GmbH - All Rights Reserved
//! Unauthorized usage, editing, or copying is strictly prohibited.
//! Proprietary and confidential.
//!
//! # Description
//! This is a minimalistic driver that allows manual PWM control of both fans.
#![no_std]
#![deny(warnings)]

use bit_field::BitField;
use embedded_hal::blocking::i2c::{Write, WriteRead};

/// The driver representing the programmable reference generator.
pub struct Max6639<I2C>
where
    I2C: Write + WriteRead,
{
    i2c: I2C,
    address: u8,
}

/// Represents the various states of the address pin on the device. The value of the enum
/// corresponds to the 7-bit I2C bus address it represents.
pub enum AddressPin {
    Pullup = 0x2F,
    Float = 0x2E,
    Pulldown = 0x2C,
}

#[doc(hidden)]
#[allow(dead_code)]
#[derive(Copy, Clone, Debug)]
/// Represents various registers that can be read or written on the controller.
enum Register {
    Status = 0x02,
    GlobalConfig = 0x04,
    Fan1Config1 = 0x10,
    Fan1Config2a = 0x11,
    Fan1Config2b = 0x12,
    Fan1Config3 = 0x13,
    Fan2Config1 = 0x14,
    Fan2Config2a = 0x15,
    Fan2Config2b = 0x16,
    Fan2Config3 = 0x17,

    Fan1TachCount = 0x20,
    Fan2TachCount = 0x21,

    // Note: The duty cycle register can be written to the target duty cycle, but will read out the
    // current duty cycle.
    Fan1DutyCycle = 0x26,
    Fan2DutyCycle = 0x27,

    DeviceId = 0x3d,
    ManufacturerId = 0x3e,
    DeviceRevision = 0x3f,
}

/// An error that the fan controller driver may encounter.
#[derive(Copy, Clone, Debug)]
pub enum Error<E> {
    Interface(E),
    FanFail,
    Bounds,
}

impl<E> From<E> for Error<E> {
    fn from(err: E) -> Error<E> {
        Error::Interface(err)
    }
}

/// An indication of which fan to operate on.
#[derive(Copy, Clone, Debug)]
pub enum Fan {
    Fan1,
    Fan2,
}

impl<I2C> Max6639<I2C>
where
    I2C: Write + WriteRead,
    <I2C as Write>::Error: Into<<I2C as WriteRead>::Error>,
{
    /// Create a new MAX6639 driver.
    ///
    /// # Args
    /// * `i2c` - The I2C interface used to communicate with the device.
    /// * `address_pin` - the pin state of the ADD input pin.
    pub fn new(
        i2c: I2C,
        address_pin: AddressPin,
    ) -> Result<Self, Error<<I2C as WriteRead>::Error>> {
        let mut device = Max6639 {
            i2c,
            address: address_pin as u8,
        };

        // Initiate a power-on-reset of the device.
        device.write(Register::GlobalConfig, 1 << 6)?;

        // Configure both fans for PWM control mode with the fan off.
        device.write(Register::Fan1DutyCycle, 0)?;
        device.write(Register::Fan2DutyCycle, 0)?;

        // Run the tachometer clocks at 4KHz and select PWM control mode.
        device.write(Register::Fan1Config1, 1 << 7 | 0b10)?;
        device.write(Register::Fan2Config1, 1 << 7 | 0b10)?;

        Ok(device)
    }

    fn write(
        &mut self,
        register: Register,
        value: u8,
    ) -> Result<(), Error<<I2C as WriteRead>::Error>> {
        let write_data: [u8; 2] = [register as u8, value];

        self.i2c
            .write(self.address, &write_data)
            .map_err(|err| err.into())?;

        Ok(())
    }

    fn read(&mut self, register: Register) -> Result<u8, Error<<I2C as WriteRead>::Error>> {
        let mut result: [u8; 1] = [0; 1];

        self.i2c
            .write_read(self.address, &[register as u8], &mut result)?;

        Ok(result[0])
    }

    /// Configure the duty cycle of a fan.
    ///
    /// # Note
    /// This function will check for fan faults on the configured fan and indicate an error if a
    /// fault is detected.
    ///
    /// # Args
    /// * `fan` - Specifies which fan the duty cycle should be configured on.
    /// * `duty_cycle` - The normalized duty cycle desired for the fan.
    pub fn set_duty_cycle(
        &mut self,
        fan: Fan,
        duty_cycle: f32,
    ) -> Result<(), Error<<I2C as WriteRead>::Error>> {
        if duty_cycle < 0.0 || duty_cycle > 1.0 {
            return Err(Error::Bounds);
        }

        let register_value = (duty_cycle * 120.0) as u8;

        let register = match fan {
            Fan::Fan1 => Register::Fan1DutyCycle,
            Fan::Fan2 => Register::Fan2DutyCycle,
        };

        self.write(register, register_value)?;

        // Check the fan for fan-fault detections. Modifying the duty cycle won't help if the fan
        // isn't operating nominally.
        if self.check_fan_fault(fan)? {
            Err(Error::FanFail)
        } else {
            Ok(())
        }
    }

    /// Check if a fan has had a fault.
    ///
    /// # Note
    /// This will erase fault status on the other fan until the next check cycle.
    ///
    /// # Args
    /// * `fan` - Specifies which fan to check.
    ///
    /// # Return
    /// True if a fan fault was detected. False otherwise.
    pub fn check_fan_fault(&mut self, fan: Fan) -> Result<bool, Error<<I2C as WriteRead>::Error>> {
        // Note that this register read will erase all status indications.
        let status_register = self.read(Register::Status)?;

        match fan {
            Fan::Fan1 => Ok(status_register.get_bit(1)),
            Fan::Fan2 => Ok(status_register.get_bit(0)),
        }
    }

    /// Get the current RPMs of the fan.
    ///
    /// # Args
    /// * `fan` - The fan to get the RPM count of.
    ///
    /// # Returns
    /// The current fan speed in RPMs (revolutions per minute).
    pub fn current_rpms(&mut self, fan: Fan) -> Result<u16, Error<<I2C as WriteRead>::Error>> {
        let tach_reg = match fan {
            Fan::Fan1 => Register::Fan1TachCount,
            Fan::Fan2 => Register::Fan2TachCount,
        };

        let tach_count = self.read(tach_reg)?;

        // If the tachometer registers 0xFF, we can't measure the RPMs. Assume the fan is stopped.
        if tach_count == 0xFF {
            Ok(0)
        } else {
            // The tachometer clock is configured for 4KHz. The RPMs are calculated as follows:
            // RPM = (Tach_Clk * 60) / tach_count
            Ok(((60 * 4000) as f32 / tach_count as f32) as u16)
        }
    }
}
