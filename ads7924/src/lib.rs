//! Driver for the ADS7924 external ADC.
#![no_std]
#![deny(warnings)]

use core::convert::TryInto;

use bit_field::BitField;

use embedded_hal::{
    delay::DelayNs,
    i2c::{ErrorType, I2c},
};

/// A driver for the ADS7924 4-channel analog-to-digital converter.
pub struct Ads7924<I2C>
where
    I2C: I2c,
{
    i2c: I2C,
    address: u8,
    volts_per_lsb: f32,
}

#[derive(Copy, Clone, PartialEq)]
#[doc(hidden)]
#[allow(dead_code)]
enum OperationMode {
    Idle = 0b000000,
    Awake = 0b100000,
    ManualSingle = 0b110000,
    AutoscanWithSleep = 0b111011,
}

#[derive(Copy, Clone)]
#[doc(hidden)]
#[allow(dead_code)]
enum Register {
    ModeCntrl = 0x00,
    IntCntrl = 0x01,
    Data0Upper = 0x02,
    Data0Lower = 0x03,
    Data1Upper = 0x04,
    Data1Lower = 0x05,
    Data2Upper = 0x06,
    Data2Lower = 0x07,
    Data3Upper = 0x08,
    Data3Lower = 0x09,
    ULR0 = 0x0A,
    LLR0 = 0x0B,
    ULR1 = 0x0C,
    LLR1 = 0x0D,
    ULR2 = 0x0E,
    LLR2 = 0x0F,
    ULR3 = 0x10,
    LLR3 = 0x11,
    IntConfig = 0x12,
    SleepConfig = 0x13,
    AcquireConfig = 0x14,
    PowerConfig = 0x15,
    Reset = 0x16,
}

/// Indicates an ADC sample channel.
#[derive(Copy, Clone)]
pub enum Channel {
    Zero = 0,
    One = 1,
    Two = 2,
    Three = 3,
}

/// Indicates errors that the ADC may encounter.
#[derive(Debug)]
pub enum Error<E> {
    Interface(E),
    Size,
    Bounds,
}

impl<E> From<E> for Error<E> {
    fn from(err: E) -> Error<E> {
        Error::Interface(err)
    }
}

impl<I2C> Ads7924<I2C>
where
    I2C: I2c,
    <I2C as ErrorType>::Error: Into<<I2C as ErrorType>::Error>,
{
    /// Create a new ADC driver.
    ///
    /// # Args
    /// * `i2c` - The I2C interface to use to communicate with the device.
    /// * `address` - the I2C address of the device.
    /// * `avdd` - The voltage connected to the AVDD terminal.
    /// * `delay` - A means of delaying during initialization.
    pub fn new(
        i2c: I2C,
        address: u8,
        avdd: f32,
        delay: &mut impl DelayNs,
    ) -> Result<Self, Error<<I2C as ErrorType>::Error>> {
        let mut ads7924 = Ads7924 {
            i2c,
            address,
            volts_per_lsb: avdd / 0x1000 as f32,
        };

        ads7924.reset(delay)?;

        // Bring the ADC from idle to awake mode unconditionally.
        ads7924.set_mode(OperationMode::Awake, None)?;

        // Configure the interrupt pin to operate in alarm mode when thresholds are exceeded once.
        let interrupt_config = *0u8.set_bits(5..8, 0b1);
        ads7924.write(Register::IntConfig, &[interrupt_config])?;

        // Configure the ADC to operate in auto-scan (with sleep) mode.
        ads7924.set_mode(OperationMode::AutoscanWithSleep, None)?;

        Ok(ads7924)
    }

    /// Create a default ADC driver.
    ///
    /// # Note
    /// A default driver assumes a 3.434V power supply and the A0 pin pulled high.
    ///
    /// # Args
    /// * `i2c` - The I2C interface to use to communicate with the device.
    /// * `delay` - A means of delaying during initialization.
    pub fn default(
        i2c: I2C,
        delay: &mut impl DelayNs,
    ) -> Result<Self, Error<<I2C as ErrorType>::Error>> {
        Ads7924::new(i2c, 0x49, 3.434, delay)
    }

    fn set_mode(
        &mut self,
        mode: OperationMode,
        channel: Option<Channel>,
    ) -> Result<(), Error<<I2C as ErrorType>::Error>> {
        let mut mode_control: [u8; 1] = [0];
        if let Some(channel) = channel {
            mode_control[0].set_bits(0..3, channel as u8);
        }

        mode_control[0].set_bits(2..8, mode as u8);
        self.write(Register::ModeCntrl, &mode_control)?;

        Ok(())
    }

    fn reset(
        &mut self,
        delay: &mut impl DelayNs,
    ) -> Result<(), Error<<I2C as ErrorType>::Error>> {
        self.write(Register::Reset, &[0xAA])?;

        // Wait a small delay to ensure the device is processing the reset request.
        delay.delay_us(500_u32);

        Ok(())
    }

    fn write(
        &mut self,
        register: Register,
        data: &[u8],
    ) -> Result<(), Error<<I2C as ErrorType>::Error>> {
        if data.len() > 2 {
            return Err(Error::Size);
        }

        let mut write_data: [u8; 3] = [register as u8, 0, 0];
        write_data[1..][..data.len()].copy_from_slice(data);

        // Set the INC bit in the register address byte if writing more than one register.
        if data.len() > 1 {
            write_data[0].set_bit(7, true);
        }

        self.i2c
            .write(self.address, &write_data[..data.len() + 1])
            .map_err(|err| err.into())?;

        Ok(())
    }

    fn read(
        &mut self,
        register: Register,
        data: &mut [u8],
    ) -> Result<(), Error<<I2C as ErrorType>::Error>> {
        let mut command_byte = register as u8;

        // Set the INC bit in the command byte if reading more than 1 register.
        if data.len() > 1 {
            command_byte.set_bit(7, true);
        }

        self.i2c.write_read(self.address, &[command_byte], data)?;

        Ok(())
    }

    /// Configure and enable alarm thresholds for a channel.
    ///
    /// # Args
    /// * `channel` - The channel to configure thresholds for.
    /// * `low_threshold` - The lower threshold desired in volts.
    /// * `high_threshold` - The upper threshold desired in volts.
    pub fn set_thresholds(
        &mut self,
        channel: Channel,
        low_threshold: f32,
        high_threshold: f32,
    ) -> Result<(), Error<<I2C as ErrorType>::Error>> {
        if high_threshold < low_threshold || low_threshold < 0.0 || high_threshold < 0.0 {
            return Err(Error::Bounds);
        }

        let upper_limit_register = match channel {
            Channel::Zero => Register::ULR0,
            Channel::One => Register::ULR1,
            Channel::Two => Register::ULR2,
            Channel::Three => Register::ULR3,
        };

        // Convert the voltages to ADC codes.
        let low_threshold_code = (low_threshold / self.volts_per_lsb) as u16;
        let high_threshold_code = (high_threshold / self.volts_per_lsb) as u16;

        if low_threshold_code >= 0x1000 || high_threshold_code >= 0x1000 {
            return Err(Error::Bounds);
        }

        // The thresholds are configured using only the 8 most significant bits.
        let low_threshold_code = (low_threshold_code >> 4) as u8;
        let high_threshold_code = (high_threshold_code >> 4) as u8;

        self.write(
            upper_limit_register,
            &[high_threshold_code, low_threshold_code],
        )?;

        // Enable the alarm in the interrupt control register.
        let mut interrupt_control_register: [u8; 1] = [0];
        self.read(Register::IntCntrl, &mut interrupt_control_register)?;

        interrupt_control_register[0].set_bit(channel as usize, true);

        self.write(Register::IntCntrl, &interrupt_control_register)?;

        Ok(())
    }

    /// Clear the any pending alarm state of the device.
    ///
    /// # Returns
    /// A bit mask of which channel caused the alarm. The position of the bit corresponds with the
    /// channel number.
    pub fn clear_alarm(&mut self) -> Result<u8, Error<<I2C as ErrorType>::Error>> {
        // Clearing the alarm is completed by reading the interrupt control register.
        let mut alarm_status: [u8; 1] = [0];
        self.read(Register::IntCntrl, &mut alarm_status)?;

        // To determine the cause of the alarm, logically and the alarm enable bits with the alarm
        // status bits.
        let alarm_enable: u8 = alarm_status[0].get_bits(0..4);
        let alarm_status: u8 = alarm_status[0].get_bits(4..8);

        Ok(alarm_status & alarm_enable)
    }

    /// Get the analog voltage of a channel.
    ///
    /// # Args
    /// * `channel` - The channel to get the voltage of.
    ///
    /// # Returns
    /// The analog measurement of the specified channel in volts.
    pub fn get_voltage(
        &mut self,
        channel: Channel,
    ) -> Result<f32, Error<<I2C as ErrorType>::Error>> {
        let upper_data_register = match channel {
            Channel::Zero => Register::Data0Upper,
            Channel::One => Register::Data1Upper,
            Channel::Two => Register::Data2Upper,
            Channel::Three => Register::Data3Upper,
        };
        // First, disable Autoscan mode.
        self.set_mode(OperationMode::Idle, None)?;

        let mut voltage_register: [u8; 2] = [0; 2];
        self.read(upper_data_register, &mut voltage_register)?;

        // Reenable Autoscan.
        self.set_mode(OperationMode::Awake, None)?;
        self.set_mode(OperationMode::AutoscanWithSleep, None)?;

        // Convert the voltage register to an ADC code. The code is stored MSB-aligned, so we need
        // to shift it back into alignment.
        let code = u16::from_be_bytes(voltage_register) >> 4;

        Ok(code as f32 * self.volts_per_lsb)
    }

    /// Get the analog voltages of all channels.
    ///
    /// # Returns
    /// The analog measurements of all channel in volts.
    pub fn get_voltages(&mut self) -> Result<[f32; 4], Error<<I2C as ErrorType>::Error>> {
        // First, disable Autoscan mode.
        self.set_mode(OperationMode::Idle, None)?;

        // Read all ADC data registers
        let mut data = [0u8; 4 * 2];
        self.read(Register::Data0Upper, &mut data)?;

        // Reenable Autoscan.
        self.set_mode(OperationMode::Awake, None)?;
        self.set_mode(OperationMode::AutoscanWithSleep, None)?;

        let mut voltages = [0f32; 4];
        // Convert the voltage registers to an ADC code. The code is stored MSB-aligned,
        // so we need to shift it back into alignment.
        for i in 0..4 {
            let code = u16::from_be_bytes(data[2 * i..2 * (i + 1)].try_into().unwrap());
            voltages[i] = (code >> 4) as f32 * self.volts_per_lsb;
        }

        Ok(voltages)
    }
}
