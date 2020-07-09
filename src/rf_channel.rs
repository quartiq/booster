//! Definitions for Booster RF management channels.
//!
//! # Copyright
//! Copyright (C) 2020 QUARTIQ GmbH - All Rights Reserved
//! Unauthorized usage, editing, or copying is strictly prohibited.
//! Proprietary and confidential.
use ad5627::{self, Ad5627};
use ads7924::Ads7924;
use dac7571::Dac7571;
use max6642::Max6642;
use mcp3221::Mcp3221;
use microchip_24aa02e48::Microchip24AA02E48;

use super::{BusManager, BusProxy, I2C};
use crate::error::Error;
use stm32f4xx_hal::{
    self as hal,
    gpio::{Analog, Floating, Input, Output, PullDown, PushPull},
    prelude::*,
};

// Convenience type definition for all I2C devices on the bus.
type I2cDevice = BusProxy<I2C>;

/// A collection of analog pins (ADC channels) associated with an RF channel.
#[allow(dead_code)]
pub struct AnalogPins<AdcPin> {
    tx_power_analog_pin: AdcPin,
    reflected_power_analog_pin: AdcPin,
}

/// Enumeration containing ADC pins associated with an RF channel.
///
/// # Note
/// This is a convenience enumeration to allow an `RfChannel` to be generic across analog pin types.
pub enum AdcPins {
    PortA(AnalogPins<hal::gpio::gpioa::PA<Analog>>),
    PortC(AnalogPins<hal::gpio::gpioc::PC<Analog>>),
    PortF(AnalogPins<hal::gpio::gpiof::PF<Analog>>),
}

impl AdcPins {
    /// Construct a pair of ADC pins from the GPIOA port.
    ///
    /// # Args
    /// * `tx_power` The TX-power analog input pin.
    /// * `reflected_power` The reflected -power analog input pin.
    pub fn gpioa(
        tx_power: hal::gpio::gpioa::PA<Analog>,
        reflected_power: hal::gpio::gpioa::PA<Analog>,
    ) -> Self {
        AdcPins::PortA(AnalogPins {
            tx_power_analog_pin: tx_power,
            reflected_power_analog_pin: reflected_power,
        })
    }

    /// Construct a pair of ADC pins from the GPIOC port.
    ///
    /// # Args
    /// * `tx_power` The TX-power analog input pin.
    /// * `reflected_power` The reflected -power analog input pin.
    pub fn gpioc(
        tx_power: hal::gpio::gpioc::PC<Analog>,
        reflected_power: hal::gpio::gpioc::PC<Analog>,
    ) -> Self {
        AdcPins::PortC(AnalogPins {
            tx_power_analog_pin: tx_power,
            reflected_power_analog_pin: reflected_power,
        })
    }

    /// Construct a pair of ADC pins from the GPIOF port.
    ///
    /// # Args
    /// * `tx_power` The TX-power analog input pin.
    /// * `reflected_power` The reflected -power analog input pin.
    pub fn gpiof(
        tx_power: hal::gpio::gpiof::PF<Analog>,
        reflected_power: hal::gpio::gpiof::PF<Analog>,
    ) -> Self {
        AdcPins::PortF(AnalogPins {
            tx_power_analog_pin: tx_power,
            reflected_power_analog_pin: reflected_power,
        })
    }
}

/// Represents all of the I2C devices on the bus for a single RF channel.
#[allow(dead_code)]
pub struct Devices {
    interlock_thresholds_dac: Ad5627<I2cDevice>,
    input_power_adc: Mcp3221<I2cDevice>,
    temperature_monitor: Max6642<I2cDevice>,
    bias_dac: Dac7571<I2cDevice>,
    power_monitor: Ads7924<I2cDevice>,
    pub eui48: Microchip24AA02E48<I2cDevice>,
}

impl Devices {
    /// Check if an RF channel is available and construct devices for it.
    ///
    /// # Note
    /// This function will and probe devices on the RF channel to see if the module is installed.
    ///
    /// # Args
    /// * `manager` - The I2C bus manager used interfacing with devices on the I2C bus.
    /// * `control_pins` - The pins used for interacting with the RF channel.
    ///
    /// # Returns
    /// An option containing the devices if they were discovered on the bus. If any device did not
    /// properly enumerate, the option will be empty.
    fn new(manager: &'static BusManager) -> Option<Self> {
        // The ADS7924 and DAC7571 are present on the booster mainboard, so instantiation
        // and communication should never fail.
        let mut dac7571 = Dac7571::default(manager.acquire());

        // Ensure the bias DAC is placing the RF amplifier in pinch off (disabled).
        dac7571.set_voltage(3.3).expect("Bias DAC did not respond");

        // Verify we can communicate with the power monitor.
        let mut ads7924 =
            Ads7924::default(manager.acquire()).expect("Power monitor did not enumerate");
        ads7924
            .get_voltage(ads7924::Channel::Three)
            .expect("Power monitor did not respond");

        if let Ok(ad5627) = Ad5627::default(manager.acquire()) {
            if let Ok(eui48) = Microchip24AA02E48::new(manager.acquire()) {
                // Query devices on the RF module to verify they are present.
                let mut max6642 = Max6642::att94(manager.acquire());
                if let Err(_) = max6642.get_remote_temperature() {
                    return None;
                }

                let mut mcp3221 = Mcp3221::default(manager.acquire());
                if let Err(_) = mcp3221.get_voltage() {
                    return None;
                }

                return Some(Self {
                    interlock_thresholds_dac: ad5627,
                    input_power_adc: mcp3221,
                    temperature_monitor: max6642,
                    bias_dac: dac7571,
                    power_monitor: ads7924,
                    eui48: eui48,
                });
            }
        }

        None
    }
}

/// Represents the control and status pins for an RF channel.
#[allow(dead_code)]
pub struct ChannelPins {
    enable_power: hal::gpio::gpiod::PD<Output<PushPull>>,

    // The alert and input overdrive pins have external pull resistors, so we don't need to pull
    // them internall.
    alert: hal::gpio::gpiod::PD<Input<Floating>>,
    input_overdrive: hal::gpio::gpioe::PE<Input<Floating>>,

    // There are no pullup/pulldown resistors on this input, so we will pull it down internally.
    output_overdrive: hal::gpio::gpioe::PE<Input<PullDown>>,

    signal_on: hal::gpio::gpiog::PG<Output<PushPull>>,

    adc_pins: AdcPins,
}

impl ChannelPins {
    /// Construct new set of control pins.
    ///
    /// # Args
    /// * `enable_power_pin` - An output pin used to power up the RF channel.
    /// * `alert_pin` - An input pin monitoring the status of the RF power module. This is connected
    ///   to the ADS7924 alert output.
    /// * `input_overdrive_pin` - An input pin that indicates an input overdrive.
    /// * `output_overdrive_pin` - An input pin that indicates an output overdrive.
    /// * `signal_on_pin` - An output pin that is set high to enable output signal amplification.
    pub fn new(
        enable_power: hal::gpio::gpiod::PD<Output<PushPull>>,
        alert: hal::gpio::gpiod::PD<Input<Floating>>,
        input_overdrive: hal::gpio::gpioe::PE<Input<Floating>>,
        output_overdrive: hal::gpio::gpioe::PE<Input<PullDown>>,
        signal_on: hal::gpio::gpiog::PG<Output<PushPull>>,
        adc_pins: AdcPins,
    ) -> Self {
        let mut pins = Self {
            enable_power,
            alert,
            input_overdrive,
            output_overdrive,
            signal_on,
            adc_pins,
        };

        pins.power_down_channel();

        pins
    }

    /// Power down a channel.
    fn power_down_channel(&mut self) {
        self.signal_on.set_low().unwrap();
        self.enable_power.set_low().unwrap();
    }
}

/// Represents a means of interacting with an RF output channel.
#[allow(dead_code)]
pub struct RfChannel {
    pub i2c_devices: Devices,
    pins: ChannelPins,
}

impl RfChannel {
    /// Construct a new RF channel.
    ///
    /// # Note
    /// This function attempts to detect an installed RF module.
    ///
    /// # Args
    /// * `manager` - The manager that controls the shared I2C bus used for RF module devices.
    /// * `control_pins` - The control and status pins associated with the channel.
    ///
    /// # Returns
    /// An option containing an RfChannel if a channel was discovered on the bus. None otherwise.
    pub fn new(manager: &'static BusManager, control_pins: ChannelPins) -> Option<Self> {
        // Attempt to instantiate the I2C devices on the channel.
        match Devices::new(manager) {
            // TODO: Configure alerts/alarms for the power monitor.
            Some(devices) => Some(Self {
                i2c_devices: devices,
                pins: control_pins,
            }),
            None => None,
        }
    }

    /// Set the interlock thresholds for the channel.
    ///
    /// # Args
    /// * `output` - The dBm interlock threshold to configure for the output power.
    /// * `reflected` - The dBm interlock threshold to configure for reflected power.
    pub fn set_interlock_thresholds(&mut self, output: f32, reflected: f32) -> Result<(), Error> {
        // When operating at 100MHz, the power detectors specify the following output
        // characteristics for -10 dBm to 10 dBm (the equation uses slightly different coefficients
        // for different power levels and frequencies):
        //
        // dBm = V(Vout) / .035 V/dB - 35.6 dBm
        //
        // Because we're comparing the output of the detector with an analog comparator, we need to
        // scale the provided power thresholds into analog voltages comparable to the output of the
        // detectors. To accomplish this, we invert the equation.

        // The reflected power detector is then passed through an op-amp with gain 1.5x - this
        // modifies the slope from 35mV/dB to 52.5mV/dB
        let voltage = (reflected + 35.6) * 0.0525;
        match self
            .i2c_devices
            .interlock_thresholds_dac
            .set_voltage(voltage, ad5627::Dac::A)
        {
            Err(ad5627::Error::Range) => return Err(Error::Bounds),
            Err(ad5627::Error::I2c(_)) => return Err(Error::Interface),
            Ok(_) => {}
        }

        // The output power detector passes through an op-amp with unity gain (1.0x) - the power
        // detector equation is not modified.
        let voltage = (output + 35.6) * 0.035;
        match self
            .i2c_devices
            .interlock_thresholds_dac
            .set_voltage(voltage, ad5627::Dac::B)
        {
            Err(ad5627::Error::Range) => return Err(Error::Bounds),
            Err(ad5627::Error::I2c(_)) => return Err(Error::Interface),
            Ok(_) => {}
        }

        Ok(())
    }

    pub fn is_overdriven(&self) -> bool {
        let input_overdrive = self.pins.input_overdrive.is_low().unwrap();
        let output_overdrive = self.pins.output_overdrive.is_low().unwrap();

        input_overdrive || output_overdrive
    }

    pub fn is_enabled(&self) -> bool {
        let enabled = self.pins.enable_power.is_high().unwrap() &&
            self.pins.signal_on.is_high().unwrap();

        // TODO: Check that the bias is out of pinch off?

        enabled && !self.is_overdriven()
    }

    pub fn is_alarmed(&self) -> bool {
        self.pins.alert.is_low().unwrap()
    }

    pub fn enable(&mut self) -> Result<(), Error> {
        // TODO: Power-up the channel.
        Err(Error::NotImplemented)
    }

    pub fn disable(&mut self) -> Result<(), Error> {
        self.pins.power_down_channel();

        // Set the bias DAC output into pinch-off.
        self.i2c_devices.bias_dac.set_voltage(-3.3).expect("Failed to disable RF bias voltage");

        Ok(())
    }

    pub fn get_temperature(&mut self) -> Result<f32, Error> {
        self.i2c_devices.temperature_monitor.get_remote_temperature().map_err(|_| Error::Interface)
    }

    pub fn set_bias(&mut self, bias_voltage: f32) -> Result<f32, Error> {
        // The bias voltage is the inverse of the DAC output voltage.
        let bias_voltage = -1.0 * bias_voltage;

        let set_voltage = match self.i2c_devices.bias_dac.set_voltage(bias_voltage) {
            Err(dac7571::Error::Bounds) => return Err(Error::Bounds),
            Err(_) => panic!("Failed to set DAC bias voltage"),
            Ok(voltage) => voltage,
        };

        Ok(-1.0 * set_voltage)
    }
}
