//! Definitions for Booster RF management channels.
//!
//! # Copyright
//! Copyright (C) 2020 QUARTIQ GmbH - All Rights Reserved
//! Unauthorized usage, editing, or copying is strictly prohibited.
//! Proprietary and confidential.
use ad5627::{self, Ad5627};
use max6642::Max6642;
use mcp3221::Mcp3221;
use ads7924::Ads7924;
use dac7571::Dac7571;

use super::{BusManager, BusProxy, I2C};
use crate::error::Error;
use stm32f4xx_hal::{
    self as hal,
    gpio::{
        Input,
        Output,
        PushPull,
        PullDown,
        Floating,
    },
};

type I2cDevice = BusProxy<I2C>;

#[allow(dead_code)]
pub struct Devices {
    interlock_thresholds_dac: Ad5627<I2cDevice>,
    input_power_adc: Mcp3221<I2cDevice>,
    temperature_monitor: Max6642<I2cDevice>,
    bias_dac: Dac7571<I2cDevice>,
    power_monitor: Ads7924<I2cDevice>,
    // TODO: Incorporate the EUI-48 EEPROM.
}

impl Devices {
    pub fn new(manager: &'static BusManager) -> Option<Self> {
        if let Ok(ad5627) = Ad5627::default(manager.acquire()) {
            // The ADS7924 is present on the booster mainboard, so instantiation should never fail.
            let ads7924 = Ads7924::default(manager.acquire()).unwrap();
            let dac7571 = Dac7571::default(manager.acquire());

            // TODO: Query these devices to verify they are present.
            let max6642 = Max6642::att94(manager.acquire());
            let mcp3221 = Mcp3221::default(manager.acquire());

            // TODO: Incorporate EUI-48.

            Some(Self {
                interlock_thresholds_dac: ad5627,
                input_power_adc: mcp3221,
                temperature_monitor: max6642,
                bias_dac: dac7571,
                power_monitor: ads7924,
            })
        } else {
            None
        }
    }
}

#[allow(dead_code)]
pub struct ControlPins {
    enable_power_pin: hal::gpio::gpiod::PD<Output<PushPull>>,

    // The alert and input overdrive pins have external pull resistors, so we don't need to pull
    // them internall.
    alert_pin: hal::gpio::gpiod::PD<Input<Floating>>,
    input_overdrive_pin: hal::gpio::gpioe::PE<Input<Floating>>,

    // There are no pullup/pulldown resistors on this input, so we will pull it down internally.
    output_overdrive_pin: hal::gpio::gpioe::PE<Input<PullDown>>,

    signal_on_pin: hal::gpio::gpiog::PG<Output<PushPull>>,
}

impl ControlPins {
    pub fn new(
        enable_power_pin: hal::gpio::gpiod::PD<Output<PushPull>>,
        alert_pin: hal::gpio::gpiod::PD<Input<Floating>>,
        input_overdrive_pin: hal::gpio::gpioe::PE<Input<Floating>>,
        output_overdrive_pin: hal::gpio::gpioe::PE<Input<PullDown>>,
        signal_on_pin: hal::gpio::gpiog::PG<Output<PushPull>>,
    ) -> Self {
        Self {
            enable_power_pin,
            alert_pin,
            input_overdrive_pin,
            output_overdrive_pin,
            signal_on_pin
        }
    }
}

#[allow(dead_code)]
pub struct RfChannel {
    i2c_devices: Devices,
    io_pins: ControlPins,
}

impl RfChannel {
    pub fn new(devices: Devices, control_pins: ControlPins) -> Self {
        Self {
            i2c_devices: devices,
            io_pins: control_pins,
        }
    }

    pub fn set_interlock_thresholds(&mut self, output: f32, reflected: f32) -> Result<(), Error> {
        // From the power detectors, dBm = Vout / 0.035 - 35.7;
        let voltage = (reflected + 35.6) * 0.055;
        match self.i2c_devices
            .interlock_thresholds_dac
            .set_voltage(voltage, ad5627::Dac::A)
        {
            Err(ad5627::Error::Range) => return Err(Error::Bounds),
            Err(ad5627::Error::I2c(_)) => return Err(Error::Interface),
            Ok(_) => {}
        }

        let voltage = (output + 35.6) * 0.055;
        match self.i2c_devices
            .interlock_thresholds_dac
            .set_voltage(voltage, ad5627::Dac::B)
        {
            Err(ad5627::Error::Range) => return Err(Error::Bounds),
            Err(ad5627::Error::I2c(_)) => return Err(Error::Interface),
            Ok(_) => {}
        }

        Ok(())
    }
}
