//! Definitions for Booster RF management channels.
//!
//! # Copyright
//! Copyright (C) 2020 QUARTIQ GmbH - All Rights Reserved
//! Unauthorized usage, editing, or copying is strictly prohibited.
//! Proprietary and confidential.
use crate::linear_transformation::LinearTransformation;
use ad5627::{self, Ad5627};
use ads7924::Ads7924;
use dac7571::Dac7571;
use max6642::Max6642;
use mcp3221::Mcp3221;
use microchip_24aa02e48::Microchip24AA02E48;

use super::{BusManager, BusProxy, I2C};
use crate::error::Error;
use embedded_hal::blocking::delay::DelayUs;
use stm32f4xx_hal::{
    self as hal,
    adc::config::SampleTime,
    gpio::{Analog, Floating, Input, Output, PullDown, PushPull},
    prelude::*,
};

use rtic::cyccnt::{Instant, Duration};

// Convenience type definition for all I2C devices on the bus.
type I2cDevice = BusProxy<I2C>;

/// A structure representing power supply measurements of a channel.
pub struct SupplyMeasurements {
    pub v_p5v0mp: f32,
    pub i_p5v0ch: f32,
    pub i_p28v0ch: f32,
}

/// The current state of an RF channel.
pub enum ChannelState {
    /// The channel output is disabled.
    Disabled,

    /// The channel is in the enabling process.
    Enabling(Instant),

    /// The channel is actively outputting.
    Active,
}

// Macro magic to generate an enum that looks like:
//
// ```rust
// pub enum AdcPins {
//     PA0(hal::gpio::gpioa::PA0<Analog>),
//     // ...
// }
//
// impl AdcPins {
//     pub fn pa0(pin: hal::gpio::gpioa::PA0<Analog>) -> Self {
//         AdcPins::PA0(pin)
//     }
//
//     pub fn convert(&self, adc: &mut hal::adc::Adc<hal::stm32::ADC3>, sample_time: SampleTime) -> u16 {
//         match self {
//             AdcPin::PA0(pin) => adc.convert(pin, sample_time)
//             // ...
//         }
//     }
// }
// ```
//
// This allows storing non-generic pin types into a single enumeration type.
macro_rules! adc_pins {
    (define: [$($pin:ident, $pin_lower:ident, $port:ident),*]) => {
        pub enum AdcPin {
            $(
             $pin(hal::gpio::$port::$pin<Analog>),
             )*
        }
    };

    (implement: $pin_defs:tt) => {
        impl AdcPin {
            adc_pins!(implement_convert: $pin_defs);
            adc_pins!(implement_new: $pin_defs);
        }
    };

    (implement_new: [$($pin:ident, $pin_lower:ident, $port:ident),*]) => {
        $(
        pub fn $pin_lower(pin: hal::gpio::$port::$pin<Analog>) -> Self {
            AdcPin::$pin(pin)
        }
        )*
    };

    (implement_convert: [$($pin:ident, $pin_lower:ident, $port:ident),*]) => {
        pub fn convert(&self, adc: &mut hal::adc::Adc<hal::stm32::ADC3>, sample_time: SampleTime) -> u16 {
            match self {
                $(
                 AdcPin::$pin(pin) => adc.convert(pin, sample_time),
                 )*
            }
        }
    };

    ($pin_defs:tt) => {
        adc_pins!(define: $pin_defs);
        adc_pins!(implement: $pin_defs);
    };
}
adc_pins!([
    PA0, pa0, gpioa, PA1, pa1, gpioa, PA2, pa2, gpioa, PA3, pa3, gpioa, PC0, pc0, gpioc, PC1, pc1,
    gpioc, PC2, pc2, gpioc, PC3, pc3, gpioc, PF3, pf3, gpiof, PF4, pf4, gpiof, PF5, pf5, gpiof,
    PF6, pf6, gpiof, PF7, pf7, gpiof, PF8, pf8, gpiof, PF9, pf9, gpiof, PF10, pf10, gpiof
]);

/// A collection of analog pins (ADC channels) associated with an RF channel.
pub struct AnalogPins {
    pub tx_power: AdcPin,
    pub reflected_power: AdcPin,
}

impl AnalogPins {
    /// Create a new analog pin structure.
    ///
    /// # Args
    /// * `tx_power` - The pin to use for measuring transmitted power.
    /// * `reflected_power` - The pin to use for measuring reflected power.
    pub fn new(tx_power: AdcPin, reflected_power: AdcPin) -> Self {
        Self {
            tx_power,
            reflected_power,
        }
    }
}

/// Represents all of the I2C devices on the bus for a single RF channel.
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
    /// * `delay` - A means of delaying during initialization.
    ///
    /// # Returns
    /// An option containing the devices if they were discovered on the bus. If any device did not
    /// properly enumerate, the option will be empty.
    fn new(manager: &'static BusManager, delay: &mut impl DelayUs<u8>) -> Option<Self> {
        // The ADS7924 and DAC7571 are present on the booster mainboard, so instantiation
        // and communication should never fail.
        let mut dac7571 = Dac7571::default(manager.acquire());

        // Ensure the bias DAC is placing the RF amplifier in pinch off (disabled).
        dac7571.set_voltage(3.2).expect("Bias DAC did not respond");

        // Verify we can communicate with the power monitor.
        let mut ads7924 =
            Ads7924::default(manager.acquire(), delay).expect("Power monitor did not enumerate");
        ads7924
            .get_voltage(ads7924::Channel::Three)
            .expect("Power monitor did not respond");

        // Configure alarm thresholds for the P5V0_MP signal.
        ads7924
            .set_thresholds(ads7924::Channel::Three, 0.0, 5.5 / 2.5)
            .expect("Power monitor failed to set thresholds");

        // Verify that there is no active alarm condition.
        assert!(ads7924.clear_alarm().expect("Failed to clear alarm") == 0);

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
pub struct ChannelPins {
    enable_power: hal::gpio::gpiod::PD<Output<PushPull>>,

    // The alert and input overdrive pins have external pull resistors, so we don't need to pull
    // them internally.
    pub alert: hal::gpio::gpiod::PD<Input<Floating>>,

    pub input_overdrive: hal::gpio::gpioe::PE<Input<Floating>>,

    // There are no pullup/pulldown resistors on this input, so we will pull it down internally.
    pub output_overdrive: hal::gpio::gpioe::PE<Input<PullDown>>,

    signal_on: hal::gpio::gpiog::PG<Output<PushPull>>,

    adc_pins: AnalogPins,
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
    /// * `adc_pins` - The AnalogPins that are associated with the channel.
    pub fn new(
        enable_power: hal::gpio::gpiod::PD<Output<PushPull>>,
        alert: hal::gpio::gpiod::PD<Input<Floating>>,
        input_overdrive: hal::gpio::gpioe::PE<Input<Floating>>,
        output_overdrive: hal::gpio::gpioe::PE<Input<PullDown>>,
        signal_on: hal::gpio::gpiog::PG<Output<PushPull>>,
        adc_pins: AnalogPins,
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
pub struct RfChannel {
    pub i2c_devices: Devices,
    pub pins: ChannelPins,
    output_interlock_threshold: f32,
    reflected_interlock_threshold: f32,
    bias_voltage: f32,
    input_power_transform: LinearTransformation,
    reflected_power_transform: LinearTransformation,
    output_power_transform: LinearTransformation,
    state: ChannelState,
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
    /// * `delay` - A means of delaying during setup.
    ///
    /// # Returns
    /// An option containing an RfChannel if a channel was discovered on the bus. None otherwise.
    pub fn new(
        manager: &'static BusManager,
        control_pins: ChannelPins,
        delay: &mut impl DelayUs<u8>,
    ) -> Option<Self> {
        // Attempt to instantiate the I2C devices on the channel.
        match Devices::new(manager, delay) {
            Some(devices) => {
                let mut channel = Self {
                    i2c_devices: devices,
                    pins: control_pins,
                    output_interlock_threshold: f32::NAN,
                    reflected_interlock_threshold: f32::NAN,
                    bias_voltage: -3.2,

                    // When operating at 100MHz, the power detectors specify the following output
                    // characteristics for -10 dBm to 10 dBm (the equation uses slightly different coefficients
                    // for different power levels and frequencies):
                    //
                    // dBm = V(Vout) / .035 V/dB - 35.6 dBm
                    //
                    // All of the power meters are preceded by attenuators which are incorporated in
                    // the offset.
                    output_power_transform: LinearTransformation::new(
                        1.0 / 0.035,
                        -35.6 + 19.8 + 10.0,
                    ),

                    // The input power and reflected power detectors are then passed through an
                    // op-amp with gain 1.5x - this modifies the slope from 35mV/dB to 52.5mV/dB
                    input_power_transform: LinearTransformation::new(1.5 / 0.035, -35.6 + 8.9),
                    reflected_power_transform: LinearTransformation::new(
                        1.5 / 0.035,
                        -35.6 + 19.8 + 10.0,
                    ),
                    state: ChannelState::Disabled,
                };

                channel.set_interlock_thresholds(0.0, 0.0).unwrap();
                channel.set_bias(-3.2).unwrap();

                // Configure alerts/alarms for the power monitor.

                // Ensure that P5V0MP remains within +/- 500mV of the specified voltage. Note that
                // the 5V rail is divided by 2.5 before entering the ADC.
                channel
                    .i2c_devices
                    .power_monitor
                    .set_thresholds(ads7924::Channel::Three, 0.0, 5.5 / 2.5)
                    .unwrap();

                // The P28V0 current rail has an equivalent equation of Isns = Vsns * 0.233.
                // Configure limits for 500mA range.
                channel
                    .i2c_devices
                    .power_monitor
                    .set_thresholds(
                        ads7924::Channel::Zero,
                        0.0,
                        0.500 * (100.0 / 0.100 / 4300.0),
                    )
                    .unwrap();

                // The P5V0 current rail has an equivalent equation of Isns = Vsns * 0.156.
                // Configure limits for 300mA of range.
                channel
                    .i2c_devices
                    .power_monitor
                    .set_thresholds(ads7924::Channel::One, 0.0, 0.300 * (100.0 / 0.100 / 6200.0))
                    .unwrap();

                channel.i2c_devices.power_monitor.clear_alarm().unwrap();

                Some(channel)
            }
            None => None,
        }
    }

    /// Set the interlock thresholds for the channel.
    ///
    /// # Args
    /// * `output_power` - The dBm interlock threshold to configure for the output power.
    /// * `reflected_power` - The dBm interlock threshold to configure for reflected power.
    pub fn set_interlock_thresholds(
        &mut self,
        output_power: f32,
        reflected_power: f32,
    ) -> Result<(), Error> {
        match self.i2c_devices.interlock_thresholds_dac.set_voltage(
            self.reflected_power_transform.invert(reflected_power),
            ad5627::Dac::A,
        ) {
            Err(ad5627::Error::Range) => return Err(Error::Bounds),
            Err(ad5627::Error::I2c(_)) => return Err(Error::Interface),
            Ok(voltage) => {
                self.reflected_interlock_threshold = self.reflected_power_transform.map(voltage);
            }
        }

        match self.i2c_devices.interlock_thresholds_dac.set_voltage(
            self.output_power_transform.invert(output_power),
            ad5627::Dac::B,
        ) {
            Err(ad5627::Error::Range) => return Err(Error::Bounds),
            Err(ad5627::Error::I2c(_)) => return Err(Error::Interface),
            Ok(voltage) => {
                self.output_interlock_threshold = self.output_power_transform.map(voltage);
            }
        }

        Ok(())
    }

    /// Update the current state of the RF channel.
    ///
    /// # Note
    /// This must be called periodically to facilitate enabling a channel.
    pub fn process_state(&mut self) -> Result<(), Error> {
        if let ChannelState::Enabling(start_time) = self.state {
            // TODO: Replace constant definition of CPU frequency here.
            if start_time.elapsed() > Duration::from_cycles(200 * (168_000_000 / 1000)) {
                self.finalize_enable()?;
            }
        }

        Ok(())
    }


    /// Check if the channel is indicating an interlock has tripped.
    pub fn is_overdriven(&self) -> bool {
        let input_overdrive = self.pins.input_overdrive.is_low().unwrap();
        let output_overdrive = self.pins.output_overdrive.is_low().unwrap();

        input_overdrive || output_overdrive
    }

    /// Check if the channel is enabled.
    pub fn is_enabled(&self) -> bool {
        let enabled =
            self.pins.enable_power.is_high().unwrap() && self.pins.signal_on.is_high().unwrap();

        // Check that the bias is out of pinch off. We're using a somewhat arbitrary value here as
        // the nominal threshold voltage is -1.6V, but the disabled channel should always be set to
        // -3.2 V.
        let bias_enabled = self.bias_voltage > -3.0;

        enabled && !self.is_overdriven() && bias_enabled
    }

    /// Check if the channel is indicating an alarm.
    pub fn is_alarmed(&self) -> bool {
        self.pins.alert.is_low().unwrap()
    }

    /// Start the enable process for channel and power it up.
    pub fn enable(&mut self) -> Result<(), Error> {
        // TODO: Power-up the channel.
        self.i2c_devices
            .bias_dac
            .set_voltage(3.2)
            .expect("Failed to disable RF bias voltage");
        self.pins.enable_power.set_high().unwrap();

        // We have just started the supply sequencer for the RF channel power rail. This may take
        // some time. We can't set the bias DAC until those supplies have stabilized.
        self.state  = ChannelState::Enabling(Instant::now());

        Ok(())
    }

    /// Finalize the enable process once all RF channel supplies have enabled.
    fn finalize_enable(&mut self) -> Result<(), Error> {
        self.i2c_devices
            .bias_dac
            .set_voltage(-1.0 * self.bias_voltage)
            .expect("Failed to configure RF bias voltage");

        self.pins.signal_on.set_high().unwrap();

        self.state = ChannelState::Active;

        Ok(())
    }

    /// Disable the channel and power it off.
    pub fn disable(&mut self) -> Result<(), Error> {
        self.pins.signal_on.set_low().unwrap();

        // Set the bias DAC output into pinch-off.
        self.i2c_devices
            .bias_dac
            .set_voltage(3.2)
            .expect("Failed to disable RF bias voltage");

        self.pins.enable_power.set_low().unwrap();

        self.state = ChannelState::Disabled;

        Ok(())
    }

    /// Get the temperature of the channel in celsius.
    pub fn get_temperature(&mut self) -> f32 {
        self.i2c_devices
            .temperature_monitor
            .get_remote_temperature()
            .unwrap()
    }

    /// Set the bias of the channel.
    ///
    /// # Args
    /// * `bias_voltage` - The desired bias voltage on the RF amplification transitor.
    pub fn set_bias(&mut self, bias_voltage: f32) -> Result<(), Error> {
        // The bias voltage is the inverse of the DAC output voltage.
        let bias_voltage = -1.0 * bias_voltage;

        match self.i2c_devices.bias_dac.set_voltage(bias_voltage) {
            Err(dac7571::Error::Bounds) => return Err(Error::Bounds),
            Err(_) => panic!("Failed to set DAC bias voltage"),
            Ok(voltage) => {
                self.bias_voltage = -voltage;
            }
        };

        Ok(())
    }

    /// Get current power supply measurements from the channel.
    ///
    /// # Returns
    /// The most recent power supply measurements of the channel.
    pub fn get_supply_measurements(&mut self) -> SupplyMeasurements {
        // The P5V0 rail goes through a resistor divider of 15K -> 10K. This corresponds with a 2.5x
        // reduction in measured voltage.
        let p5v_voltage = self
            .i2c_devices
            .power_monitor
            .get_voltage(ads7924::Channel::Three)
            .unwrap();
        let v_p5v0mp = p5v_voltage * 2.5;

        // The 28V current is sensed across a 100mOhm resistor with 100 Ohm input resistance. The
        // output resistance on the current sensor is 4.3K Ohm.
        //
        // From the LT6106 (current monitor) datasheet:
        // Vout = Vsns * Rout / Rin
        //
        // Given:
        // Vsns = Isns * Rsns
        // Rsns = 100m Ohm
        // Rin = 100 Ohm
        // Rout = 6.2K Ohm
        //
        // Vout = Isns * Rsns * Rout / Rin
        // Isns = (Vout * Rin) / Rsns / Rout
        let p28v_rail_current_sense = self
            .i2c_devices
            .power_monitor
            .get_voltage(ads7924::Channel::Zero)
            .unwrap();
        let i_p28v0ch = p28v_rail_current_sense * (100.0 / 0.100 / 4300.0);

        // P5V rail uses an Rout of 6.2K with identical other characteristics.
        let p5v_rail_current_sense = self
            .i2c_devices
            .power_monitor
            .get_voltage(ads7924::Channel::One)
            .unwrap();
        let i_p5v0ch = p5v_rail_current_sense * (100.0 / 0.100 / 6200.0);

        SupplyMeasurements {
            v_p5v0mp,
            i_p28v0ch,
            i_p5v0ch,
        }
    }

    /// Get the current input power measurement.
    ///
    /// # Returns
    /// The input power in dBm.
    pub fn get_input_power(&mut self) -> f32 {
        let voltage = self.i2c_devices.input_power_adc.get_voltage().unwrap();

        self.input_power_transform.map(voltage)
    }

    /// Get the current reflected power measurement.
    ///
    /// # Args
    /// * `adc` - The ADC to use for performing the measurement.
    ///
    /// # Returns
    /// The reflected power in dBm.
    pub fn get_reflected_power(&mut self, mut adc: &mut hal::adc::Adc<hal::stm32::ADC3>) -> f32 {
        let sample = self
            .pins
            .adc_pins
            .reflected_power
            .convert(&mut adc, SampleTime::Cycles_480);
        let voltage = adc.sample_to_millivolts(sample) as f32 / 1000.0;

        self.reflected_power_transform.map(voltage)
    }

    /// Get the current output power measurement.
    ///
    /// # Args
    /// * `adc` - The ADC to use for performing the measurement.
    ///
    /// # Returns
    /// The output power in dBm.
    pub fn get_output_power(&mut self, mut adc: &mut hal::adc::Adc<hal::stm32::ADC3>) -> f32 {
        let sample = self
            .pins
            .adc_pins
            .tx_power
            .convert(&mut adc, SampleTime::Cycles_480);
        let voltage = adc.sample_to_millivolts(sample) as f32 / 1000.0;

        self.output_power_transform.map(voltage)
    }

    /// Get the current output power interlock threshold.
    ///
    /// # Returns
    /// The current output interlock threshold in dBm.
    pub fn get_output_interlock_threshold(&self) -> f32 {
        self.output_interlock_threshold
    }

    /// Get the current reflected power interlock threshold.
    ///
    /// # Returns
    /// The current reflected interlock threshold in dBm.
    pub fn get_reflected_interlock_threshold(&self) -> f32 {
        self.output_interlock_threshold
    }

    /// Get the current bias voltage programmed to the RF amplification transistor.
    pub fn get_bias_voltage(&mut self) -> f32 {
        self.bias_voltage
    }
}
