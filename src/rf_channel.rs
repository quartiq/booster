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

use super::{I2cBusManager, I2cProxy};
use crate::{settings::BoosterChannelSettings, Error};
use embedded_hal::blocking::delay::DelayUs;
use stm32f4xx_hal::{
    self as hal,
    adc::config::SampleTime,
    gpio::{Analog, Floating, Input, Output, PullDown, PushPull},
    prelude::*,
};

use rtic::cyccnt::{Duration, Instant};

/// A structure representing power supply measurements of a channel.
pub struct SupplyMeasurements {
    pub v_p5v0mp: f32,
    pub i_p5v0ch: f32,
    pub i_p28v0ch: f32,
}

/// Represents the possible channel fault conditions.
#[derive(Debug, Copy, Clone, serde::Serialize)]
pub enum ChannelFault {
    OverTemperature,
    UnderTemperature,
    OverCurrent,
}

/// Represents the three power interlocks present on the device.
#[derive(Debug, Copy, Clone, serde::Serialize)]
pub enum Interlock {
    Input,
    Output,
    Reflected,
}

/// The current state of an RF channel.
#[derive(Debug, Copy, Clone)]
pub enum ChannelState {
    /// The channel has been blocked due to a fatal error condition.
    Blocked(ChannelFault),

    /// The channel output is disabled.
    Disabled,

    /// The channel is in the enabling process. The two parameters are the instant that power-up
    /// began and whether or not the output should be enabled once power-up is complete.
    Powerup(Instant, bool),

    /// The channel is powered up, but outputs are not enabled.
    Powered,

    /// The channel is actively outputting.
    Enabled,

    /// The channel has tripped an interlock threshold. Outputs are disabled.
    Tripped(Interlock),

    /// The channel is in the process of shutting down.
    Powerdown(Instant),

    /// The channel should begin powerup and enabling as soon as possible.
    WillPowerupEnable,
}

impl serde::Serialize for ChannelState {
    fn serialize<S: serde::Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        match *self {
            ChannelState::Blocked(_) => {
                serializer.serialize_unit_variant("ChannelState", 0, "Blocked")
            }
            ChannelState::Disabled => {
                serializer.serialize_unit_variant("ChannelState", 1, "Disabled")
            }
            ChannelState::Powerup(_, _) => {
                serializer.serialize_unit_variant("ChannelState", 2, "Powerup")
            }
            ChannelState::Powered => {
                serializer.serialize_unit_variant("ChannelState", 3, "Powered")
            }
            ChannelState::Enabled => {
                serializer.serialize_unit_variant("ChannelState", 4, "Enabled")
            }
            ChannelState::Tripped(Interlock::Input) => {
                serializer.serialize_unit_variant("ChannelState", 5, "Tripped(Input)")
            }
            ChannelState::Tripped(Interlock::Output) => {
                serializer.serialize_unit_variant("ChannelState", 5, "Tripped(Output)")
            }
            ChannelState::Tripped(Interlock::Reflected) => {
                serializer.serialize_unit_variant("ChannelState", 5, "Tripped(Reflected)")
            }
            ChannelState::Powerdown(_) => {
                serializer.serialize_unit_variant("ChannelState", 6, "Powerdown")
            }
            ChannelState::WillPowerupEnable => {
                serializer.serialize_unit_variant("ChannelState", 7, "WillPowerupEnable")
            }
        }
    }
}

struct StateMachine {
    state: ChannelState,
}

impl StateMachine {
    /// Construct a new state machine.
    pub fn new(state: ChannelState) -> StateMachine {
        Self { state }
    }

    /// Get the current state of the state machine.
    pub fn state(&self) -> ChannelState {
        self.state
    }

    /// Transition the state machine to a new state.
    pub fn transition(&mut self, new_state: ChannelState) -> Result<(), Error> {
        // Explicitly always allow the state machine to transition to the blocked state.
        if let ChannelState::Blocked(fault) = new_state {
            match self.state {
                ChannelState::Blocked(_) => {}
                _ => self.state = ChannelState::Blocked(fault),
            }
            return Ok(());
        }

        let new_state = match self.state {
            // It is never valid to transition out of the blocked state.
            ChannelState::Blocked(_) => return Err(Error::InvalidState),

            // It is only valid to transition from disabled into the power-up state.
            ChannelState::Disabled => match new_state {
                ChannelState::Disabled
                | ChannelState::Powerup(_, _)
                | ChannelState::WillPowerupEnable => new_state,
                _ => return Err(Error::InvalidState),
            },

            // During power up, it is only possible to transition to powered, enabled, or
            // power-down.
            ChannelState::Powerup(_, _) => match new_state {
                ChannelState::Enabled | ChannelState::Powered | ChannelState::Powerdown(_) => {
                    new_state
                }
                _ => return Err(Error::InvalidState),
            },

            // When powered, it is only valid to enter enabled or power-down.
            ChannelState::Powered => match new_state {
                ChannelState::Powered | ChannelState::Enabled | ChannelState::Powerdown(_) => {
                    new_state
                }
                _ => return Err(Error::InvalidState),
            },

            // When enabled, it is only possible to transition to powered, powerdown, or tripped.
            ChannelState::Enabled => match new_state {
                ChannelState::Enabled
                | ChannelState::Powered
                | ChannelState::Tripped(_)
                | ChannelState::Powerdown(_) => new_state,
                _ => return Err(Error::InvalidState),
            },

            // When powering down, it is only possible to finish the power-down process.
            ChannelState::Powerdown(_) => match new_state {
                ChannelState::Disabled => new_state,
                _ => return Err(Error::InvalidState),
            },

            // When in the WillPowerUpEnable state, it is only valid to enter the `PowerUp` state
            // with the enable flag asserted.
            ChannelState::WillPowerupEnable => match new_state {
                ChannelState::Powerup(_, true) => new_state,
                _ => return Err(Error::InvalidState),
            },

            // When tripped, it is only possible to re-enable, enter powered mode, or enter
            // power-down.  Note that semantically, `Powered` and `Tripped` are identical states -
            // the channel is powered, but RF output is disabled.
            ChannelState::Tripped(_) => match new_state {
                ChannelState::Powerdown(_) | ChannelState::Powered | ChannelState::Enabled => {
                    new_state
                }
                _ => return Err(Error::InvalidState),
            },
        };

        self.state = new_state;
        Ok(())
    }
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
    interlock_thresholds_dac: Ad5627<I2cProxy>,
    input_power_adc: Mcp3221<I2cProxy>,
    temperature_monitor: Max6642<I2cProxy>,
    bias_dac: Dac7571<I2cProxy>,
    power_monitor: Ads7924<I2cProxy>,
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
    /// properly enumerate, the option will be empty. The returend tuple will be (devices, eeprom).
    fn new(
        manager: &'static I2cBusManager,
        delay: &mut impl DelayUs<u8>,
    ) -> Option<(Self, Microchip24AA02E48<I2cProxy>)> {
        // The ADS7924 and DAC7571 are present on the booster mainboard, so instantiation
        // and communication should never fail.
        let mut dac7571 = Dac7571::default(manager.acquire_i2c());

        // Ensure the bias DAC is placing the RF amplifier in pinch off (disabled).
        dac7571.set_voltage(3.2).expect("Bias DAC did not respond");

        // Verify we can communicate with the power monitor.
        let mut ads7924 = Ads7924::default(manager.acquire_i2c(), delay)
            .expect("Power monitor did not enumerate");
        ads7924
            .get_voltage(ads7924::Channel::Three)
            .expect("Power monitor did not respond");

        // Configure alarm thresholds for the P5V0_MP signal.
        ads7924
            .set_thresholds(ads7924::Channel::Three, 0.0, 5.5 / 2.5)
            .expect("Power monitor failed to set thresholds");

        // Verify that there is no active alarm condition.
        assert!(ads7924.clear_alarm().expect("Failed to clear alarm") == 0);

        if let Ok(ad5627) = Ad5627::default(manager.acquire_i2c()) {
            if let Ok(eui48) = Microchip24AA02E48::new(manager.acquire_i2c()) {
                // Query devices on the RF module to verify they are present.
                let mut max6642 = Max6642::att94(manager.acquire_i2c());
                if let Err(_) = max6642.get_remote_temperature() {
                    return None;
                }

                let mut mcp3221 = Mcp3221::default(manager.acquire_i2c());
                if let Err(_) = mcp3221.get_voltage() {
                    return None;
                }

                return Some((
                    Self {
                        interlock_thresholds_dac: ad5627,
                        input_power_adc: mcp3221,
                        temperature_monitor: max6642,
                        bias_dac: dac7571,
                        power_monitor: ads7924,
                    },
                    eui48,
                ));
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

    pub reflected_overdrive: hal::gpio::gpioe::PE<Input<Floating>>,

    // There are no pullup/pulldown resistors on this input, so we will pull it down internally.
    pub output_overdrive: hal::gpio::gpioe::PE<Input<PullDown>>,

    signal_on: hal::gpio::gpiog::PG<Output<PushPull>>,

    adc_pins: AnalogPins,
}

impl ChannelPins {
    /// Construct new set of control pins.
    ///
    /// # Args
    /// * `enable_power` - An output pin used to power up the RF channel.
    /// * `alert` - An input pin monitoring the status of the RF power module. This is connected to
    ///   the ADS7924 alert output.
    /// * `reflected_overdrive` - An input pin that indicates an input overdrive.
    /// * `output_overdrive` - An input pin that indicates an output overdrive.
    /// * `signal_on` - An output pin that is set high to enable output signal amplification.
    /// * `adc_pins` - The AnalogPins that are associated with the channel.
    pub fn new(
        enable_power: hal::gpio::gpiod::PD<Output<PushPull>>,
        alert: hal::gpio::gpiod::PD<Input<Floating>>,
        reflected_overdrive: hal::gpio::gpioe::PE<Input<Floating>>,
        output_overdrive: hal::gpio::gpioe::PE<Input<PullDown>>,
        signal_on: hal::gpio::gpiog::PG<Output<PushPull>>,
        adc_pins: AnalogPins,
    ) -> Self {
        let mut pins = Self {
            enable_power,
            alert,
            reflected_overdrive,
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
    settings: BoosterChannelSettings,
    state_machine: StateMachine,
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
        manager: &'static I2cBusManager,
        control_pins: ChannelPins,
        delay: &mut impl DelayUs<u8>,
    ) -> Option<Self> {
        // Attempt to instantiate the I2C devices on the channel.
        match Devices::new(manager, delay) {
            Some((devices, eeprom)) => {
                let mut channel = Self {
                    i2c_devices: devices,
                    pins: control_pins,
                    settings: BoosterChannelSettings::new(eeprom),
                    state_machine: StateMachine::new(ChannelState::Disabled),
                };

                channel
                    .set_interlock_thresholds(
                        channel.settings.data.output_interlock_threshold,
                        channel.settings.data.reflected_interlock_threshold,
                    )
                    .unwrap();

                // Place the bias DAC to drive the RF amplifier into pinch-off.
                channel
                    .i2c_devices
                    .bias_dac
                    .set_voltage(3.2)
                    .expect("Failed to disable RF bias voltage");

                // If the channel configuration specifies the channel as enabled, power up the
                // channel now.
                if channel.settings.data.enabled {
                    channel
                        .state_machine
                        .transition(ChannelState::WillPowerupEnable)
                        .unwrap();
                }

                // Configure alerts/alarms for the power monitor.

                // Ensure that P5V0MP remains within +/- 500mV of the specified voltage. Note that
                // the 5V rail is divided by 2.5 before entering the ADC.
                channel
                    .i2c_devices
                    .power_monitor
                    .set_thresholds(ads7924::Channel::Three, 0.0, 5.5 / 2.5)
                    .unwrap();

                channel.i2c_devices.power_monitor.clear_alarm().unwrap();

                Some(channel)
            }
            None => None,
        }
    }

    /// Save the current channel configuration.
    pub fn save_configuration(&mut self) {
        self.settings.save();
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
            self.settings
                .data
                .reflected_power_transform
                .invert(reflected_power),
            ad5627::Dac::A,
        ) {
            Err(ad5627::Error::Range) => return Err(Error::Bounds),
            Err(ad5627::Error::I2c(_)) => return Err(Error::Interface),
            Ok(voltage) => {
                self.settings.data.reflected_interlock_threshold =
                    self.settings.data.reflected_power_transform.map(voltage);
            }
        }

        match self.i2c_devices.interlock_thresholds_dac.set_voltage(
            self.settings
                .data
                .output_power_transform
                .invert(output_power),
            ad5627::Dac::B,
        ) {
            Err(ad5627::Error::Range) => return Err(Error::Bounds),
            Err(ad5627::Error::I2c(_)) => return Err(Error::Interface),
            Ok(voltage) => {
                self.settings.data.output_interlock_threshold =
                    self.settings.data.output_power_transform.map(voltage);
            }
        }

        Ok(())
    }

    fn check_faults(&mut self) -> Option<ChannelFault> {
        let temperature = self.get_temperature();
        if temperature > 60.0 {
            Some(ChannelFault::OverTemperature)
        } else if temperature < 5.0 {
            Some(ChannelFault::UnderTemperature)
        } else if self.pins.alert.is_low().unwrap() {
            Some(ChannelFault::OverCurrent)
        } else {
            None
        }
    }

    /// Update the current state of the RF channel.
    ///
    /// # Note
    /// This must be called periodically to facilitate enabling a channel.
    pub fn update(&mut self) -> Result<(), Error> {
        // Check potential fault conditions.
        if let Some(fault) = self.check_faults() {
            // Latch the fault condition.
            self.state_machine
                .transition(ChannelState::Blocked(fault))
                .unwrap();

            // Power off the channel if it was powered on.
            if self.pins.enable_power.is_high().unwrap() {
                self.start_disable();
            }
        }

        match self.state_machine.state() {
            ChannelState::Powerup(start_time, should_enable) => {
                // The LM3880 requires 180ms to power up all supplies on the channel. We add an
                // additional 20ms margin.

                // TODO: Replace constant definition of CPU frequency here.
                if start_time.elapsed() > Duration::from_cycles(200 * (168_000_000 / 1000)) {
                    if should_enable {
                        self.enable_output()?;
                    } else {
                        self.state_machine.transition(ChannelState::Powered)?;
                    }
                }
            }

            ChannelState::Powerdown(start_time) => {
                // Note that we use 500ms here due to the worst-case power-sequencing operation of
                // the LM3880 that occurs when a channel is disabled immediately after enable. In
                // this case, the LM3880 will require 180ms to power up the channel, 120ms to
                // stabilize, and then 180ms to power down the channel.

                // TODO: Replace constant definition of CPU frequency here.
                if start_time.elapsed() > Duration::from_cycles(500 * (168_000_000 / 1000)) {
                    self.state_machine
                        .transition(ChannelState::Disabled)
                        .unwrap();
                }
            }

            ChannelState::Enabled => {
                // We explicitly only check for overdrive conditions once the channel has been
                // fully enabled.
                if let Some(trip_source) = self.get_overdrive_source() {
                    // Manually disable the ON_OFFn net - this may be a software-initiated
                    // interlock
                    self.pins.signal_on.set_low().unwrap();
                    self.state_machine
                        .transition(ChannelState::Tripped(trip_source))
                        .unwrap();
                }
            }

            // Begin the power-up and enabling process immediately.
            ChannelState::WillPowerupEnable => self.start_powerup(true).unwrap(),

            // There's no active transitions in the following states.
            ChannelState::Disabled
            | ChannelState::Blocked(_)
            | ChannelState::Tripped(_)
            | ChannelState::Powered => {}
        }

        Ok(())
    }

    fn get_overdrive_source(&mut self) -> Option<Interlock> {
        let reflected_overdrive = self.pins.reflected_overdrive.is_high().unwrap();
        let output_overdrive = self.pins.output_overdrive.is_high().unwrap();

        // The schematic indicates the maximum input power is 25dBm. We'll use 20dBm to provide
        // a safety margin.
        let input_overdrive = self.get_input_power() > 20.0;

        if input_overdrive {
            Some(Interlock::Input)
        } else if output_overdrive {
            Some(Interlock::Output)
        } else if reflected_overdrive {
            Some(Interlock::Reflected)
        } else {
            None
        }
    }

    /// Start the power-up process for channel.
    ///
    /// # Args
    /// * `should_enable` - Specified true if the channel output should be enabled when power-up
    ///   completes.
    pub fn start_powerup(&mut self, should_enable: bool) -> Result<(), Error> {
        // If we are just tripped or are already powered, we can re-enable the channel by cycling
        // the ON_OFFn input.
        match self.state_machine.state() {
            ChannelState::Tripped(_) | ChannelState::Powered => {
                if should_enable {
                    return self.enable_output();
                } else {
                    return self.state_machine.transition(ChannelState::Powered);
                }
            }

            // If the channel is already enabled, there's nothing to do.
            ChannelState::Enabled => return Ok(()),
            _ => {}
        }

        // We will be starting the supply sequencer for the RF channel power rail. This will take
        // some time. We can't set the bias DAC until those supplies have stabilized.
        self.state_machine
            .transition(ChannelState::Powerup(Instant::now(), should_enable))?;

        // Place the bias DAC to drive the RF amplifier into pinch-off during the power-up process.
        self.i2c_devices
            .bias_dac
            .set_voltage(3.2)
            .expect("Failed to disable RF bias voltage");

        // Ensure that the RF output is disabled during the power-up process.
        self.pins.signal_on.set_low().unwrap();

        // Start the LM3880 power supply sequencer.
        self.pins.enable_power.set_high().unwrap();
        Ok(())
    }

    /// Enable RF input/output signals on the channel.
    ///
    /// # Note
    /// This can only be completed once the channel has been fully powered.
    fn enable_output(&mut self) -> Result<(), Error> {
        // It is only valid to enable the output if the channel is powered.
        if self.pins.enable_power.is_low().unwrap() {
            return Err(Error::InvalidState);
        }

        // It is not valid to enable the channel while the interlock thresholds are low. Due to
        // hardware configurations, it is possible that this would result in a condition where the
        // interlock is never tripped even though the output is exceeding the interlock threshold.
        // As a workaround, we need to ensure that the interlock level is above the output power
        // detector level. When RF is disabled, the power detectors output a near-zero value, so
        // 100mV should be a sufficient level.
        if (self.settings.data.reflected_interlock_threshold
            < self.settings.data.reflected_power_transform.map(0.100))
            || (self.settings.data.output_interlock_threshold
                < self.settings.data.output_power_transform.map(0.100))
        {
            self.start_disable();
            return Err(Error::Invalid);
        }

        self.i2c_devices
            .bias_dac
            .set_voltage(-1.0 * self.settings.data.bias_voltage)
            .expect("Failed to configure RF bias voltage");

        self.pins.signal_on.set_high().unwrap();

        self.settings.data.enabled = true;

        self.state_machine
            .transition(ChannelState::Enabled)
            .expect("Invalid state transition");

        Ok(())
    }

    /// Disable the channel and power it off.
    pub fn start_disable(&mut self) {
        let channel_was_powered = self.pins.enable_power.is_high().unwrap();

        self.settings.data.enabled = false;

        // The RF channel may be unconditionally disabled at any point to aid in preventing damage.
        // The effect of this is that we must assume worst-case power-down timing, which increases
        // the time until we can enable a channel after a power-down.
        self.pins.signal_on.set_low().unwrap();

        // Set the bias DAC output into pinch-off.
        self.i2c_devices
            .bias_dac
            .set_voltage(3.2)
            .expect("Failed to disable RF bias voltage");

        self.pins.enable_power.set_low().unwrap();

        if channel_was_powered {
            // Only update the channel state if the channel has not latched an error.
            match self.state_machine.state() {
                ChannelState::Blocked(_) => {}
                _ => self
                    .state_machine
                    .transition(ChannelState::Powerdown(Instant::now()))
                    .expect("Failed to enter power-down"),
            }
        }
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
                self.settings.data.bias_voltage = -voltage;
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

        let i_p28v0ch = self.measure_p28v_current(false);

        // The P5V current is sensed across a 100mOhm resistor with 100 Ohm input resistance. The
        // output resistance on the current sensor is 6.2K Ohm.
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

    fn measure_p28v_current(&mut self, force_update: bool) -> f32 {
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
        // Rout = 4.3K Ohm
        //
        // Vout = Isns * Rsns * Rout / Rin
        // Isns = (Vout * Rin) / Rsns / Rout

        let p28v_rail_current_sense = if force_update {
            // Force the power monitor to make a new reading.
            self.i2c_devices
                .power_monitor
                .measure_voltage(ads7924::Channel::Zero)
                .unwrap()
        } else {
            // Read the cached (scanned) ADC measurement from the monitor.
            self.i2c_devices
                .power_monitor
                .get_voltage(ads7924::Channel::Zero)
                .unwrap()
        };

        p28v_rail_current_sense * (100.0 / 0.100 / 4300.0)
    }

    /// Get the current input power measurement.
    ///
    /// # Returns
    /// The input power in dBm.
    pub fn get_input_power(&mut self) -> f32 {
        let voltage = self.i2c_devices.input_power_adc.get_voltage().unwrap();

        self.settings.data.input_power_transform.map(voltage)
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

        self.settings.data.reflected_power_transform.map(voltage)
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

        self.settings.data.output_power_transform.map(voltage)
    }

    /// Get the current output power interlock threshold.
    ///
    /// # Returns
    /// The current output interlock threshold in dBm.
    pub fn get_output_interlock_threshold(&self) -> f32 {
        self.settings.data.output_interlock_threshold
    }

    /// Get the current reflected power interlock threshold.
    ///
    /// # Returns
    /// The current reflected interlock threshold in dBm.
    pub fn get_reflected_interlock_threshold(&self) -> f32 {
        self.settings.data.reflected_interlock_threshold
    }

    /// Get the current bias voltage programmed to the RF amplification transistor.
    pub fn get_bias_voltage(&self) -> f32 {
        self.settings.data.bias_voltage
    }

    /// Get the current state of the channel.
    pub fn get_state(&self) -> ChannelState {
        self.state_machine.state()
    }

    /// Tune the RF amplifier bias current.
    ///
    /// # Args
    /// * `desired_current` - The desired RF amplifier drain current.
    ///
    /// # Returns
    /// A tuple of (Vgs, Ids) where Vgs is the bias voltage on the amplifier gate. Ids is the actual
    /// drain current achieved.
    pub fn tune_bias(&mut self, desired_current: f32) -> Result<(f32, f32), Error> {
        // Verify that the channel is powered, but is not actively outputting for bias calibration.
        match self.state_machine.state() {
            ChannelState::Powered | ChannelState::Tripped(_) => {}
            _ => return Err(Error::InvalidState),
        }

        // Booster schematic indicates the regulator is configured to supply up to 550mA. However,
        // when the RF input is disabled, the bias current is significantly lower. For this reason,
        // the upper bound is currently limited to 100mA, but may be adjusted in the future.
        if desired_current < 0.010 || desired_current > 0.100 {
            return Err(Error::Bounds);
        }

        // Disable the RF input during the test. Note: The state should ensure this is already the
        // case, but it is done here redundantly as a safety precaution.
        self.pins.signal_on.set_low().unwrap();

        // Place the RF channel into pinch-off to start.
        let mut bias_voltage = -3.2;
        self.set_bias(bias_voltage).unwrap();
        let mut last_current = self.measure_p28v_current(true);

        // First, increase the bias voltage until we overshoot the desired set current by a small
        // margin.
        while last_current < desired_current {
            // Slowly bring the bias voltage up in steps of 20mV until the desired drain current is
            // achieved.
            bias_voltage = bias_voltage + 0.020;
            if bias_voltage >= 0.0 {
                return Err(Error::Invalid);
            }

            self.set_bias(bias_voltage).unwrap();

            // Re-measure the drain current.
            let new_current = self.measure_p28v_current(true);

            // Check that the LDO did not enter fold-back. When the LDO enters fold-back, the
            // current should begin to drop dramatically. For now, a 20mA threshold is used to
            // detect foldback.
            if last_current - new_current > 0.020 {
                return Err(Error::Foldback);
            }
            last_current = new_current;
        }

        // Next, decrease the bias voltage until we drop back below the desired set point.
        while last_current > desired_current {
            // Decrease the bias voltage in 1mV steps to decrease the drain current marginally.
            bias_voltage = bias_voltage - 0.001;
            if bias_voltage <= -3.2 {
                return Err(Error::Invalid);
            }

            // Set the new bias and re-measure the drain current.
            self.set_bias(bias_voltage).unwrap();
            last_current = self.measure_p28v_current(true);
        }

        // Note that we're returning the actual bias voltage as calculated by the DAC as opposed to
        // the voltage we used in the algorithm because the voltage reported by the DAC is more
        // accurate.
        Ok((self.settings.data.bias_voltage, last_current))
    }
}
