//! Definitions for Booster RF management channels.

use ad5627::{self, Ad5627};
use ads7924::Ads7924;
use dac7571::Dac7571;
use embedded_hal_bus::i2c;
use max6642::Max6642;
use mcp3221::Mcp3221;
use microchip_24aa02e48::Microchip24AA02E48;
use minimq::embedded_time::{duration::Extensions, Clock, Instant};

use super::{delay::AsmDelay, platform, I2cBusManager, I2cProxy, SystemTimer};
use crate::{
    settings::eeprom::rf_channel::{BoosterChannelSettings, ChannelSettings, ChannelState},
    Error,
};
use stm32f4xx_hal::{
    self as hal,
    adc::config::SampleTime,
    gpio::{Analog, Input, Output},
    hal::delay::DelayNs,
};

/// A structure representing power supply measurements of a channel.
struct SupplyMeasurements {
    v_p5v0mp: f32,
    i_p5v0ch: f32,
    i_p28v0ch: f32,
}

/// Represents the possible channel fault conditions.
#[derive(Debug, Copy, Clone, serde::Serialize)]
pub enum ChannelFault {
    OverTemperature,
    UnderTemperature,
    SupplyAlert,
}

/// Represents the three power interlocks present on the device.
#[derive(Debug, Copy, Clone, serde::Serialize)]
pub enum Interlock {
    Input,
    Output,
    Reflected,
}

/// A succinct representation of RF channel state for front panel status indication.
/// The three flags match the three LED states.
#[derive(Default, Copy, Clone, Debug)]
pub struct PowerStatus {
    /// The RF channel is powered on. Green LED.
    pub powered: bool,

    /// The RF output switch is disabled. Yellow LED
    pub rf_disabled: bool,

    /// The channel is in a force-disabled state due to a latched error. Red LED.
    pub blocked: bool,
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
//     pub fn convert(&self, adc: &mut hal::adc::Adc<hal::pac::ADC3>, sample_time: SampleTime) -> u16 {
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
        pub fn convert(&self, adc: &mut hal::adc::Adc<hal::pac::ADC3>, sample_time: SampleTime) -> u16 {
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
        delay: &mut AsmDelay,
    ) -> Option<(Self, Microchip24AA02E48<I2cProxy>)> {
        // The ADS7924 and DAC7571 are present on the booster mainboard, so instantiation
        // and communication should never fail.
        let mut dac7571 = Dac7571::default(i2c::AtomicDevice::new(manager));

        // Ensure the bias DAC is placing the RF amplifier in pinch off (disabled).
        dac7571
            .set_voltage(platform::BIAS_DAC_VCC)
            .expect("Bias DAC did not respond");

        // Verify we can communicate with the power monitor.
        let mut ads7924 = Ads7924::default(i2c::AtomicDevice::new(manager), delay)
            .expect("Power monitor did not enumerate");
        ads7924
            .get_voltage(ads7924::Channel::Three)
            .expect("Power monitor did not respond");

        // Note: Due to hardware limitations, the ADS7924 ALERT output is not used. Refer to
        // https://github.com/quartiq/booster/issues/130 for more information.

        // Verify that there is no active alarm condition.
        assert!(ads7924.clear_alarm().expect("Failed to clear alarm") == 0);

        // Query devices on the RF module to verify they are present.
        let ad5627 = Ad5627::default(i2c::AtomicDevice::new(manager)).ok()?;
        let eui48 = Microchip24AA02E48::new(i2c::AtomicDevice::new(manager)).ok()?;
        let mut max6642 = Max6642::att94(i2c::AtomicDevice::new(manager));
        max6642.get_remote_temperature().ok()?;
        let mut mcp3221 = Mcp3221::default(i2c::AtomicDevice::new(manager));
        mcp3221.get_voltage().ok()?;

        Some((
            Self {
                interlock_thresholds_dac: ad5627,
                input_power_adc: mcp3221,
                temperature_monitor: max6642,
                bias_dac: dac7571,
                power_monitor: ads7924,
            },
            eui48,
        ))
    }
}

/// Represents the control and status pins for an RF channel.
pub struct ChannelPins {
    enable_power: hal::gpio::EPin<Output>,

    // The alert and input overdrive pins have external pull resistors, so we don't need to pull
    // them internally.
    alert: hal::gpio::EPin<Input>,

    reflected_overdrive: hal::gpio::EPin<Input>,

    // There are no pullup/pulldown resistors on this input, so we will pull it down internally.
    output_overdrive: hal::gpio::EPin<Input>,

    signal_on: hal::gpio::EPin<Output>,

    output_power: AdcPin,
    reflected_power: AdcPin,
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
    /// * `output_power` - The pin to use for measuring transmitted power.
    /// * `reflected_power` - The pin to use for measuring reflected power.
    pub fn new(
        enable_power: hal::gpio::EPin<Output>,
        alert: hal::gpio::EPin<Input>,
        reflected_overdrive: hal::gpio::EPin<Input>,
        output_overdrive: hal::gpio::EPin<Input>,
        signal_on: hal::gpio::EPin<Output>,
        output_power: AdcPin,
        reflected_power: AdcPin,
    ) -> Self {
        let mut pins = Self {
            enable_power,
            alert,
            reflected_overdrive,
            output_overdrive,
            signal_on,
            output_power,
            reflected_power,
        };

        // Power down channel.
        pins.signal_on.set_low();
        pins.enable_power.set_low();
        pins
    }
}

/// Contains channel status information in SI base units.
#[derive(serde::Serialize)]
pub struct ChannelStatus {
    reflected_overdrive: bool,
    output_overdrive: bool,
    alert: bool,
    temperature: f32,
    p28v_current: f32,
    p5v_current: f32,
    p5v_voltage: f32,
    input_power: f32,
    reflected_power: f32,
    output_power: f32,
    state: sm::States,
}

/// Represents a means of interacting with an RF output channel.
pub struct RfChannel {
    devices: Devices,
    pins: ChannelPins,
    settings: BoosterChannelSettings,
    clock: SystemTimer,
    delay: AsmDelay,
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
        pins: ChannelPins,
        clock: SystemTimer,
        mut delay: AsmDelay,
    ) -> Option<Self> {
        // Attempt to instantiate the I2C devices on the channel.
        Devices::new(manager, &mut delay).map(|(devices, eeprom)| {
            let mut channel = Self {
                devices,
                pins,
                settings: BoosterChannelSettings::new(eeprom),
                clock,
                delay,
            };

            channel.apply_output_interlock_threshold().unwrap();

            // The reflected power interlock threshold is always configured to 30 dBm (1W
            // reflected power) to protect Booster hardware.
            channel
                .set_reflected_interlock_threshold(platform::MAXIMUM_REFLECTED_POWER_DBM)
                .unwrap();

            channel
        })
    }

    /// Save the current channel configuration.
    pub fn save_configuration(&mut self) {
        self.settings.save()
    }

    /// Check if the channel RF output is enabled.
    pub fn is_enabled(&self) -> bool {
        self.pins.signal_on.is_set_high()
    }

    /// Check if the channel is powered.
    pub fn is_powered(&self) -> bool {
        self.pins.enable_power.is_set_high()
    }

    /// Set the interlock thresholds for the channel.
    ///
    /// # Args
    /// * `power` - The dBm interlock threshold to configure for reflected power.
    fn set_reflected_interlock_threshold(&mut self, power: f32) -> Result<f32, Error> {
        self.devices
            .interlock_thresholds_dac
            .set_voltage(
                self.settings
                    .settings()
                    .reflected_power_transform
                    .invert(power),
                ad5627::Dac::A,
            )
            .map_err(|e| match e {
                ad5627::Error::Range => Error::Bounds,
                ad5627::Error::I2c(_) => Error::Interface,
            })
    }

    fn apply_output_interlock_threshold(&mut self) -> Result<f32, Error> {
        let settings = self.settings.settings();

        self.devices
            .interlock_thresholds_dac
            .set_voltage(
                settings
                    .output_power_transform
                    .invert(settings.output_interlock_threshold),
                ad5627::Dac::B,
            )
            .map_err(|e| match e {
                ad5627::Error::Range => Error::Bounds,
                ad5627::Error::I2c(_) => Error::Interface,
            })
    }

    fn check_faults(&mut self) -> Option<ChannelFault> {
        let temperature = self.get_temperature();
        if temperature > 60.0 {
            Some(ChannelFault::OverTemperature)
        } else if temperature < 5.0 {
            Some(ChannelFault::UnderTemperature)
        } else if self.pins.alert.is_low() {
            Some(ChannelFault::SupplyAlert)
        } else {
            None
        }
    }

    fn get_overdrive_source(&mut self) -> Option<Interlock> {
        // The schematic indicates the maximum input power is 25dBm. We'll use 20dBm to provide
        // a safety margin.
        if self.get_input_power() > 20.0 {
            Some(Interlock::Input)
        } else if self.pins.output_overdrive.is_high() {
            Some(Interlock::Output)
        } else if self.pins.reflected_overdrive.is_high() {
            Some(Interlock::Reflected)
        } else {
            None
        }
    }

    /// Apply channel settings to the RF channel.
    ///
    /// # Note
    /// This is always implemented in a "least-effort" manner. If settings haven't changed, they
    /// won't be applied.
    ///
    /// # Args
    /// * `new_settings` - The new settings to apply to the channel.
    fn apply_settings(&mut self, new_settings: &ChannelSettings) -> Result<(), Error> {
        let settings = self.settings.settings_mut();

        // If the settings haven't changed, we can short circuit now.
        if settings == new_settings {
            return Ok(());
        }

        let bias_changed = new_settings.bias_voltage != settings.bias_voltage;
        let output_interlock_updated = settings
            .output_power_transform
            .map(settings.output_interlock_threshold)
            != new_settings
                .output_power_transform
                .map(new_settings.output_interlock_threshold);
        let reflected_interlock_updated = settings
            .reflected_power_transform
            .map(platform::MAXIMUM_REFLECTED_POWER_DBM)
            != new_settings
                .reflected_power_transform
                .map(platform::MAXIMUM_REFLECTED_POWER_DBM);

        // Copy transforms before applying the interlock threshold, since the interlock DAC level
        // is calculated from the output interlock transform.
        *settings = *new_settings;

        // Only update the interlock and bias DACs if they've actually changed.
        if output_interlock_updated {
            self.apply_output_interlock_threshold()?;
        }
        if reflected_interlock_updated {
            self.set_reflected_interlock_threshold(platform::MAXIMUM_REFLECTED_POWER_DBM)?;
        }
        if bias_changed {
            self.apply_bias()?;
        }

        Ok(())
    }

    /// Get the temperature of the channel in celsius.
    fn get_temperature(&mut self) -> f32 {
        self.devices
            .temperature_monitor
            .get_remote_temperature()
            .unwrap()
    }

    fn apply_bias(&mut self) -> Result<f32, Error> {
        // The bias voltage is the inverse of the DAC output voltage.
        let bias_voltage = -1.0 * self.settings().bias_voltage;

        match self.devices.bias_dac.set_voltage(bias_voltage) {
            Err(dac7571::Error::Bounds) => Err(Error::Bounds),
            Err(_) => panic!("Failed to set DAC bias voltage"),
            Ok(u) => Ok(u),
        }
    }

    /// Get current power supply measurements from the channel.
    ///
    /// # Returns
    /// The most recent power supply measurements of the channel.
    fn get_supply_measurements(&mut self) -> SupplyMeasurements {
        // Read the cached (scanned) ADC measurements from the monitor.
        let voltages = self.devices.power_monitor.get_voltages().unwrap();

        // The P5V0 rail goes through a resistor divider of 15K -> 10K. This corresponds with a 2.5x
        // reduction in measured voltage.
        let v_p5v0mp = voltages[3] * 2.5;

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
        let i_p28v0ch = voltages[0] * (100.0 / 0.100 / 4300.0);

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
        let i_p5v0ch = voltages[1] * (100.0 / 0.100 / 6200.0);

        SupplyMeasurements {
            v_p5v0mp,
            i_p28v0ch,
            i_p5v0ch,
        }
    }

    /// Get P28V rail current.
    ///
    /// # Returns
    /// The most recent P28V rail current measurements of the channel.
    pub fn get_p28v_current(&mut self) -> f32 {
        let p28v_rail_current_sense = self
            .devices
            .power_monitor
            .get_voltage(ads7924::Channel::Zero)
            .unwrap();

        p28v_rail_current_sense * (100.0 / 0.100 / 4300.0)
    }

    /// Get the current input power measurement.
    ///
    /// # Returns
    /// The input power in dBm.
    fn get_input_power(&mut self) -> f32 {
        let voltage = self.devices.input_power_adc.get_voltage().unwrap();

        self.settings.settings().input_power_transform.map(voltage)
    }

    /// Get the current reflected power measurement.
    ///
    /// # Args
    /// * `adc` - The ADC to use for performing the measurement.
    ///
    /// # Returns
    /// The reflected power in dBm.
    pub fn get_reflected_power(&mut self, adc: &mut hal::adc::Adc<hal::pac::ADC3>) -> f32 {
        let sample = self
            .pins
            .reflected_power
            .convert(adc, SampleTime::Cycles_480);
        let voltage = adc.sample_to_millivolts(sample) as f32 / 1000.0;

        self.settings
            .settings()
            .reflected_power_transform
            .map(voltage)
    }

    /// Get the current output power measurement.
    ///
    /// # Args
    /// * `adc` - The ADC to use for performing the measurement.
    ///
    /// # Returns
    /// The output power in dBm.
    pub fn get_output_power(&mut self, adc: &mut hal::adc::Adc<hal::pac::ADC3>) -> f32 {
        let sample = self.pins.output_power.convert(adc, SampleTime::Cycles_480);
        let voltage = adc.sample_to_millivolts(sample) as f32 / 1000.0;

        self.settings.settings().output_power_transform.map(voltage)
    }

    /// Get the current bias voltage programmed to the RF amplification transistor.
    pub fn get_bias_voltage(&self) -> f32 {
        self.settings.settings().bias_voltage
    }

    pub fn settings(&self) -> &ChannelSettings {
        self.settings.settings()
    }
}

mod sm {
    use super::{ChannelFault, Interlock};
    use crate::hardware::SystemTimer;
    use minimq::embedded_time::Instant;
    use smlang::statemachine;

    impl Copy for States {}
    impl Clone for States {
        fn clone(&self) -> States {
            *self
        }
    }

    impl serde::Serialize for States {
        fn serialize<S: serde::Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
            let (idx, var) = match self {
                States::Blocked(ChannelFault::OverTemperature) => (0, "Blocked(OverTemperature)"),
                States::Blocked(ChannelFault::UnderTemperature) => (0, "Blocked(UnderTemperature)"),
                States::Blocked(ChannelFault::SupplyAlert) => (0, "Blocked(SupplyAlert)"),
                States::Off => (1, "Off"),
                States::Powerup(_) => (2, "Powerup"),
                States::Powered => (3, "Powered"),
                States::Enabled => (4, "Enabled"),
                States::Powerdown(_) => (6, "Powerdown"),
                States::Tripped(Interlock::Output) => (5, "Tripped(Output)"),
                States::Tripped(Interlock::Input) => (5, "Tripped(Input)"),
                States::Tripped(Interlock::Reflected) => (5, "Tripped(Reflected)"),
            };
            serializer.serialize_unit_variant("State", idx, var)
        }
    }

    statemachine! {
        transitions: {
            *Off + InterlockReset [guard_powerup] / start_powerup = Powerup(Instant<SystemTimer>),
            Off + Disable = Off,
            Off + Fault(ChannelFault) / handle_fault = Blocked(ChannelFault),

            Powerup(Instant<SystemTimer>) + Update [check_timeout] / reset_interlocks = Powered,
            Powerup(Instant<SystemTimer>) + Disable / start_disable_instant = Powerdown(Instant<SystemTimer>),
            Powerup(Instant<SystemTimer>) + Fault(ChannelFault) / handle_fault_instant = Blocked(ChannelFault),

            Powered + Update [guard_enable] / enable_output = Enabled,
            Powered + Disable / start_disable = Powerdown(Instant<SystemTimer>),
            Powered + Fault(ChannelFault) / handle_fault = Blocked(ChannelFault),

            Enabled + InterlockReset = Enabled,
            Enabled + Trip(Interlock) / handle_trip = Tripped(Interlock),
            Enabled + DisableRf / disable_rf_switch = Powered,
            Enabled + Disable / start_disable = Powerdown(Instant<SystemTimer>),
            Enabled + Fault(ChannelFault) / handle_fault = Blocked(ChannelFault),

            Tripped(Interlock) + InterlockReset = Powered,
            Tripped(Interlock) + DisableRf = Powered,
            Tripped(Interlock) + Disable / start_disable_interlock = Powerdown(Instant<SystemTimer>),
            Tripped(Interlock) + Fault(ChannelFault) / handle_fault_interlock = Blocked(ChannelFault),

            Powerdown(Instant<SystemTimer>) + Update [check_timeout] = Off,
            Powerdown(Instant<SystemTimer>) + Fault(ChannelFault) / handle_fault_instant = Blocked(ChannelFault),

            Blocked(ChannelFault) + Fault(ChannelFault) / handle_recurrent_fault = Blocked(ChannelFault),
        }
    }
}

impl sm::StateMachineContext for RfChannel {
    /// Handle the occurrence of a tripped interlock.
    fn handle_trip(&mut self, interlock: &Interlock) -> Interlock {
        self.disable_rf_switch();
        *interlock
    }

    /// Turn off the RF output enable switch.
    fn disable_rf_switch(&mut self) {
        self.pins.signal_on.set_low();
    }

    /// Begin the process of powering up the channel.
    ///
    /// # Returns
    /// The time at which the powerup process can be deemed complete.
    fn start_powerup(&mut self) -> Instant<SystemTimer> {
        // Place the bias DAC to drive the RF amplifier into pinch-off during the power-up process.
        self.devices
            .bias_dac
            .set_voltage(3.2)
            .expect("Failed to disable RF bias voltage");

        // Start the LM3880 power supply sequencer.
        self.pins.enable_power.set_high();

        // The LM3880 requires 180ms to power up all supplies on the channel. We add an additional
        // 20ms margin.
        self.clock.try_now().unwrap() + 200_u32.milliseconds()
    }

    fn reset_interlocks(&mut self, _: &Instant<SystemTimer>) {
        // Next, handle resetting interlocks for v1.6 hardware. The interlocks are reset by a
        // falling edge on ON/OFF. Because the bias dac is currently in pinch-off (and the RF
        // channel is unpowered), toggling ON/OFF introduces no output transients on the RF
        // connectors.
        self.pins.signal_on.set_high();

        // Note: The delay here are purely to accomodate potential capacitance on the ON/OFF
        // rail.
        self.delay.delay_ms(1u32);

        self.pins.signal_on.set_low();
    }

    /// Guard against powering up the channel.
    ///
    /// # Returns
    /// Ok if the channel can power up. Err otherwise.
    fn guard_powerup(&mut self) -> Result<(), ()> {
        let settings = self.settings.settings();
        if settings.state == ChannelState::Off {
            Err(())
        } else {
            Ok(())
        }
    }

    /// Check to see if it's currently acceptable to enable the RF output switch.
    ///
    /// # Returns
    /// An error if it's not currently acceptable to enable. Otherwise, Ok.
    fn guard_enable(&mut self) -> Result<(), ()> {
        let settings = self.settings.settings();

        if platform::watchdog_detected() {
            return Err(());
        }

        // It is not valid to enable the channel while the interlock thresholds are low. Due to
        // hardware configurations, it is possible that this would result in a condition where the
        // interlock is never tripped even though the output is exceeding the interlock threshold.
        // As a workaround, we need to ensure that the interlock level is above the output power
        // detector level. When RF is disabled, the power detectors output a near-zero value, so
        // 100mV should be a sufficient level.
        if settings.output_interlock_threshold < settings.output_power_transform.map(0.100) {
            return Err(());
        }

        // Do not enable output if it shouldn't be enabled due to settings.
        if settings.state != ChannelState::Enabled {
            return Err(());
        }

        // Enable power should always be high before we're enabling the RF switch.
        if self.pins.enable_power.is_set_low() {
            return Err(());
        }

        Ok(())
    }

    fn enable_output(&mut self) {
        let settings = self.settings.settings();

        // It is only valid to enable the output if the channel is powered.
        assert!(self.pins.enable_power.is_set_high());
        assert!(settings.output_interlock_threshold > settings.output_power_transform.map(0.100));

        self.apply_bias().unwrap();
        self.pins.signal_on.set_high();
    }

    /// Begin the process of powering down the channel.
    ///
    /// # Returns
    /// The time at which the powerdown process can be deemed complete.
    fn start_disable(&mut self) -> Instant<SystemTimer> {
        self.disable_rf_switch();

        // Set the bias DAC output into pinch-off.
        self.devices
            .bias_dac
            .set_voltage(3.2)
            .expect("Failed to disable RF bias voltage");

        self.pins.enable_power.set_low();

        // Note that we use 500ms here due to the worst-case power-sequencing operation of the
        // LM3880 that occurs when a channel is disabled immediately after enable. In this case,
        // the LM3880 will require 180ms to power up the channel, 120ms to stabilize, and then
        // 180ms to power down the channel.
        self.clock.try_now().unwrap() + 500_u32.milliseconds()
    }

    fn start_disable_instant(&mut self, _: &Instant<SystemTimer>) -> Instant<SystemTimer> {
        self.start_disable()
    }

    fn start_disable_interlock(&mut self, _: &Interlock) -> Instant<SystemTimer> {
        self.start_disable()
    }

    /// Check if a deadline has been met.
    ///
    /// # Returns
    /// Ok if the deadline (timeout) has occurred. Error otherwise.
    fn check_timeout(&mut self, deadline: &Instant<SystemTimer>) -> Result<(), ()> {
        if self.clock.try_now().unwrap() > *deadline {
            Ok(())
        } else {
            Err(())
        }
    }

    /// Handle the occurrence of a channel fault.
    fn handle_fault(&mut self, fault: &ChannelFault) -> ChannelFault {
        self.start_disable();
        *fault
    }

    fn handle_fault_instant(
        &mut self,
        _: &Instant<SystemTimer>,
        fault: &ChannelFault,
    ) -> ChannelFault {
        self.handle_fault(fault)
    }

    fn handle_fault_interlock(&mut self, _: &Interlock, fault: &ChannelFault) -> ChannelFault {
        self.handle_fault(fault)
    }

    fn handle_recurrent_fault(
        &mut self,
        old_fault: &ChannelFault,
        _: &ChannelFault,
    ) -> ChannelFault {
        self.handle_fault(old_fault)
    }
}

pub type RfChannelMachine = sm::StateMachine<RfChannel>;

impl sm::StateMachine<RfChannel> {
    /// Periodically called to update the channel state machine.
    ///
    /// # Returns
    /// The current channel [PowerStatus]
    pub fn update(&mut self) -> PowerStatus {
        // Check for channel faults.
        if let Some(fault) = self.context_mut().check_faults() {
            self.process_event(sm::Events::Fault(fault)).unwrap();
        }

        // Check for interlock trips.
        if matches!(self.state(), &sm::States::Enabled) {
            if let Some(interlock) = self.context_mut().get_overdrive_source() {
                self.process_event(sm::Events::Trip(interlock)).unwrap();
            }
        }

        self.process_event(sm::Events::Update).ok();

        PowerStatus {
            powered: self.context().pins.enable_power.is_set_high(),
            rf_disabled: self.context().pins.signal_on.is_set_low(),
            blocked: matches!(self.state(), &sm::States::Blocked(_)),
        }
    }

    /// Handle the user pressing the "Interlock Reset" button.
    pub fn interlock_reset(&mut self) -> Result<(), sm::Error> {
        self.process_event(sm::Events::InterlockReset)?;
        Ok(())
    }

    /// Handle the user pressing the "Standby" button.
    pub fn standby(&mut self) {
        self.process_event(sm::Events::Disable).ok();
    }

    /// Handle initial startup of the channel.
    pub fn handle_startup(&mut self) {
        // Start powering up the channel. Note that we guard against the current channel
        // configuration state here.
        self.process_event(sm::Events::InterlockReset).ok();
    }

    /// Handle an update to channel settings.
    pub fn handle_settings(&mut self, settings: &ChannelSettings) -> Result<(), Error> {
        self.context_mut().apply_settings(settings)?;

        match (self.state(), settings.state) {
            // It's always acceptable to power off.
            (_, ChannelState::Off) => {
                self.process_event(sm::Events::Disable).ok();
            }

            // For bias tuning, we may need to disable the RF switch.
            (sm::States::Enabled | sm::States::Tripped(_), ChannelState::Powered) => {
                self.process_event(sm::Events::DisableRf).unwrap();
            }

            (sm::States::Off, ChannelState::Powered | ChannelState::Enabled) => {
                self.process_event(sm::Events::InterlockReset).unwrap();
            }

            // Note: Note: Powered -> Enabled transitions are handled via the periodic `Update`
            // service event automatically.
            _ => {}
        }

        Ok(())
    }

    /// Get status information about the channel.
    pub fn get_status(&mut self, adc: &mut hal::adc::Adc<hal::pac::ADC3>) -> ChannelStatus {
        let channel = self.context_mut();

        let power_measurements = channel.get_supply_measurements();

        ChannelStatus {
            reflected_overdrive: channel.pins.reflected_overdrive.is_high(),
            output_overdrive: channel.pins.output_overdrive.is_high(),
            alert: channel.pins.alert.is_low(),
            temperature: channel.get_temperature(),
            p28v_current: power_measurements.i_p28v0ch,
            p5v_current: power_measurements.i_p5v0ch,
            p5v_voltage: power_measurements.v_p5v0mp,
            input_power: channel.get_input_power(),
            output_power: channel.get_output_power(adc),
            reflected_power: channel.get_reflected_power(adc),
            state: *self.state(),
        }
    }
}
