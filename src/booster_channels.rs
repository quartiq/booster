//! Booster NGFW channel management control interface definitions.
//!
//! # Copyright
//! Copyright (C) 2020 QUARTIQ GmbH - All Rights Reserved
//! Unauthorized usage, editing, or copying is strictly prohibited.
//! Proprietary and confidential.

use enum_iterator::IntoEnumIterator;
use stm32f4xx_hal::{self as hal, prelude::*};
use tca9548::{self, Tca9548};

use super::{I2cBusManager, I2cProxy};
use crate::error::Error;
use crate::rf_channel::{ChannelPins as RfChannelPins, RfChannel};
use embedded_hal::blocking::delay::DelayUs;

/// A EUI-48 identifier for a given channel.
pub struct ChannelIdentifier {
    pub data: [u8; 6],
}

impl ChannelIdentifier {
    /// Construct a new channel identifier.
    pub fn new(identifier: [u8; 6]) -> Self {
        Self { data: identifier }
    }
}

/// Contains channel status information in SI base units.
#[derive(Debug)]
pub struct ChannelStatus {
    pub reflected_overdrive: bool,
    pub output_overdrive: bool,
    pub alert: bool,
    pub outputting: bool,
    pub temperature: f32,
    pub p28v_current: f32,
    pub p5v_current: f32,
    pub p5v_voltage: f32,
    pub input_power: f32,
    pub reflected_power: f32,
    pub output_power: f32,
    pub reflected_overdrive_threshold: f32,
    pub output_overdrive_threshold: f32,
    pub bias_voltage: f32,
}

/// Represents a control structure for interfacing to booster RF channels.
pub struct BoosterChannels {
    channels: [Option<RfChannel>; 8],
    adc: core::cell::RefCell<hal::adc::Adc<hal::stm32::ADC3>>,
    mux: Tca9548<I2cProxy>,
}

/// Indicates a booster RF channel.
#[derive(IntoEnumIterator, Copy, Clone, Debug)]
pub enum Channel {
    Zero = 0,
    One = 1,
    Two = 2,
    Three = 3,
    Four = 4,
    Five = 5,
    Six = 6,
    Seven = 7,
}

impl Into<tca9548::Bus> for Channel {
    fn into(self) -> tca9548::Bus {
        match self {
            Channel::Zero => tca9548::Bus::Zero,
            Channel::One => tca9548::Bus::One,
            Channel::Two => tca9548::Bus::Two,
            Channel::Three => tca9548::Bus::Three,
            Channel::Four => tca9548::Bus::Four,
            Channel::Five => tca9548::Bus::Five,
            Channel::Six => tca9548::Bus::Six,
            Channel::Seven => tca9548::Bus::Seven,
        }
    }
}

impl BoosterChannels {
    /// Construct booster RF channels.
    ///
    /// # Note
    /// This function will scan channels to check if they are present.
    ///
    /// # Args
    /// * `mux` - The I2C mux used for switching between channel communications.
    /// * `adc` - The ADC used to measure analog channels.
    /// * `manager` - The I2C bus manager used for the shared I2C bus.
    /// * `pins` - An array of all RfChannel control/status pins.
    /// * `delay` - A means of delaying during setup.
    ///
    /// # Returns
    /// A `BoosterChannels` object that can be used to manage all available RF channels.
    pub fn new(
        mut mux: Tca9548<I2cProxy>,
        adc: hal::adc::Adc<hal::stm32::ADC3>,
        manager: &'static I2cBusManager,
        mut pins: [Option<RfChannelPins>; 8],
        delay: &mut impl DelayUs<u8>,
    ) -> Self {
        let mut rf_channels: [Option<RfChannel>; 8] =
            [None, None, None, None, None, None, None, None];

        for channel in Channel::into_enum_iter() {
            // Selecting an I2C bus should never fail.
            mux.select_bus(Some(channel.into()))
                .expect("Failed to select channel");

            let control_pins = pins[channel as usize]
                .take()
                .expect("Channel pins not available");

            match RfChannel::new(&manager, control_pins, delay) {
                Some(mut rf_channel) => {
                    // Setting interlock thresholds should not fail here as we have verified the
                    // device is on the bus.
                    rf_channel.set_interlock_thresholds(0.0, 0.0).unwrap();
                    rf_channels[channel as usize].replace(rf_channel);
                }
                None => {
                    info!("Channel {} did not enumerate", channel as usize);
                }
            }
        }

        BoosterChannels {
            channels: rf_channels,
            mux: mux,
            adc: core::cell::RefCell::new(adc),
        }
    }

    /// Set the interlock thresholds for the channel.
    ///
    /// # Args
    /// * `channel` - The RF channel to set thresholds for.
    /// * `forward_threshold` - The dBm interlock threshold for forward power.
    /// * `reflected_threshold` - The dBm interlock threshold for reflected power.
    pub fn set_interlock_thresholds(
        &mut self,
        channel: Channel,
        forward_threshold: f32,
        reflected_threshold: f32,
    ) -> Result<(), Error> {
        self.mux.select_bus(Some(channel.into())).unwrap();

        match &mut self.channels[channel as usize] {
            Some(rf_channel) => {
                match rf_channel.set_interlock_thresholds(forward_threshold, reflected_threshold) {
                    // Configuring a present channel should never have an interface failure
                    // (although the requested value may be out of range).
                    Err(Error::Interface) => {
                        panic!("Failed to configure thresholds on CH{}", channel as usize);
                    }
                    x => x,
                }
            }
            None => Err(Error::NotPresent),
        }
    }

    /// Get a channel's EUI-48 identifier.
    ///
    /// # Args
    /// * `channel` - The channel to get the identifier from.
    ///
    /// # Returns
    /// The channel identifier.
    pub fn get_identifier(&mut self, channel: Channel) -> Result<ChannelIdentifier, Error> {
        self.mux.select_bus(Some(channel.into())).unwrap();

        match &mut self.channels[channel as usize] {
            Some(rf_channel) => {
                let mut identifier: [u8; 6] = [0; 6];
                rf_channel
                    .i2c_devices
                    .eui48
                    .read_eui48(&mut identifier)
                    .unwrap();
                Ok(ChannelIdentifier::new(identifier))
            }
            None => Err(Error::NotPresent),
        }
    }

    /// Check if a channel has encountered an overload condition.
    ///
    /// # Args
    /// * `channel` - The channel to check.
    ///
    /// # Returns
    /// True if an overload condition is present on the channel.
    pub fn overload_detected(&mut self, channel: Channel) -> Result<bool, Error> {
        self.mux.select_bus(Some(channel.into())).unwrap();

        match &mut self.channels[channel as usize] {
            Some(rf_channel) => Ok(rf_channel.is_overdriven()),
            None => Err(Error::NotPresent),
        }
    }

    /// Check if a channel can enable.
    ///
    /// # Args
    /// * `channel` - The channel to check.
    ///
    /// # Returns
    /// True if the channel can be enabled.
    pub fn channel_can_enable(&mut self, channel: Channel) -> Result<bool, Error> {
        self.mux.select_bus(Some(channel.into())).unwrap();

        match &self.channels[channel as usize] {
            Some(rf_channel) => Ok(rf_channel.can_enable()),
            None => Err(Error::NotPresent),
        }
    }

    /// Check if a channel is standing by (not outputting).
    ///
    /// # Args
    /// * `channel` - The channel to check.
    ///
    /// # Returns
    /// True if the channel is standing by and may be re-enabled.
    pub fn is_standing_by(&mut self, channel: Channel) -> Result<bool, Error> {
        self.mux.select_bus(Some(channel.into())).unwrap();

        match &self.channels[channel as usize] {
            Some(rf_channel) => Ok(rf_channel.is_standing_by()),
            None => Err(Error::NotPresent),
        }
    }

    /// Enable an RF channel.
    ///
    /// # Args
    /// * `channel` - The channel to enable.
    pub fn enable_channel(&mut self, channel: Channel) -> Result<(), Error> {
        self.mux.select_bus(Some(channel.into())).unwrap();

        match &mut self.channels[channel as usize] {
            Some(rf_channel) => rf_channel.start_enable(),
            None => Err(Error::NotPresent),
        }
    }

    /// Toggle an RF channel from standby
    ///
    /// # Args
    /// * `channel` - The channel to toggle.
    pub fn toggle_standby(&mut self, channel: Channel) -> Result<(), Error> {
        self.mux.select_bus(Some(channel.into())).unwrap();

        match &mut self.channels[channel as usize] {
            Some(rf_channel) => Ok(rf_channel.toggle_standby()),
            None => Err(Error::NotPresent),
        }
    }

    /// Check if an RF channel is enabled.
    ///
    /// # Note
    /// This will return true even if the channel interlock is tripped.
    ///
    /// # Args
    /// * `channel` - The channel to check.
    pub fn is_enabled(&mut self, channel: Channel) -> Result<bool, Error> {
        self.mux.select_bus(Some(channel.into())).unwrap();

        match &mut self.channels[channel as usize] {
            Some(rf_channel) => Ok(rf_channel.is_enabled()),
            None => Err(Error::NotPresent),
        }
    }

    /// Disable an RF channel.
    ///
    /// # Args
    /// * `channel` - The channel to disable.
    pub fn disable_channel(&mut self, channel: Channel) -> Result<(), Error> {
        self.mux.select_bus(Some(channel.into())).unwrap();

        match &mut self.channels[channel as usize] {
            Some(rf_channel) => Ok(rf_channel.start_disable()),
            None => Err(Error::NotPresent),
        }
    }

    /// Reset interlocks of an RF channel.
    ///
    /// # Args
    /// * `channel` - The channel to reset the interlocks of.
    pub fn reset_channel_interlocks(&mut self, channel: Channel) -> Result<(), Error> {
        self.mux.select_bus(Some(channel.into())).unwrap();

        match &mut self.channels[channel as usize] {
            Some(rf_channel) => Ok(rf_channel.reset_interlocks()),
            None => Err(Error::NotPresent),
        }
    }

    /// Get the temperature of a channel.
    ///
    /// # Args
    /// * `channel` - The channel to get the temperature of.
    ///
    /// # Returns
    /// The temperature of the channel in degrees celsius.
    pub fn get_temperature(&mut self, channel: Channel) -> Result<f32, Error> {
        self.mux.select_bus(Some(channel.into())).unwrap();

        match &mut self.channels[channel as usize] {
            Some(rf_channel) => Ok(rf_channel.get_temperature()),
            None => Err(Error::NotPresent),
        }
    }

    /// Set the bias voltage of a channel.
    ///
    /// # Args
    /// * `channel` - The channel to set the bias voltage of.
    /// * `bias_voltage` - The desired bias voltage to apply to the RF amplification transistor.
    pub fn set_bias(&mut self, channel: Channel, bias_voltage: f32) -> Result<(), Error> {
        self.mux.select_bus(Some(channel.into())).unwrap();

        match &mut self.channels[channel as usize] {
            Some(rf_channel) => {
                rf_channel.set_bias(bias_voltage)?;
                Ok(())
            }
            None => Err(Error::NotPresent),
        }
    }

    /// Get the current status of the channel.
    ///
    /// # Args
    /// * `channel` - The channel to get the status of.
    ///
    /// Returns
    /// A structure indicating all measurements on the channel.
    pub fn get_status(&mut self, channel: Channel) -> Result<ChannelStatus, Error> {
        self.mux.select_bus(Some(channel.into())).unwrap();

        match &mut self.channels[channel as usize] {
            Some(rf_channel) => {
                let power_measurements = rf_channel.get_supply_measurements();

                let mut adc = self.adc.borrow_mut();

                let status = ChannelStatus {
                    reflected_overdrive: rf_channel.pins.reflected_overdrive.is_high().unwrap(),
                    output_overdrive: rf_channel.pins.output_overdrive.is_high().unwrap(),
                    alert: rf_channel.is_alarmed(),
                    outputting: rf_channel.is_outputting(),
                    temperature: rf_channel.get_temperature(),
                    p28v_current: power_measurements.i_p28v0ch,
                    p5v_current: power_measurements.i_p5v0ch,
                    p5v_voltage: power_measurements.v_p5v0mp,
                    input_power: rf_channel.get_input_power(),
                    output_power: rf_channel.get_output_power(&mut adc),
                    reflected_power: rf_channel.get_reflected_power(&mut adc),
                    reflected_overdrive_threshold: rf_channel.get_reflected_interlock_threshold(),
                    output_overdrive_threshold: rf_channel.get_output_interlock_threshold(),
                    bias_voltage: rf_channel.get_bias_voltage(),
                };

                Ok(status)
            }
            None => Err(Error::NotPresent),
        }
    }

    /// Update the states of RF channels as necessary.
    pub fn update(&mut self) {
        for channel in Channel::into_enum_iter() {
            self.mux.select_bus(Some(channel.into())).unwrap();

            if let Some(rf_channel) = &mut self.channels[channel as usize] {
                rf_channel.update().unwrap();
            }
        }
    }
}
