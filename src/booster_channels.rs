//! Booster NGFW channel management control interface definitions.
//!
//! # Copyright
//! Copyright (C) 2020 QUARTIQ GmbH - All Rights Reserved
//! Unauthorized usage, editing, or copying is strictly prohibited.
//! Proprietary and confidential.

use enum_iterator::IntoEnumIterator;
use tca9548::{self, Tca9548};

use super::{BusManager, BusProxy, I2C};
use crate::error::Error;
use crate::rf_channel::{ChannelPins as RfChannelPins, RfChannel};

pub struct ChannelIdentifier {
    pub data: [u8; 6]
}

impl ChannelIdentifier {
    pub fn new(identifier: [u8; 6]) -> Self {
        Self {
            data: identifier
        }

    }
}

pub struct ChannelStatus {
}

/// Represents a control structure for interfacing to booster RF channels.
pub struct BoosterChannels {
    channels: [Option<RfChannel>; 8],
    mux: Tca9548<BusProxy<I2C>>,
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
    /// * `manager` - The I2C bus manager used for the shared I2C bus.
    /// * `pins` - An array of all RfChannel control/status pins.
    ///
    /// # Returns
    /// A `BoosterChannels` object that can be used to manage all available RF channels.
    pub fn new(
        mut mux: Tca9548<BusProxy<I2C>>,
        manager: &'static BusManager,
        mut pins: [Option<RfChannelPins>; 8],
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

            match RfChannel::new(manager, control_pins) {
                Some(mut rf_channel) => {
                    // Setting interlock thresholds should not fail here as we have verified the
                    // device is on the bus.
                    rf_channel.set_interlock_thresholds(-30.0, -30.0).unwrap();
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
        if self.channels[channel as usize].is_none() {
            return Err(Error::NotPresent);
        }

        // Selecting an I2C bus should never fail.
        self.mux
            .select_bus(Some(channel.into()))
            .expect("Failed to select channel");

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

    pub fn is_enabled(&mut self, channel: Channel) -> Result<bool, Error> {
        self.mux.select_bus(Some(channel.into())).unwrap();

        match &self.channels[channel as usize] {
            Some(rf_channel) => Ok(rf_channel.is_enabled()),
            None => Err(Error::NotPresent),
        }
    }

    pub fn get_identifier(&mut self, channel: Channel) -> Result<ChannelIdentifier, Error> {
        self.mux.select_bus(Some(channel.into())).unwrap();

        match &mut self.channels[channel as usize] {
            Some(rf_channel) => {
                let mut identifier: [u8; 6] = [0; 6];
                rf_channel.i2c_devices.eui48.read_eui48(&mut identifier).unwrap();
                Ok(ChannelIdentifier::new(identifier))
            },
            None => Err(Error::NotPresent),
        }
    }

    pub fn warning_detected(&mut self, channel: Channel) -> Result<bool, Error> {
        // Check if any alerts/alarms are present on the channel.
        // TODO: Determine what other conditions may constitute a warning to the user.

        self.mux.select_bus(Some(channel.into())).unwrap();

        match &self.channels[channel as usize] {
            Some(rf_channel) => Ok(rf_channel.is_alarmed()),
            None => Err(Error::NotPresent),
        }
    }

    pub fn error_detected(&mut self, channel: Channel) -> Result<bool, Error> {
        // If input or output overdrive is detected

        self.mux.select_bus(Some(channel.into())).unwrap();

        match &self.channels[channel as usize] {
            Some(rf_channel) => Ok(rf_channel.is_overdriven()),
            None => Err(Error::NotPresent),
        }
    }

    pub fn enable_channel(&mut self, channel: Channel) -> Result<(), Error> {

        self.mux.select_bus(Some(channel.into())).unwrap();

        match &mut self.channels[channel as usize] {
            Some(rf_channel) => rf_channel.enable(),
            None => Err(Error::NotPresent),
        }
    }

    pub fn disable_channel(&mut self, channel: Channel) -> Result<(), Error> {
        self.mux.select_bus(Some(channel.into())).unwrap();

        match &mut self.channels[channel as usize] {
            Some(rf_channel) => rf_channel.disable(),
            None => Err(Error::NotPresent),
        }
    }

    pub fn get_temperature(&mut self, channel: Channel) -> Result<f32, Error> {
        self.mux.select_bus(Some(channel.into())).unwrap();

        match &mut self.channels[channel as usize] {
            Some(rf_channel) => rf_channel.get_temperature(),
            None => Err(Error::NotPresent),
        }
    }

    pub fn set_bias(&mut self, channel: Channel, bias_voltage: f32) -> Result<(), Error> {
        self.mux.select_bus(Some(channel.into())).unwrap();

        match &mut self.channels[channel as usize] {
            Some(rf_channel) => {
                rf_channel.set_bias(bias_voltage)?;
                Ok(())
            },
            None => Err(Error::NotPresent),
        }
    }

    pub fn get_status(&mut self, _channel: Channel) -> Result<ChannelStatus, Error> {
        // TODO: Fill out.
        Ok(ChannelStatus{})
    }
}
