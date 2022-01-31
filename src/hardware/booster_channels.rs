//! Booster NGFW channel management control interface definitions.
//!
//! # Copyright
//! Copyright (C) 2020 QUARTIQ GmbH - All Rights Reserved
//! Unauthorized usage, editing, or copying is strictly prohibited.
//! Proprietary and confidential.

use super::Channel;
use enum_iterator::IntoEnumIterator;
use stm32f4xx_hal as hal;
use tca9548::{self, Tca9548};

use super::rf_channel::{ChannelPins as RfChannelPins, RfChannel, RfChannelWrapper};
use super::{I2cBusManager, I2cProxy};
use crate::Error;
use embedded_hal::blocking::delay::DelayUs;

/// Represents a control structure for interfacing to booster RF channels.
pub struct BoosterChannels {
    channels: [Option<RfChannelWrapper>; 8],
    adc: hal::adc::Adc<hal::stm32::ADC3>,
    mux: Tca9548<I2cProxy>,
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
        delay: &mut impl DelayUs<u16>,
    ) -> Self {
        let mut rf_channels: [Option<RfChannelWrapper>; 8] =
            [None, None, None, None, None, None, None, None];

        for channel in Channel::into_enum_iter() {
            // Selecting an I2C bus should never fail.
            mux.select_bus(Some(channel.into()))
                .expect("Failed to select channel");

            let control_pins = pins[channel as usize]
                .take()
                .expect("Channel pins not available");

            if let Some(rf_channel) = RfChannel::new(&manager, control_pins, delay) {
                rf_channels[channel as usize].replace(RfChannelWrapper::new(rf_channel));
            } else {
                info!("Channel {} did not enumerate", channel as usize);
            }
        }

        BoosterChannels {
            channels: rf_channels,
            mux: mux,
            adc: adc,
        }
    }

    /// Perform an action on a channel.
    ///
    /// # Args
    /// * `channel` - The channel to perform the action on.
    /// * `func` - A function called with the channel selected and
    ///     with the channel and the ADC3 peripheral passed as arguments.
    pub fn map<F, R>(&mut self, channel: Channel, func: F) -> Result<R, Error>
    where
        F: FnOnce(&mut RfChannel, &mut hal::adc::Adc<hal::stm32::ADC3>) -> Result<R, Error>,
    {
        let mux = &mut self.mux;
        let adc = &mut self.adc;
        self.channels[channel as usize]
            .as_mut()
            .ok_or(Error::NotPresent)
            .and_then(|ch| {
                mux.select_bus(Some(channel.into())).unwrap();
                func(ch.0.context_mut(), adc)
            })
    }

    pub fn channel_mut(&mut self, channel: Channel) -> Result<&mut RfChannelWrapper, Error> {
        if let Some(ref mut rf_channel) = self.channels[channel as usize] {
            self.mux.select_bus(Some(channel.into())).unwrap();
            Ok(rf_channel)
        } else {
            Err(Error::NotPresent)
        }
    }
}
