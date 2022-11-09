//! Booster NGFW channel management control interface definitions.
//!
//! # Copyright
///! Copyright (C) 2020-2022 QUARTIQ GmbH

use enum_iterator::IntoEnumIterator;
use stm32f4xx_hal as hal;
use tca9548::{self, Tca9548};

use super::rf_channel::{ChannelPins as RfChannelPins, RfChannel, RfChannelMachine};
use super::{Channel, I2cBusManager, I2cProxy, SystemTimer};
use embedded_hal::blocking::delay::DelayUs;

/// Represents a control structure for interfacing to booster RF channels.
pub struct BoosterChannels {
    channels: [Option<RfChannelMachine>; 8],
    adc: hal::adc::Adc<hal::stm32::ADC3>,
    mux: Tca9548<I2cProxy>,
}

impl From<Channel> for tca9548::Bus {
    fn from(channel: Channel) -> tca9548::Bus {
        match channel {
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
        pins: [RfChannelPins; 8],
        clock: SystemTimer,
        delay: &mut impl DelayUs<u16>,
    ) -> Self {
        let mut channels: [Option<RfChannelMachine>; 8] =
            [None, None, None, None, None, None, None, None];

        for (idx, pins) in Channel::into_enum_iter().zip(pins) {
            // Selecting an I2C bus should never fail.
            mux.select_bus(Some(idx.into()))
                .expect("Failed to select channel");

            if let Some(channel) = RfChannel::new(manager, pins, clock, delay) {
                let mut machine = RfChannelMachine::new(channel);
                machine.handle_startup();
                channels[idx as usize].replace(machine);
            } else {
                info!("Channel {} did not enumerate", idx as usize);
            }
        }

        BoosterChannels { channels, mux, adc }
    }

    /// Select a given channel on the I2C multiplexer and get
    /// mutable references to that channel and the ADC.
    ///
    /// # Args
    /// * `channel` - The channel to get.
    ///
    /// # Returns
    /// An optional pair of mutable references to the channel and the ADC and
    /// `None` if the channel is absent.
    pub fn channel_mut(
        &mut self,
        channel: Channel,
    ) -> Option<(&mut RfChannelMachine, &mut hal::adc::Adc<hal::stm32::ADC3>)> {
        let mux = &mut self.mux;
        let adc = &mut self.adc;
        self.channels[channel as usize].as_mut().map(|ch| {
            mux.select_bus(Some(channel.into())).unwrap();
            (ch, adc)
        })
    }
}
