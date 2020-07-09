//! Booster NGFW Application
//!
//! # Copyright
//! Copyright (C) 2020 QUARTIQ GmbH - All Rights Reserved
//! Unauthorized usage, editing, or copying is strictly prohibited.
//! Proprietary and confidential.
#![no_std]
#![no_main]

#[macro_use]
extern crate log;

use cortex_m::asm;
use panic_halt as _;
use shared_bus_rtic::{self, BusProxy};
use stm32f4xx_hal as hal;

use hal::prelude::*;

mod booster_channels;
mod error;
mod rf_channel;
use booster_channels::BoosterChannels;
use rf_channel::{AdcPin, AnalogPins as AdcPins, ChannelPins as RfChannelPins};

// Convenience type definition for the I2C bus used for booster RF channels.
type I2C = hal::i2c::I2c<
    hal::stm32::I2C1,
    (
        hal::gpio::gpiob::PB6<hal::gpio::AlternateOD<hal::gpio::AF4>>,
        hal::gpio::gpiob::PB7<hal::gpio::AlternateOD<hal::gpio::AF4>>,
    ),
>;

// Convenience type definition for the shared bus BusManager type.
type BusManager = shared_bus_rtic::shared_bus::BusManager<shared_bus_rtic::Mutex<I2C>, I2C>;

/// Construct ADC pins associated with an RF channel.
///
/// # Args
/// * `gpio` - The GPIO port used to instantiate analog pins.
/// * `tx_power` - The name of the pin to instantiate for the TX power measurement.
/// * `reflected_power` - The name of the pin to instantiate for the reflected power measurement.
///
/// # Returns
/// An AdcPin enumeration describing the ADC pins.
macro_rules! adc_pins {
    ($gpio:ident, $tx_power:ident, $reflected_power:ident) => {{
        let tx_power = AdcPin::$tx_power($gpio.$tx_power.into_analog());
        let reflected_power = AdcPin::$reflected_power($gpio.$reflected_power.into_analog());
        AdcPins::new(tx_power, reflected_power)
    }};
}

/// Macro for genering an RfChannelPins structure.
///
/// # Args
/// * `gpiod` - The GPIOD Parts structure to extract pins from.
/// * `gpioe` - The GPIOE Parts structure to extract pins from.
/// * `gpiog` - The GPIOG Parts structure to extract pins from.
/// * `enable` - The pin ID of the enable pin in GPIOD.
/// * `alert` - The pin ID of the alert pin in GPIOD.
/// * `input_overdrive` - The pin ID of the input overdrive pin in GPIOE.
/// * `output_overdrive` - The pin ID of the output overdrive pin in GPIOE.
/// * `signal_on` - The pin ID of the signal on pin in GPIOG.
///
/// # Returns
/// An option containing the RfChannelPins structure.
macro_rules! channel_pins {
    ($gpiod:ident, $gpioe:ident, $gpiog:ident, $enable:ident, $alert:ident, $input_overdrive:ident,
     $output_overdrive:ident, $signal_on:ident, $analog_pins:ident) => {{
        let enable_power = $gpiod.$enable.into_push_pull_output().downgrade();
        let alert = $gpiod.$alert.into_floating_input().downgrade();
        let input_overdrive = $gpioe.$input_overdrive.into_floating_input().downgrade();
        let output_overdrive = $gpioe.$output_overdrive.into_pull_down_input().downgrade();
        let signal_on = $gpiog.$signal_on.into_push_pull_output().downgrade();

        Some(RfChannelPins::new(
            enable_power,
            alert,
            input_overdrive,
            output_overdrive,
            signal_on,
            $analog_pins,
        ))
    }};
}

#[rtic::app(device = stm32f4xx_hal::stm32, peripherals = true)]
const APP: () = {
    struct Resources {
        channels: BoosterChannels,
    }

    #[init]
    fn init(c: init::Context) -> init::LateResources {
        let cp = cortex_m::peripheral::Peripherals::take().unwrap();

        // Initialize the chip
        let rcc = c.device.RCC.constrain();

        // TODO: Determine an ideal operating point for the system clock. Currently set to max.
        let clocks = rcc
            .cfgr
            .use_hse(8.mhz())
            .sysclk(168.mhz())
            .hclk(168.mhz())
            .require_pll48clk()
            .freeze();

        let mut delay = hal::delay::Delay::new(cp.SYST, clocks);

        let gpioa = c.device.GPIOA.split();
        let gpiob = c.device.GPIOB.split();
        let gpioc = c.device.GPIOC.split();
        let gpiod = c.device.GPIOD.split();
        let gpioe = c.device.GPIOE.split();
        let gpiof = c.device.GPIOF.split();
        let gpiog = c.device.GPIOG.split();

        let i2c_bus_manager = {
            let i2c = {
                let scl = gpiob.pb6.into_alternate_af4_open_drain();
                let sda = gpiob.pb7.into_alternate_af4_open_drain();
                hal::i2c::I2c::i2c1(c.device.I2C1, (scl, sda), 100.khz(), clocks)
            };

            shared_bus_rtic::new!(i2c, I2C)
        };

        // Instantiate the I2C interface to the I2C mux. Use a shared-bus so we can share the I2C
        // bus with all of the Booster peripheral devices.
        let channels = {
            let channel_pins = {
                let ch1_pins = {
                    let analog_pins = adc_pins!(gpioa, pa0, pa1);
                    channel_pins!(gpiod, gpioe, gpiog, pd0, pd8, pe8, pe0, pg8, analog_pins)
                };
                let ch2_pins = {
                    let analog_pins = adc_pins!(gpioa, pa2, pa3);
                    channel_pins!(gpiod, gpioe, gpiog, pd1, pd9, pe9, pe1, pg9, analog_pins)
                };
                let ch3_pins = {
                    let analog_pins = adc_pins!(gpiof, pf6, pf7);
                    channel_pins!(gpiod, gpioe, gpiog, pd2, pd10, pe10, pe2, pg10, analog_pins)
                };
                let ch4_pins = {
                    let analog_pins = adc_pins!(gpiof, pf8, pf9);
                    channel_pins!(gpiod, gpioe, gpiog, pd3, pd11, pe11, pe3, pg11, analog_pins)
                };
                let ch5_pins = {
                    let analog_pins = adc_pins!(gpiof, pf10, pf3);
                    channel_pins!(gpiod, gpioe, gpiog, pd4, pd12, pe12, pe4, pg12, analog_pins)
                };
                let ch6_pins = {
                    let analog_pins = adc_pins!(gpioc, pc0, pc1);
                    channel_pins!(gpiod, gpioe, gpiog, pd5, pd13, pe13, pe5, pg13, analog_pins)
                };
                let ch7_pins = {
                    let analog_pins = adc_pins!(gpioc, pc2, pc3);
                    channel_pins!(gpiod, gpioe, gpiog, pd6, pd14, pe14, pe6, pg14, analog_pins)
                };
                let ch8_pins = {
                    let analog_pins = adc_pins!(gpiof, pf4, pf5);
                    channel_pins!(gpiod, gpioe, gpiog, pd7, pd15, pe15, pe7, pg15, analog_pins)
                };

                [
                    ch1_pins, ch2_pins, ch3_pins, ch4_pins, ch5_pins, ch6_pins, ch7_pins, ch8_pins,
                ]
            };

            let mux = {
                let mut i2c_mux_reset = gpiob.pb14.into_push_pull_output();
                tca9548::Tca9548::default(i2c_bus_manager.acquire(), &mut i2c_mux_reset, &mut delay)
                    .unwrap()
            };

            BoosterChannels::new(mux, &i2c_bus_manager, channel_pins)
        };

        info!("Startup complete");

        init::LateResources { channels: channels }
    }

    #[idle(resources=[channels])]
    fn idle(_: idle::Context) -> ! {
        loop {
            asm::nop();
        }
    }
};
