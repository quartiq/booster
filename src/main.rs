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
use enum_iterator::IntoEnumIterator;
use panic_halt as _;
use stm32f4xx_hal as hal;

use hal::{prelude::*, timer::Event};

mod booster_channels;
mod chassis_fans;
mod error;
mod linear_transformation;
mod mutex;
mod rf_channel;
use booster_channels::{BoosterChannels, Channel};
use chassis_fans::ChassisFans;
use error::Error;
use rf_channel::{AdcPin, AnalogPins as AdcPins, ChannelPins as RfChannelPins};

// Convenience type definition for the I2C bus used for booster RF channels.
type I2C = hal::i2c::I2c<
    hal::stm32::I2C1,
    (
        hal::gpio::gpiob::PB6<hal::gpio::AlternateOD<hal::gpio::AF4>>,
        hal::gpio::gpiob::PB7<hal::gpio::AlternateOD<hal::gpio::AF4>>,
    ),
>;

type I2cBusManager = mutex::AtomicCheckManager<I2C>;
type I2cProxy = shared_bus::I2cProxy<'static, mutex::AtomicCheckMutex<I2C>>;

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
        monitor_timer: hal::timer::Timer<hal::stm32::TIM2>,
        telemetry_timer: hal::timer::Timer<hal::stm32::TIM3>,
        channels: BoosterChannels,
    }

    #[init]
    fn init(c: init::Context) -> init::LateResources {
        let mut cp = cortex_m::peripheral::Peripherals::take().unwrap();
        cp.DWT.enable_cycle_counter();

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

        let mut pa_ch_reset_n = gpiob.pb9.into_push_pull_output();
        pa_ch_reset_n.set_high().unwrap();

        let i2c_bus_manager: &'static _ = {
            let i2c = {
                let scl = gpiob.pb6.into_alternate_af4_open_drain();
                let sda = gpiob.pb7.into_alternate_af4_open_drain();
                hal::i2c::I2c::i2c1(c.device.I2C1, (scl, sda), 100.khz(), clocks)
            };

            new_atomic_check_manager!(I2C = i2c).unwrap()
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

            let mut mux = {
                let mut i2c_mux_reset = gpiob.pb14.into_push_pull_output();
                tca9548::Tca9548::default(
                    i2c_bus_manager.acquire_i2c(),
                    &mut i2c_mux_reset,
                    &mut delay,
                )
                .unwrap()
            };

            // Test scanning and reading back MUX channels.
            assert!(mux.self_test().unwrap() == true);

            let adc = hal::adc::Adc::adc3(
                c.device.ADC3,
                true,
                2500,
                hal::adc::config::AdcConfig::default(),
            );

            BoosterChannels::new(mux, adc, i2c_bus_manager, channel_pins, &mut delay)
        };

        let i2c2 = {
            let scl = gpiob.pb10.into_alternate_af4_open_drain();
            let sda = gpiob.pb11.into_alternate_af4_open_drain();
            hal::i2c::I2c::i2c2(c.device.I2C2, (scl, sda), 100.khz(), clocks)
        };

        // Selftest: Read the EUI48 identifier.
        let mut eui = microchip_24aa02e48::Microchip24AA02E48::new(i2c2).unwrap();
        let mut eui48: [u8; 6] = [0; 6];
        eui.read_eui48(&mut eui48).unwrap();

        let fan1 =
            max6639::Max6639::new(i2c_bus_manager.acquire_i2c(), max6639::AddressPin::Pulldown)
                .unwrap();
        let fan2 = max6639::Max6639::new(i2c_bus_manager.acquire_i2c(), max6639::AddressPin::Float)
            .unwrap();
        let fan3 =
            max6639::Max6639::new(i2c_bus_manager.acquire_i2c(), max6639::AddressPin::Pullup)
                .unwrap();

        let mut fans = ChassisFans::new([fan1, fan2, fan3]);
        assert!(fans.self_test(&mut delay));

        info!("Startup complete");

        // Configure a timer to periodically monitor the output channels.
        let mut monitor_timer = hal::timer::Timer::tim2(c.device.TIM2, 50.hz(), clocks);
        monitor_timer.listen(Event::TimeOut);

        // Configure a timer to periodically gather telemetry.
        let mut telemetry_timer = hal::timer::Timer::tim3(c.device.TIM3, 2.hz(), clocks);
        telemetry_timer.listen(Event::TimeOut);

        init::LateResources {
            monitor_timer: monitor_timer,
            telemetry_timer: telemetry_timer,
            channels: channels,
        }
    }

    #[task(binds=TIM2, priority=2, resources=[monitor_timer, channels])]
    fn channel_monitor(c: channel_monitor::Context) {
        c.resources.monitor_timer.clear_interrupt(Event::TimeOut);

        // Update the state of any channels.
        // TODO: Only need to call this periodically (e.g. every 60-200ms or so).
        c.resources.channels.update();

        // Check all of the timer channels.
        for channel in Channel::into_enum_iter() {
            let _error_detected = match c.resources.channels.error_detected(channel) {
                Err(Error::NotPresent) => {
                    // TODO: Clear all LEDs for this channel.
                    continue;
                }
                Ok(detected) => detected,
                Err(error) => panic!("Encountered error: {:?}", error),
            };

            let _warning_detected = match c.resources.channels.warning_detected(channel) {
                Ok(detected) => detected,
                Err(error) => panic!("Encountered error: {:?}", error),
            };

            let _enabled = match c.resources.channels.is_enabled(channel) {
                Ok(detected) => detected,
                Err(error) => panic!("Encountered error: {:?}", error),
            };

            // TODO: Echo the measured values to the LEDs on the user interface for this channel.
        }
    }

    #[task(binds=TIM3, priority=1, resources=[telemetry_timer, channels])]
    fn telemetry(mut c: telemetry::Context) {
        c.resources.telemetry_timer.clear_interrupt(Event::TimeOut);

        // Gather telemetry for all of the channels.
        for channel in Channel::into_enum_iter() {
            let measurements = c
                .resources
                .channels
                .lock(|booster_channels| booster_channels.get_status(channel));

            // TODO: Broadcast the measured data over the telemetry interface.
            info!("{:?}", measurements);
        }
    }

    #[idle(resources=[channels])]
    fn idle(_: idle::Context) -> ! {
        loop {
            asm::nop();
        }
    }
};
