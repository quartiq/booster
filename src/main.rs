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

use enum_iterator::IntoEnumIterator;
use panic_halt as _;
use stm32f4xx_hal as hal;

use hal::{gpio::ExtiPin, prelude::*, timer::Event};

mod booster_channels;
mod error;
mod linear_transformation;
mod rf_channel;
mod user_interface;
mod monotonic;
use monotonic::Instant;
use booster_channels::{BoosterChannels, Channel};
use error::Error;
use rf_channel::{AdcPin, AnalogPins as AdcPins, ChannelPins as RfChannelPins};
use user_interface::{ButtonEvent, Color, UserButtons, UserLeds};

// Convenience type definition for the I2C bus used for booster RF channels.
type I2C = hal::i2c::I2c<
    hal::stm32::I2C1,
    (
        hal::gpio::gpiob::PB6<hal::gpio::AlternateOD<hal::gpio::AF4>>,
        hal::gpio::gpiob::PB7<hal::gpio::AlternateOD<hal::gpio::AF4>>,
    ),
>;

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

#[rtic::app(device = stm32f4xx_hal::stm32, peripherals = true, monotonic = monotonic::Tim5)]
const APP: () = {
    struct Resources {
        monitor_timer: hal::timer::Timer<hal::stm32::TIM2>,
        telemetry_timer: hal::timer::Timer<hal::stm32::TIM3>,
        channels: BoosterChannels,
        buttons: UserButtons,
        leds: UserLeds,
    }

    #[init]
    fn init(mut c: init::Context) -> init::LateResources {
        let cp = cortex_m::peripheral::Peripherals::take().unwrap();

        // Initialize the chip
        let rcc = c.device.RCC.constrain();

        // TODO: Determine an ideal operating point for the system clock. Currently set to max.
        let clocks = rcc
            .cfgr
            .use_hse(8.mhz())
            .sysclk(168.mhz())
            .hclk(168.mhz())
            .pclk1(42.mhz())
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

            let adc =
                hal::adc::Adc::adc3(c.device.ADC3, true, hal::adc::config::AdcConfig::default());

            BoosterChannels::new(mux, adc, i2c_bus_manager, channel_pins)
        };

        let buttons = {
            let mut button1 = gpiof.pf14.into_floating_input();

            button1.make_interrupt_source(&mut c.device.SYSCFG);
            button1.trigger_on_edge(&mut c.device.EXTI, hal::gpio::Edge::RISING_FALLING);
            button1.clear_interrupt_pending_bit();
            button1.enable_interrupt(&mut c.device.EXTI);

            let mut button2 = gpiof.pf15.into_floating_input();
            button2.make_interrupt_source(&mut c.device.SYSCFG);
            button2.trigger_on_edge(&mut c.device.EXTI, hal::gpio::Edge::RISING_FALLING);
            button2.clear_interrupt_pending_bit();
            button2.enable_interrupt(&mut c.device.EXTI);

            UserButtons::new(button1, button2)
        };

        let leds = {
            let spi = {
                let sck = gpiob.pb13.into_alternate_af5();
                let mosi = gpiob.pb15.into_alternate_af5();

                let mode = hal::spi::Mode {
                    polarity: hal::spi::Polarity::IdleLow,
                    phase: hal::spi::Phase::CaptureOnFirstTransition,
                };

                hal::spi::Spi::spi2(
                    c.device.SPI2,
                    (sck, hal::spi::NoMiso, mosi),
                    mode,
                    10.mhz().into(),
                    clocks,
                )
            };

            let csn = gpiob.pb12.into_push_pull_output();
            let oen = gpiob.pb8.into_push_pull_output();

            UserLeds::new(spi, csn, oen)
        };

        monotonic::Tim5::new(c.device.TIM5, clocks);

        info!("Startup complete");

        // Configure a timer to periodically monitor the output channels.
        let mut monitor_timer = hal::timer::Timer::tim2(c.device.TIM2, 50.hz(), clocks);
        monitor_timer.listen(Event::TimeOut);

        // Configure a timer to periodically gather telemetry.
        let mut telemetry_timer = hal::timer::Timer::tim3(c.device.TIM3, 2.hz(), clocks);
        telemetry_timer.listen(Event::TimeOut);

        // Enable the cycle counter.
        // TODO: Replace the cycle counter monotonic with something that rolls over less. This may
        // cause errors with button press detections because the cycle counter will roll over every
        // ~22s.
        let mut cp = cortex_m::Peripherals::take().unwrap();
        cp.DWT.enable_cycle_counter();

        init::LateResources {
            monitor_timer: monitor_timer,
            telemetry_timer: telemetry_timer,
            buttons: buttons,
            leds: leds,
            channels: channels,
        }
    }

    #[task(binds=TIM2, priority=2, resources=[monitor_timer, channels, leds])]
    fn channel_monitor(c: channel_monitor::Context) {
        c.resources.monitor_timer.clear_interrupt(Event::TimeOut);

        // Check all of the timer channels.
        for channel in Channel::into_enum_iter() {
            let error_detected = match c.resources.channels.error_detected(channel) {
                Err(Error::NotPresent) => {
                    // Clear all LEDs for this channel.
                    c.resources.leds.set_led(Color::Red, channel, false);
                    c.resources.leds.set_led(Color::Yellow, channel, false);
                    c.resources.leds.set_led(Color::Green, channel, false);
                    continue;
                }
                Ok(detected) => detected,
                Err(error) => panic!("Encountered error: {:?}", error),
            };

            let warning_detected = match c.resources.channels.warning_detected(channel) {
                Ok(detected) => detected,
                Err(error) => panic!("Encountered error: {:?}", error),
            };

            let enabled = match c.resources.channels.is_enabled(channel) {
                Ok(detected) => detected,
                Err(error) => panic!("Encountered error: {:?}", error),
            };

            // Echo the measured values to the LEDs on the user interface for this channel.
            c.resources
                .leds
                .set_led(Color::Red, channel, error_detected);
            c.resources
                .leds
                .set_led(Color::Yellow, channel, warning_detected);
            c.resources.leds.set_led(Color::Green, channel, enabled);
        }

        // Propagate the updated LED values to the user interface.
        c.resources.leds.update();
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

    #[task(resources=[channels])]
    fn enable_channels(_c: enable_channels::Context) {
        for _chan in Channel::into_enum_iter() {
            //TODO: Enable all channels
            //match channels.disable_channel(chan) {
            //    Ok(_) | Err(Error::NotPresent) => {}
            //    Err(e) => panic!("Enable failed on {:?}: {:?}", chan, e),
            //}
        }
    }

    #[task(resources=[channels])]
    fn disable_channels(mut c: disable_channels::Context) {
        for chan in Channel::into_enum_iter() {
            c.resources
                .channels
                .lock(|channels| match channels.disable_channel(chan) {
                    Ok(_) | Err(Error::NotPresent) => {}
                    Err(e) => panic!("Disable failed on {:?}: {:?}", chan, e),
                })
        }
    }

    #[task(binds=EXTI4, spawn=[disable_channels, enable_channels], resources=[buttons])]
    fn button(c: button::Context) {
        if let Some(event) = c.resources.buttons.event(Instant::now()) {
            match event {
                ButtonEvent::EnableAllChannels => {
                    c.spawn.enable_channels().unwrap();
                }
                ButtonEvent::DisableChannels => {
                    c.spawn.disable_channels().unwrap();
                }
            }
        }
    }

    #[idle(resources=[buttons, channels])]
    fn idle(mut c: idle::Context) -> ! {
        loop {
            // Check if the user is requested a reset of the device.
            c.resources.buttons.lock(|buttons| {
                if buttons.check_reset(Instant::now()) {
                    cortex_m::peripheral::SCB::sys_reset();
                }
            });
        }
    }

    extern "C" {
        fn EXTI0();
        fn EXTI1();
        fn EXTI2();
        fn EXTI3();
    }
};
