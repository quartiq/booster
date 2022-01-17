//! Booster NGFW Application
//!
//! # Copyright
//! Copyright (C) 2020 QUARTIQ GmbH - All Rights Reserved
//! Unauthorized usage, editing, or copying is strictly prohibited.
//! Proprietary and confidential.
#![no_std]
#![no_main]
#![cfg_attr(feature = "unstable", feature(llvm_asm))]
#[cfg(not(any(feature = "phy_enc424j600", feature = "phy_w5500")))]
compile_error!(
    "A least one PHY device must be enabled. Use a feature gate to
    enable."
);
#[cfg(all(feature = "phy_enc424j600", feature = "phy_w5500"))]
compile_error!("Cannot enable multiple ethernet PHY devices.");

use enum_iterator::IntoEnumIterator;
use stm32f4xx_hal as hal;

#[macro_use]
extern crate log;

use panic_persist as _;

use core::fmt::Write;

use heapless::String;
use minimq::QoS;

mod delay;
mod hardware;
mod linear_transformation;
mod logger;
mod mqtt_control;
mod serial_terminal;
mod settings;
mod watchdog;

use delay::AsmDelay;
use logger::BufferedLog;
use serial_terminal::SerialTerminal;
use settings::BoosterSettings;

use hardware::{
    rf_channel::ChannelState,
    setup::{EthernetManager, MainBus},
    user_interface::{ButtonEvent, Color, UserButtons, UserLeds},
    Channel, CPU_FREQ,
};

use watchdog::{WatchdogClient, WatchdogManager};

use rtic::cyccnt::Duration;

/// An enumeration of possible errors with the device.
#[derive(Debug, Copy, Clone, serde::Serialize)]
pub enum Error {
    Invalid,
    InvalidState,
    NotPresent,
    Interface,
    Foldback,
    Bounds,
    Fault,
}

static LOGGER: BufferedLog = BufferedLog::new();

#[rtic::app(device = stm32f4xx_hal::stm32, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        main_bus: MainBus,
        buttons: UserButtons,
        leds: UserLeds,
        usb_terminal: SerialTerminal,
        eth_mgr: EthernetManager,
        watchdog: WatchdogManager,
        identifier: String<heapless::consts::U32>,
        delay: AsmDelay,
    }

    #[init(schedule = [telemetry, channel_monitor, button, usb, fans])]
    fn init(c: init::Context) -> init::LateResources {
        // Configure booster hardware.
        let booster = hardware::setup::setup(c.core, c.device);

        let watchdog_manager = WatchdogManager::new(booster.watchdog);

        let identifier: String<heapless::consts::U32> = String::from(booster.settings.id());

        // Kick-start the periodic software tasks.
        c.schedule.channel_monitor(c.start).unwrap();
        c.schedule.telemetry(c.start).unwrap();
        c.schedule.button(c.start).unwrap();
        c.schedule.usb(c.start).unwrap();
        c.schedule.fans(c.start).unwrap();

        init::LateResources {
            main_bus: booster.main_bus,
            buttons: booster.buttons,
            leds: booster.leds,
            eth_mgr: booster.ethernet,
            usb_terminal: SerialTerminal::new(
                booster.usb_device,
                booster.usb_serial,
                booster.settings,
            ),
            watchdog: watchdog_manager,
            identifier,
            delay: booster.delay,
        }
    }

    #[task(priority = 3, schedule = [channel_monitor], resources=[main_bus, leds, watchdog])]
    fn channel_monitor(c: channel_monitor::Context) {
        // Check in with the watchdog.
        c.resources.watchdog.check_in(WatchdogClient::MonitorTask);

        // Check all of the timer channels.
        for channel in Channel::into_enum_iter() {
            let state = match c.resources.main_bus.channels.map(channel, |ch, _| {
                ch.update()?;
                Ok(ch.get_state())
            }) {
                Err(Error::NotPresent) => {
                    // Clear all LEDs for this channel.
                    c.resources.leds.set_led(Color::Red, channel, false);
                    c.resources.leds.set_led(Color::Yellow, channel, false);
                    c.resources.leds.set_led(Color::Green, channel, false);
                    continue;
                }
                Err(error) => panic!("Invalid channel error: {:?}", error),
                Ok(state) => state,
            };

            let powered = match state {
                ChannelState::Powerup(_, _)
                | ChannelState::Powered
                | ChannelState::Powerdown(_)
                | ChannelState::Enabled
                | ChannelState::Tripped(_) => true,
                _ => false,
            };

            let fault = if let ChannelState::Blocked(_) = state {
                true
            } else {
                false
            };

            // RF is only enabled in the Enabled state. We also ignore the `blocked` state as this
            // is indicated by the red fault LED instead.
            let rf_disabled = match state {
                ChannelState::Enabled | ChannelState::Blocked(_) => false,
                _ => true,
            };

            // Echo the measured values to the LEDs on the user interface for this channel.
            c.resources.leds.set_led(Color::Green, channel, powered);
            c.resources
                .leds
                .set_led(Color::Yellow, channel, rf_disabled);
            c.resources.leds.set_led(Color::Red, channel, fault);
        }

        // Propagate the updated LED values to the user interface.
        c.resources.leds.update();

        // TODO: Replace hard-coded CPU cycles here.
        // Schedule to run this task periodically at 10Hz.
        c.schedule
            .channel_monitor(c.scheduled + Duration::from_cycles(CPU_FREQ / 10))
            .unwrap();
    }

    #[task(priority = 1, schedule = [fans], resources=[main_bus, watchdog])]
    fn fans(mut c: fans::Context) {
        // Check in with the watchdog.
        c.resources
            .watchdog
            .lock(|watchdog| watchdog.check_in(WatchdogClient::FanTask));

        // Determine the maximum channel temperature.
        let mut temperatures: [f32; 8] = [0.0; 8];

        for channel in Channel::into_enum_iter() {
            temperatures[channel as usize] = match c.resources.main_bus.lock(|main_bus| {
                main_bus
                    .channels
                    .map(channel, |ch, _| Ok(ch.get_temperature()))
            }) {
                Ok(temp) => temp,
                Err(Error::NotPresent) => 0.0,
                err => err.unwrap(),
            };
        }

        // Update the fan speeds.
        c.resources
            .main_bus
            .lock(|main_bus| main_bus.fans.update(temperatures));

        // TODO: Replace hard-coded CPU cycles here.
        // Schedule to run this task periodically at 1Hz.
        c.schedule
            .fans(c.scheduled + Duration::from_cycles(CPU_FREQ))
            .unwrap();
    }

    #[task(priority = 1, schedule = [telemetry], resources=[main_bus, eth_mgr, watchdog, identifier])]
    fn telemetry(mut c: telemetry::Context) {
        let id = &c.resources.identifier;

        // Check in with the watchdog.
        c.resources
            .watchdog
            .lock(|watchdog| watchdog.check_in(WatchdogClient::TelemetryTask));

        // Gather telemetry for all of the channels.
        for channel in Channel::into_enum_iter() {
            let measurements = c.resources.main_bus.lock(|main_bus| {
                main_bus
                    .channels
                    .map(channel, |ch, adc| Ok(ch.get_status(adc)))
            });

            if let Ok(measurements) = measurements {
                // Broadcast the measured data over the telemetry interface.
                let mut topic: String<heapless::consts::U32> = String::new();
                write!(&mut topic, "{}/ch{}", id, channel as u8).unwrap();

                let message: String<heapless::consts::U1024> =
                    serde_json_core::to_string(&measurements).unwrap();

                match c.resources.eth_mgr.mqtt_client.publish(
                    topic.as_str(),
                    &message.into_bytes(),
                    QoS::AtMostOnce,
                    &[],
                ) {
                    Err(e) => info!("Telemetry failure: {:?}", e),
                    Ok(_) => {}
                }
            }
        }

        // TODO: Replace hard-coded CPU cycles here.
        // Schedule to run this task periodically at 2Hz.
        c.schedule
            .telemetry(c.scheduled + Duration::from_cycles(CPU_FREQ / 2))
            .unwrap();
    }

    #[task(priority = 2, spawn=[button], schedule = [button], resources=[main_bus, buttons, watchdog])]
    fn button(mut c: button::Context) {
        // Check in with the watchdog.
        c.resources
            .watchdog
            .lock(|watchdog| watchdog.check_in(WatchdogClient::ButtonTask));

        if let Some(event) = c.resources.buttons.update() {
            match event {
                ButtonEvent::InterlockReset => {
                    for chan in Channel::into_enum_iter() {
                        c.resources.main_bus.lock(|main_bus| {
                            match main_bus.channels.map(chan, |ch, _| ch.start_powerup(true)) {
                                Ok(_) | Err(Error::NotPresent) => {}

                                // It is possible to attempt to re-enable the channel before it was
                                // fully disabled. Ignore this transient error - the user may need
                                // to press twice.
                                Err(Error::InvalidState) => {}

                                Err(e) => panic!("Reset failed on {:?}: {:?}", chan, e),
                            }
                        })
                    }
                }

                ButtonEvent::Standby => {
                    for chan in Channel::into_enum_iter() {
                        c.resources.main_bus.lock(|main_bus| {
                            match main_bus.channels.map(chan, |ch, _| Ok(ch.start_disable())) {
                                Ok(_) | Err(Error::NotPresent) => {}
                                Err(e) => panic!("Standby failed on {:?}: {:?}", chan, e),
                            }
                        })
                    }
                }
            }
        }

        // TODO: Replace hard-coded CPU cycles here.
        // Schedule to run this task every 3ms.
        c.schedule
            .button(c.scheduled + Duration::from_cycles(3 * (CPU_FREQ / 1000)))
            .unwrap();
    }

    #[task(priority = 2, schedule=[usb], resources=[usb_terminal, watchdog])]
    fn usb(mut c: usb::Context) {
        // Check in with the watchdog.
        c.resources
            .watchdog
            .lock(|watchdog| watchdog.check_in(WatchdogClient::UsbTask));

        // Process any log output.
        LOGGER.process(&mut c.resources.usb_terminal);

        // Handle the USB serial terminal.
        c.resources.usb_terminal.process();

        // TODO: Replace hard-coded CPU cycles here.
        // Schedule to run this task every 10ms.
        c.schedule
            .usb(c.scheduled + Duration::from_cycles(10 * (CPU_FREQ / 1_000)))
            .unwrap();
    }

    #[idle(resources=[main_bus, eth_mgr, watchdog, identifier, delay])]
    fn idle(mut c: idle::Context) -> ! {
        let mut manager = c
            .resources
            .identifier
            .lock(|id| mqtt_control::ControlState::new(&id));

        loop {
            // Check in with the watchdog.
            c.resources
                .watchdog
                .lock(|watchdog| watchdog.check_in(WatchdogClient::IdleTask));

            // Handle the MQTT control interface.
            manager.update(&mut c.resources);

            // TODO: Properly sleep here until there's something to process.
            cortex_m::asm::nop();
        }
    }

    extern "C" {
        fn EXTI0();
        fn EXTI1();
        fn EXTI2();
        fn EXTI3();
        fn USART1();
        fn USART2();
    }
};
