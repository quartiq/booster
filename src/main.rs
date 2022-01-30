//! Booster NGFW Application
//!
//! # Copyright
//! Copyright (C) 2020 QUARTIQ GmbH - All Rights Reserved
//! Unauthorized usage, editing, or copying is strictly prohibited.
//! Proprietary and confidential.
#![no_std]
#![no_main]

#[cfg(not(any(feature = "phy_enc424j600", feature = "phy_w5500")))]
compile_error!(
    "A least one PHY device must be enabled. Use a feature gate to
    enable."
);
#[cfg(all(feature = "phy_enc424j600", feature = "phy_w5500"))]
compile_error!("Cannot enable multiple ethernet PHY devices.");

#[cfg(feature = "phy_enc424j600")]
compile_error!("ENC424J600 is not currently implemented");

use enum_iterator::IntoEnumIterator;
use miniconf::Miniconf;
use stm32f4xx_hal as hal;

#[macro_use]
extern crate log;

use panic_persist as _;

mod delay;
mod hardware;
mod linear_transformation;
mod logger;
mod net;
mod serial_terminal;
mod settings;
mod watchdog;

use logger::BufferedLog;
use serial_terminal::SerialTerminal;
use settings::BoosterSettings;

use hardware::{
    setup::MainBus,
    user_interface::{ButtonEvent, Color, UserButtons, UserLeds},
    Channel, CPU_FREQ,
};

use settings::channel_settings::ChannelSettings;

use watchdog::{WatchdogClient, WatchdogManager};

use rtic::cyccnt::Duration;

/// An enumeration of possible errors with the device.
#[derive(Debug, Copy, Clone, serde::Serialize)]
pub enum Error {
    Invalid,
    InvalidState,
    Interface,
    Foldback,
    Bounds,
    Fault,
}

#[derive(Default, Miniconf)]
pub struct Settings {
    pub channel: [ChannelSettings; 8],
}

static LOGGER: BufferedLog = BufferedLog::new();

#[rtic::app(device = stm32f4xx_hal::stm32, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        main_bus: MainBus,
        buttons: UserButtons,
        leds: UserLeds,
        usb_terminal: SerialTerminal,
        net_devices: net::NetworkDevices,
        watchdog: WatchdogManager,
    }

    #[init(schedule = [telemetry, channel_monitor, button, usb, fans])]
    fn init(c: init::Context) -> init::LateResources {
        // Configure booster hardware.
        let mut booster = hardware::setup::setup(c.core, c.device);

        let mut settings = Settings::default();

        for idx in Channel::into_enum_iter() {
            booster
                .main_bus
                .channels
                .channel_mut(idx)
                .map(|(channel, _)| {
                    settings.channel[idx as usize] = *channel.context_mut().settings()
                })
                .unwrap_or_else(|| {
                    settings.channel[idx as usize].enabled = false;
                    settings.channel[idx as usize].rf_disable = true;
                    settings.channel[idx as usize].bias_voltage = 0.0;
                });
        }

        let watchdog_manager = WatchdogManager::new(booster.watchdog);

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
            net_devices: net::NetworkDevices::new(
                minimq::embedded_nal::IpAddr::V4(booster.settings.broker()),
                booster.network_stack,
                booster.settings.id(),
                settings,
            ),
            usb_terminal: SerialTerminal::new(
                booster.usb_device,
                booster.usb_serial,
                booster.settings,
            ),
            watchdog: watchdog_manager,
        }
    }

    #[task(priority = 3, schedule = [channel_monitor], resources=[main_bus, leds, watchdog])]
    fn channel_monitor(c: channel_monitor::Context) {
        // Check in with the watchdog.
        c.resources.watchdog.check_in(WatchdogClient::Monitor);
        // Check all of the timer channels.
        let leds = c.resources.leds;
        for idx in Channel::into_enum_iter() {
            let status = c
                .resources
                .main_bus
                .channels
                .channel_mut(idx)
                .map(|(channel, _)| channel.update())
                // Clear all LEDs for this channel.
                .unwrap_or_default();
            // Echo the measured values to the LEDs on the user interface for this channel.
            leds.set_led(Color::Green, idx, status.powered);
            leds.set_led(Color::Yellow, idx, status.rf_disabled);
            leds.set_led(Color::Red, idx, status.blocked);
        }
        // Propagate the updated LED values to the user interface.
        leds.update();

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
            .lock(|watchdog| watchdog.check_in(WatchdogClient::Fan));

        // Determine the maximum channel temperature.
        let mut temperatures: [Option<f32>; 8] = [None; 8];

        for idx in Channel::into_enum_iter() {
            temperatures[idx as usize] = c.resources.main_bus.lock(|main_bus| {
                main_bus
                    .channels
                    .channel_mut(idx)
                    .map(|(channel, _)| channel.context_mut().get_temperature())
            });
        }

        // Update the fan speeds.
        c.resources
            .main_bus
            .lock(|main_bus| main_bus.fans.update(temperatures));

        // Schedule to run this task periodically at 1Hz.
        c.schedule
            .fans(c.scheduled + Duration::from_cycles(CPU_FREQ))
            .unwrap();
    }

    #[task(priority = 1, schedule = [telemetry], resources=[main_bus, net_devices, watchdog])]
    fn telemetry(mut c: telemetry::Context) {
        // Check in with the watchdog.
        c.resources
            .watchdog
            .lock(|watchdog| watchdog.check_in(WatchdogClient::Telemetry));
        let tele = &mut c.resources.net_devices.telemetry;
        // Gather telemetry for all of the channels.
        for idx in Channel::into_enum_iter() {
            c.resources.main_bus.lock(|main_bus| {
                main_bus
                    .channels
                    .channel_mut(idx)
                    .map(|(ch, adc)| tele.report_telemetry(idx, &ch.context_mut().get_status(adc)))
            });
        }

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
            .lock(|watchdog| watchdog.check_in(WatchdogClient::Button));

        if let Some(event) = c.resources.buttons.update() {
            match event {
                ButtonEvent::InterlockReset => {
                    for idx in Channel::into_enum_iter() {
                        c.resources.main_bus.lock(|main_bus| {
                            main_bus.channels.channel_mut(idx).map(|(channel, _)|
                                // It is possible to attempt to re-enable the channel before it was
                                // fully disabled. Ignore this transient error - the user may need
                                // to press twice.
                                channel.interlock_reset().ok())
                        });
                    }
                }

                ButtonEvent::Standby => {
                    for idx in Channel::into_enum_iter() {
                        c.resources.main_bus.lock(|main_bus| {
                            main_bus
                                .channels
                                .channel_mut(idx)
                                .map(|(channel, _)| channel.standby())
                        });
                    }
                }
            }
        }

        // Schedule to run this task every 3ms.
        c.schedule
            .button(c.scheduled + Duration::from_cycles(3 * (CPU_FREQ / 1000)))
            .unwrap();
    }

    #[task(priority = 1, resources=[net_devices, main_bus])]
    fn update_settings(mut c: update_settings::Context) {
        let all_settings = c.resources.net_devices.settings.settings();

        for idx in Channel::into_enum_iter() {
            let settings = &all_settings.channel[idx as usize];
            c.resources.main_bus.lock(|main_bus| {
                main_bus.channels.channel_mut(idx).map(|(channel, _)| {
                    channel.handle_settings(settings).unwrap_or_else(|err| {
                        log::warn!("Settings failure on {:?}: {:?}", idx, err)
                    })
                })
            });
        }
    }

    #[task(priority = 2, schedule=[usb], resources=[usb_terminal, watchdog])]
    fn usb(mut c: usb::Context) {
        // Check in with the watchdog.
        c.resources
            .watchdog
            .lock(|watchdog| watchdog.check_in(WatchdogClient::Usb));

        // Process any log output.
        LOGGER.process(c.resources.usb_terminal);

        // Handle the USB serial terminal.
        c.resources.usb_terminal.process();

        // Schedule to run this task every 10ms.
        c.schedule
            .usb(c.scheduled + Duration::from_cycles(10 * (CPU_FREQ / 1_000)))
            .unwrap();
    }

    #[idle(resources=[main_bus, net_devices, watchdog], spawn=[update_settings])]
    fn idle(mut c: idle::Context) -> ! {
        loop {
            // Check in with the watchdog.
            c.resources
                .watchdog
                .lock(|watchdog| watchdog.check_in(WatchdogClient::Idle));

            // Handle the Miniconf settings interface.
            match c.resources.net_devices.lock(|net| net.settings.update()) {
                Ok(true) => c.spawn.update_settings().unwrap(),
                Ok(false) => {}
                other => log::warn!("Miniconf update failure: {:?}", other),
            }

            // Handle the MQTT control interface.
            let main_bus = &mut c.resources.main_bus;
            c.resources
                .net_devices
                .lock(|net| net.controller.update(main_bus));

            // Handle the network stack processing if needed.
            c.resources.net_devices.lock(|net| net.process());
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
