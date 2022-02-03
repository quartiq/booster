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
use stm32f4xx_hal as hal;

#[macro_use]
extern crate log;

use panic_persist as _;

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

use settings::runtime_settings::RuntimeSettings;

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

    #[init(schedule = [telemetry, channel_monitor, button, usb])]
    fn init(c: init::Context) -> init::LateResources {
        // Configure booster hardware.
        let mut booster = hardware::setup::setup(c.core, c.device);

        let mut settings = RuntimeSettings::default();

        for idx in Channel::into_enum_iter() {
            settings.channel[idx as usize] = booster
                .main_bus
                .channels
                .channel_mut(idx)
                .map(|(channel, _)| *channel.context().settings())
        }

        let watchdog_manager = WatchdogManager::new(booster.watchdog);

        booster
            .main_bus
            .fans
            .set_default_duty_cycle(settings.fan_speed);

        // Kick-start the periodic software tasks.
        c.schedule.channel_monitor(c.start).unwrap();
        c.schedule.telemetry(c.start).unwrap();
        c.schedule.button(c.start).unwrap();
        c.schedule.usb(c.start).unwrap();

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
                booster.hardware_version,
            ),
            watchdog: watchdog_manager,
        }
    }

    #[task(priority = 3, schedule = [channel_monitor], resources=[main_bus, leds, watchdog])]
    fn channel_monitor(c: channel_monitor::Context) {
        // Check in with the watchdog.
        c.resources.watchdog.check_in(WatchdogClient::Monitor);

        // Check all of the channels.
        let mut fans_enabled = false;

        let leds = c.resources.leds;
        for idx in Channel::into_enum_iter() {
            let status = c
                .resources
                .main_bus
                .channels
                .channel_mut(idx)
                .map(|(channel, _)| {
                    if channel.context().is_enabled() {
                        fans_enabled = true;
                    }

                    channel.update()
                })
                // Clear all LEDs for this channel.
                .unwrap_or_default();

            // Echo the measured values to the LEDs on the user interface for this channel.
            leds.set_led(Color::Green, idx, status.powered);
            leds.set_led(Color::Yellow, idx, status.rf_disabled);
            leds.set_led(Color::Red, idx, status.blocked);
        }

        // Update the fan speeds.
        if fans_enabled {
            c.resources.main_bus.fans.turn_on();
        } else {
            c.resources.main_bus.fans.turn_off();
        }

        // Propagate the updated LED values to the user interface.
        leds.update();

        // Schedule to run this task periodically at 10Hz.
        c.schedule
            .channel_monitor(c.scheduled + Duration::from_cycles(CPU_FREQ / 10))
            .unwrap();
    }

    #[task(priority = 1, schedule = [telemetry], resources=[main_bus, net_devices, watchdog])]
    fn telemetry(mut c: telemetry::Context) {
        // Check in with the watchdog.
        c.resources
            .watchdog
            .lock(|watchdog| watchdog.check_in(WatchdogClient::Telemetry));
        let control = &mut c.resources.net_devices.control;
        // Gather telemetry for all of the channels.
        // And broadcast the measured data over the telemetry interface.
        for idx in Channel::into_enum_iter() {
            c.resources.main_bus.lock(|main_bus| {
                main_bus.channels.channel_mut(idx).map(|(ch, adc)| {
                    control.report_telemetry(idx, &ch.context_mut().get_status(adc))
                })
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
            for idx in Channel::into_enum_iter() {
                c.resources.main_bus.lock(|main_bus| {
                    main_bus
                        .channels
                        .channel_mut(idx)
                        .map(|(channel, _)| match event {
                            ButtonEvent::InterlockReset => {
                                // It is possible to attempt to re-enable the channel before it was
                                // fully disabled. Ignore this transient error - the user may need
                                // to press twice.
                                channel.interlock_reset().ok();
                            }
                            ButtonEvent::Standby => channel.standby(),
                        })
                });
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
            c.resources.main_bus.lock(|main_bus| {
                main_bus
                    .channels
                    .channel_mut(idx)
                    .zip(all_settings.channel[idx as usize].as_ref())
                    .map(|((channel, _), settings)| {
                        channel.handle_settings(settings).unwrap_or_else(|err| {
                            log::warn!("Settings failure on {:?}: {:?}", idx, err)
                        })
                    })
            });
        }

        // Update the fan speed.
        c.resources
            .main_bus
            .lock(|main_bus| main_bus.fans.set_default_duty_cycle(all_settings.fan_speed))
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
            let mut republish = false;
            match c.resources.net_devices.lock(|net| {
                net.settings.handled_update(|path, old, new| {
                    let result = RuntimeSettings::handle_update(path, old, new);
                    if result.is_err() {
                        republish = true;
                    }
                    result
                })
            }) {
                Ok(true) => c.spawn.update_settings().unwrap(),
                Ok(false) => {}
                other => log::warn!("Miniconf update failure: {:?}", other),
            }

            if republish {
                c.resources
                    .net_devices
                    .lock(|net| net.settings.force_republish());
            }

            // Handle the MQTT control interface.
            let main_bus = &mut c.resources.main_bus;
            c.resources
                .net_devices
                .lock(|net| net.control.update(main_bus));

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
