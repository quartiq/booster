//! Booster NGFW Application
#![no_std]
#![no_main]

#[cfg(not(any(feature = "phy_enc424j600", feature = "phy_w5500")))]
compile_error!(
    "A least one PHY device must be enabled. Use a feature gate to
    enable."
);
#[cfg(all(feature = "phy_enc424j600", feature = "phy_w5500"))]
compile_error!("Cannot enable multiple ethernet PHY devices.");

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
use systick_monotonic::fugit::ExtU64;

use hardware::{
    setup::MainBus,
    user_interface::{ButtonEvent, Color, UserButtons, UserLeds},
    Channel, SystemTimer,
};

use settings::runtime_settings::RuntimeSettings;
use watchdog::{WatchdogClient, WatchdogManager};

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

#[rtic::app(device = stm32f4xx_hal::stm32, dispatchers = [EXTI0, EXTI1, EXTI2, EXTI3, USART1, USART2])]
mod app {
    use super::*;

    #[shared]
    struct SharedResources {
        main_bus: MainBus,
        net_devices: net::NetworkDevices,
        watchdog: WatchdogManager,
    }

    #[local]
    struct LocalResources {
        buttons: UserButtons,
        leds: UserLeds,
        usb_terminal: SerialTerminal,
    }

    #[monotonic(binds = SysTick, default = true, priority = 4)]
    type Monotonic = hardware::Systick;

    #[init]
    fn init(c: init::Context) -> (SharedResources, LocalResources, init::Monotonics) {
        // Configure booster hardware.
        let clock = SystemTimer::new(|| monotonics::now().ticks() as u32);
        let mut booster = hardware::setup::setup(c.core, c.device, clock);

        let mut settings = RuntimeSettings::default();

        // Load the default fan speed
        settings.fan_speed = booster.settings.fan_speed();

        for idx in enum_iterator::all::<Channel>() {
            settings.channel[idx as usize] = booster
                .main_bus
                .channels
                .channel_mut(idx)
                .map(|(channel, _)| *channel.context().settings())
        }

        let watchdog_manager = WatchdogManager::new(booster.watchdog);

        // Kick-start the periodic software tasks.
        channel_monitor::spawn().unwrap();
        telemetry::spawn().unwrap();
        button::spawn().unwrap();
        usb::spawn().unwrap();

        (
            SharedResources {
                main_bus: booster.main_bus,
                net_devices: net::NetworkDevices::new(
                    minimq::embedded_nal::IpAddr::V4(booster.settings.broker()),
                    booster.network,
                    booster.settings.id(),
                    settings,
                    clock,
                    booster.metadata,
                ),
                watchdog: watchdog_manager,
            },
            LocalResources {
                buttons: booster.buttons,
                leds: booster.leds,
                usb_terminal: SerialTerminal::new(
                    booster.usb_device,
                    booster.usb_serial,
                    booster.settings,
                    booster.metadata,
                ),
            },
            init::Monotonics(booster.systick),
        )
    }

    #[task(priority = 3, local=[leds], shared=[main_bus, watchdog])]
    fn channel_monitor(mut c: channel_monitor::Context) {
        // Check in with the watchdog.
        c.shared
            .watchdog
            .lock(|watchdog| watchdog.check_in(WatchdogClient::Monitor));

        // Check all of the channels.
        let mut fans_enabled = false;

        let leds = c.local.leds;
        for idx in enum_iterator::all::<Channel>() {
            let status = c.shared.main_bus.lock(|main_bus| {
                main_bus
                    .channels
                    .channel_mut(idx)
                    .map(|(channel, _)| {
                        if channel.context().is_powered() {
                            fans_enabled = true;
                        }

                        channel.update()
                    })
                    // Clear all LEDs for this channel.
                    .unwrap_or_default()
            });

            // Echo the measured values to the LEDs on the user interface for this channel.
            leds.set_led(Color::Green, idx, status.powered);
            leds.set_led(Color::Yellow, idx, status.rf_disabled);
            leds.set_led(Color::Red, idx, status.blocked);
        }

        // Update the fan speeds.
        if fans_enabled {
            c.shared.main_bus.lock(|main_bus| main_bus.fans.turn_on());
        } else {
            c.shared.main_bus.lock(|main_bus| main_bus.fans.turn_off());
        }

        // Propagate the updated LED values to the user interface.
        leds.update();

        // Schedule to run this task periodically at 10Hz.
        channel_monitor::spawn_after(100u64.millis()).unwrap();
    }

    #[task(priority = 1, shared=[main_bus, net_devices])]
    fn telemetry(mut c: telemetry::Context) {
        // Gather telemetry for all of the channels.
        // And broadcast the measured data over the telemetry interface.
        for idx in enum_iterator::all::<Channel>() {
            (&mut c.shared.main_bus, &mut c.shared.net_devices).lock(|main_bus, net_devices| {
                main_bus.channels.channel_mut(idx).map(|(ch, adc)| {
                    net_devices
                        .telemetry
                        .report_telemetry(idx, &ch.get_status(adc))
                })
            });
        }

        let telemetry_period = c
            .shared
            .net_devices
            .lock(|net_devices| net_devices.telemetry.telemetry_period_secs());

        telemetry::spawn_after(telemetry_period.secs()).unwrap();
    }

    #[task(priority = 2, local=[buttons], shared=[main_bus, watchdog])]
    fn button(mut c: button::Context) {
        // Check in with the watchdog.
        c.shared
            .watchdog
            .lock(|watchdog| watchdog.check_in(WatchdogClient::Button));

        if let Some(event) = c.local.buttons.update() {
            for idx in enum_iterator::all::<Channel>() {
                c.shared.main_bus.lock(|main_bus| {
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
        button::spawn_after(3u64.millis()).unwrap();
    }

    #[task(priority = 1, shared=[net_devices, main_bus])]
    fn update_settings(mut c: update_settings::Context) {
        let all_settings = c
            .shared
            .net_devices
            .lock(|net_devices| net_devices.settings.settings().clone());

        for idx in enum_iterator::all::<Channel>() {
            c.shared.main_bus.lock(|main_bus| {
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
        c.shared
            .main_bus
            .lock(|main_bus| main_bus.fans.set_default_duty_cycle(all_settings.fan_speed));

        // Update the telemetry rate.
        c.shared.net_devices.lock(|net_devices| {
            net_devices
                .telemetry
                .set_telemetry_period(all_settings.telemetry_period)
        });
    }

    #[task(priority = 2, shared=[watchdog], local=[usb_terminal])]
    fn usb(mut c: usb::Context) {
        // Check in with the watchdog.
        c.shared
            .watchdog
            .lock(|watchdog| watchdog.check_in(WatchdogClient::Usb));

        // Process any log output.
        LOGGER.process(c.local.usb_terminal);

        // Handle the USB serial terminal.
        c.local.usb_terminal.process();

        // Schedule to run this task every 10ms.
        usb::spawn_after(10u64.millis()).unwrap();
    }

    #[idle(shared=[main_bus, net_devices, watchdog])]
    fn idle(mut c: idle::Context) -> ! {
        loop {
            // Check in with the watchdog.
            c.shared
                .watchdog
                .lock(|watchdog| watchdog.check_in(WatchdogClient::Idle));

            // Handle the Miniconf settings interface.
            let mut republish = false;
            match c.shared.net_devices.lock(|net| {
                net.settings.handled_update(|path, old, new| {
                    let result = RuntimeSettings::handle_update(path, old, new);
                    if result.is_err() {
                        republish = true;
                    }
                    result
                })
            }) {
                Ok(true) => update_settings::spawn().unwrap(),
                Ok(false) => {}
                Err(miniconf::minimq::Error::Network(smoltcp_nal::NetworkError::NoIpAddress)) => {}
                other => log::warn!("Miniconf update failure: {:?}", other),
            }

            if republish {
                c.shared
                    .net_devices
                    .lock(|net| net.settings.force_republish());
            }

            // Handle the MQTT control interface.
            let main_bus = &mut c.shared.main_bus;
            c.shared
                .net_devices
                .lock(|net| {
                    match net
                        .control
                        .poll(|handler, topic, data| main_bus.lock(|bus| handler(bus, topic, data)))
                    {
                        Err(minireq::Error::Mqtt(minimq::Error::Network(
                            smoltcp_nal::NetworkError::NoIpAddress,
                        ))) => Ok(()),
                        other => other,
                    }
                })
                .unwrap();

            // Handle the network stack processing if needed.
            c.shared.net_devices.lock(|net| net.process());
        }
    }
}
