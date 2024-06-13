//! Booster NGFW Application
#![no_std]
#![no_main]

use stm32f4xx_hal as hal;

#[macro_use]
extern crate log;

use panic_persist as _;

mod hardware;
mod linear_transformation;
mod logger;
mod net;
mod settings;
mod watchdog;

use fugit::ExtU32;
use logger::BufferedLog;
use rtic_monotonics::Monotonic;
use settings::BoosterSettings;

use hardware::{
    setup::MainBus,
    usb::UsbDevice,
    user_interface::{ButtonEvent, Color, UserButtons, UserLeds},
    Channel, SerialTerminal, SystemTimer, Systick,
};

use settings::global_settings::BoosterMainBoardData;
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

#[rtic::app(device = stm32f4xx_hal::pac, dispatchers = [EXTI0, EXTI1, EXTI2, EXTI3, USART1, USART2])]
mod app {
    use super::*;

    #[shared]
    struct SharedResources {
        main_bus: MainBus,
        net_devices: net::NetworkDevices,
        watchdog: WatchdogManager,
        runtime_settings: RuntimeSettings,
    }

    #[local]
    struct LocalResources {
        buttons: UserButtons,
        leds: UserLeds,
        usb: UsbDevice,
        usb_terminal: SerialTerminal,
        global_settings: BoosterMainBoardData,
    }

    #[init]
    fn init(c: init::Context) -> (SharedResources, LocalResources) {
        // Configure booster hardware.
        let clock = SystemTimer::new(|| Systick::now().ticks());
        let mut booster = hardware::setup::setup(c.core, c.device, clock);

        let mut settings = RuntimeSettings::default();

        // Load the default fan speed
        settings.fan_speed = booster.settings.properties.fan_speed;

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
                    &booster.settings.properties.broker,
                    booster.network_stack,
                    &booster.settings.properties.id,
                    clock,
                    booster.metadata,
                ),
                watchdog: watchdog_manager,
                runtime_settings: settings,
            },
            LocalResources {
                global_settings: booster.settings.properties,
                buttons: booster.buttons,
                leds: booster.leds,
                usb: booster.usb_device,
                usb_terminal: booster.usb_serial,
            },
        )
    }

    #[task(priority = 3, local=[leds], shared=[main_bus, watchdog])]
    async fn channel_monitor(mut c: channel_monitor::Context) {
        let leds = c.local.leds;
        loop {
            // Check in with the watchdog.
            c.shared
                .watchdog
                .lock(|watchdog| watchdog.check_in(WatchdogClient::Monitor));

            // Check all of the channels.
            let mut fans_enabled = false;

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
            Systick::delay(100u32.millis()).await;
        }
    }

    #[task(priority = 1, shared=[main_bus, net_devices])]
    async fn telemetry(mut c: telemetry::Context) {
        loop {
            // Gather telemetry for all of the channels.
            // And broadcast the measured data over the telemetry interface.
            for idx in enum_iterator::all::<Channel>() {
                (&mut c.shared.main_bus, &mut c.shared.net_devices).lock(
                    |main_bus, net_devices| {
                        main_bus.channels.channel_mut(idx).map(|(ch, adc)| {
                            net_devices
                                .telemetry
                                .report_telemetry(idx, &ch.get_status(adc))
                        })
                    },
                );
            }

            let telemetry_period = c
                .shared
                .net_devices
                .lock(|net_devices| net_devices.telemetry.telemetry_period_secs());

            Systick::delay(telemetry_period.secs()).await;
        }
    }

    #[task(priority = 2, local=[buttons], shared=[main_bus, watchdog])]
    async fn button(mut c: button::Context) {
        loop {
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
            Systick::delay(3u32.millis()).await;
        }
    }

    #[task(priority = 1, shared=[net_devices, main_bus, runtime_settings])]
    async fn update_settings(mut c: update_settings::Context) {
        for idx in enum_iterator::all::<Channel>() {
            (&mut c.shared.main_bus, &mut c.shared.runtime_settings).lock(|main_bus, settings| {
                main_bus
                    .channels
                    .channel_mut(idx)
                    .zip(settings.channel[idx as usize].as_ref().as_ref())
                    .map(|((channel, _), settings)| {
                        channel.handle_settings(settings).unwrap_or_else(|err| {
                            log::warn!("Settings failure on {:?}: {:?}", idx, err)
                        })
                    })
            });
        }

        // Update the fan speed.
        (&mut c.shared.main_bus, &mut c.shared.runtime_settings)
            .lock(|main_bus, settings| main_bus.fans.set_default_duty_cycle(settings.fan_speed));

        // Update the telemetry rate.
        (&mut c.shared.net_devices, &mut c.shared.runtime_settings).lock(
            |net_devices, settings| {
                net_devices
                    .telemetry
                    .set_telemetry_period(settings.telemetry_period)
            },
        );
    }

    #[task(priority = 2, shared=[watchdog], local=[usb, usb_terminal, global_settings])]
    async fn usb(mut c: usb::Context) {
        loop {
            // Check in with the watchdog.
            c.shared
                .watchdog
                .lock(|watchdog| watchdog.check_in(WatchdogClient::Usb));

            c.local.usb.process(c.local.usb_terminal);
            c.local
                .usb_terminal
                .process(c.local.global_settings)
                .unwrap();

            // Process any log output.
            LOGGER.process(c.local.usb_terminal);

            // Schedule to run this task every 10ms.
            Systick::delay(10u32.millis()).await;
        }
    }

    #[idle(shared=[main_bus, net_devices, watchdog, runtime_settings])]
    fn idle(mut c: idle::Context) -> ! {
        loop {
            // Check in with the watchdog.
            c.shared
                .watchdog
                .lock(|watchdog| watchdog.check_in(WatchdogClient::Idle));

            // TODO: Handle out-of-range settings
            // Handle the Miniconf settings interface.
            match (&mut c.shared.net_devices, &mut c.shared.runtime_settings)
                .lock(|net, runtime_settings| net.settings_client.update(runtime_settings))
            {
                Ok(true) => {
                    update_settings::spawn().unwrap();
                }
                Ok(false) => {}
                Err(minimq::Error::Network(smoltcp_nal::NetworkError::TcpConnectionFailure(
                    smoltcp_nal::smoltcp::socket::tcp::ConnectError::Unaddressable,
                ))) => {}
                other => log::warn!("Miniconf update failure: {:?}", other),
            };

            // Handle the MQTT control interface.
            let main_bus = &mut c.shared.main_bus;
            c.shared
                .net_devices
                .lock(|net| {
                    match net.control.poll(|handler, topic, data, output| {
                        main_bus.lock(|bus| handler(bus, topic, data, output))
                    }) {
                        Err(minireq::Error::Mqtt(minireq::minimq::Error::Network(
                            smoltcp_nal::NetworkError::TcpConnectionFailure(
                                smoltcp_nal::smoltcp::socket::tcp::ConnectError::Unaddressable,
                            ),
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
