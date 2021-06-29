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

#[macro_use]
extern crate log;

use core::fmt::Write;

use enum_iterator::IntoEnumIterator;
use heapless::String;
use minimq::QoS;
use stm32f4xx_hal as hal;

use hal::prelude::*;
use usb_device::prelude::*;

mod booster_channels;
mod chassis_fans;
mod delay;
#[cfg(feature = "phy_enc424j600")]
mod enc424j600_api;
mod error;
mod linear_transformation;
mod logger;
mod mqtt_control;
mod mutex;
mod platform;
mod rf_channel;
mod serial_terminal;
mod settings;
mod user_interface;
mod watchdog;
use booster_channels::{BoosterChannels, Channel};
use chassis_fans::ChassisFans;
use delay::AsmDelay;
use error::Error;
use logger::BufferedLog;
use rf_channel::{AdcPin, AnalogPins as AdcPins, ChannelPins as RfChannelPins, ChannelState};
use serial_terminal::SerialTerminal;
use settings::BoosterSettings;
#[cfg(feature = "phy_enc424j600")]
use smoltcp_nal::NetworkStack;
use user_interface::{ButtonEvent, Color, UserButtons, UserLeds};
use watchdog::{WatchdogClient, WatchdogManager};

use rtic::cyccnt::Duration;

// Convenience type definition for the I2C bus used for booster RF channels.
type I2C = hal::i2c::I2c<
    hal::stm32::I2C1,
    (
        hal::gpio::gpiob::PB6<hal::gpio::AlternateOD<hal::gpio::AF4>>,
        hal::gpio::gpiob::PB7<hal::gpio::AlternateOD<hal::gpio::AF4>>,
    ),
>;

type I2C2 = hal::i2c::I2c<
    hal::stm32::I2C2,
    (
        hal::gpio::gpiob::PB10<hal::gpio::AlternateOD<hal::gpio::AF4>>,
        hal::gpio::gpiob::PB11<hal::gpio::AlternateOD<hal::gpio::AF4>>,
    ),
>;

type SPI = hal::spi::Spi<
    hal::stm32::SPI1,
    (
        hal::gpio::gpioa::PA5<hal::gpio::Alternate<hal::gpio::AF5>>,
        hal::gpio::gpioa::PA6<hal::gpio::Alternate<hal::gpio::AF5>>,
        hal::gpio::gpioa::PA7<hal::gpio::Alternate<hal::gpio::AF5>>,
    ),
>;

#[cfg(feature = "phy_w5500")]
type Ethernet =
    w5500::Interface<hal::gpio::gpioa::PA4<hal::gpio::Output<hal::gpio::PushPull>>, SPI>;
#[cfg(feature = "phy_enc424j600")]
type Enc424j600 =
    enc424j600::Enc424j600<SPI, hal::gpio::gpioa::PA4<hal::gpio::Output<hal::gpio::PushPull>>>;
#[cfg(feature = "phy_enc424j600")]
type Ethernet = NetworkStack<'static, 'static, enc424j600::smoltcp_phy::SmoltcpDevice<Enc424j600>>;
#[cfg(feature = "phy_enc424j600")]
type NalClock = enc424j600_api::EpochClock<168_000_000>;
type MqttClient = minimq::MqttClient<minimq::consts::U1024, Ethernet>;

type I2cBusManager = mutex::AtomicCheckManager<I2C>;
type I2cProxy = shared_bus::I2cProxy<'static, mutex::AtomicCheckMutex<I2C>>;

type UsbBus = hal::otg_fs::UsbBus<hal::otg_fs::USB>;
type Eeprom = microchip_24aa02e48::Microchip24AA02E48<I2C2>;

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
/// * `reflected_overdrive` - The pin ID of the input overdrive pin in GPIOE.
/// * `output_overdrive` - The pin ID of the output overdrive pin in GPIOE.
/// * `signal_on` - The pin ID of the signal on pin in GPIOG.
///
/// # Returns
/// An option containing the RfChannelPins structure.
macro_rules! channel_pins {
    ($gpiod:ident, $gpioe:ident, $gpiog:ident, $enable:ident, $alert:ident, $reflected_overdrive:ident,
     $output_overdrive:ident, $signal_on:ident, $analog_pins:ident) => {{
        let enable_power = $gpiod.$enable.into_push_pull_output().downgrade();
        let alert = $gpiod.$alert.into_floating_input().downgrade();
        let reflected_overdrive = $gpioe
            .$reflected_overdrive
            .into_floating_input()
            .downgrade();
        let output_overdrive = $gpioe.$output_overdrive.into_pull_down_input().downgrade();
        let signal_on = $gpiog.$signal_on.into_push_pull_output().downgrade();

        Some(RfChannelPins::new(
            enable_power,
            alert,
            reflected_overdrive,
            output_overdrive,
            signal_on,
            $analog_pins,
        ))
    }};
}

// USB end-point memory.
static mut USB_EP_MEMORY: [u32; 1024] = [0; 1024];

static LOGGER: BufferedLog = BufferedLog::new();

/// Container method for all devices on the main I2C bus.
pub struct MainBus {
    pub channels: BoosterChannels,
    pub fans: ChassisFans,
}

/// Container method for all Ethernet-related structs.
pub struct EthernetManager {
    pub mqtt_client: MqttClient,
    #[cfg(feature = "phy_enc424j600")]
    pub nal_clock: NalClock,
}

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
    fn init(mut c: init::Context) -> init::LateResources {
        static mut USB_BUS: Option<usb_device::bus::UsbBusAllocator<UsbBus>> = None;
        static mut USB_SERIAL: Option<String<heapless::consts::U64>> = None;

        // Install the logger
        log::set_logger(&LOGGER)
            .map(|()| log::set_max_level(log::LevelFilter::Info))
            .unwrap();

        c.core.DWT.enable_cycle_counter();
        c.core.DCB.enable_trace();

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

        // Start the watchdog during the initialization process.
        let mut watchdog = hal::watchdog::IndependentWatchdog::new(c.device.IWDG);
        watchdog.start(30_000_u32.ms());

        let mut delay = AsmDelay::new(clocks.sysclk().0);

        let gpioa = c.device.GPIOA.split();
        let gpiob = c.device.GPIOB.split();
        let gpioc = c.device.GPIOC.split();
        let gpiod = c.device.GPIOD.split();
        let gpioe = c.device.GPIOE.split();
        let gpiof = c.device.GPIOF.split();
        let gpiog = c.device.GPIOG.split();

        let mut pa_ch_reset_n = gpiob.pb9.into_push_pull_output();
        pa_ch_reset_n.set_high().unwrap();

        // Manually reset all of the I2C buses across the RF channels using a bit-bang reset.
        let mut i2c_mux_reset = gpiob.pb14.into_push_pull_output();

        let i2c_bus_manager: &'static _ = {
            let mut mux = {
                let i2c = {
                    let scl = gpiob.pb6.into_alternate_af4_open_drain();
                    let sda = gpiob.pb7.into_alternate_af4_open_drain();
                    hal::i2c::I2c::i2c1(c.device.I2C1, (scl, sda), 100.khz(), clocks)
                };

                tca9548::Tca9548::default(i2c, &mut i2c_mux_reset, &mut delay).unwrap()
            };

            mux.enable(0xFF).unwrap();

            let (i2c_peripheral, pins) = mux.free().release();
            let (scl, sda) = pins;

            // Configure I2C pins as open-drain outputs.
            let mut scl = scl.into_open_drain_output();
            let mut sda = sda.into_open_drain_output();

            platform::i2c_bus_reset(&mut sda, &mut scl, &mut delay);

            let i2c = {
                let scl = scl.into_alternate_af4_open_drain();
                let sda = sda.into_alternate_af4_open_drain();
                hal::i2c::I2c::i2c1(i2c_peripheral, (scl, sda), 100.khz(), clocks)
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
                tca9548::Tca9548::default(
                    i2c_bus_manager.acquire_i2c(),
                    &mut i2c_mux_reset,
                    &mut delay,
                )
                .unwrap()
            };

            // Test scanning and reading back MUX channels.
            assert!(mux.self_test().unwrap() == true);

            let config = hal::adc::config::AdcConfig::default().reference_voltage(2500);

            let adc = hal::adc::Adc::adc3(c.device.ADC3, true, config);

            BoosterChannels::new(mux, adc, i2c_bus_manager, channel_pins, &mut delay)
        };

        let buttons = {
            let button1 = gpiof.pf14.into_floating_input();
            let button2 = gpiof.pf15.into_floating_input();
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

        info!("Startup complete");

        // Read the EUI48 identifier and configure the ethernet MAC address.
        let settings = {
            let i2c2 = {
                // Manually reset the I2C bus
                let mut scl = gpiob.pb10.into_open_drain_output();
                let mut sda = gpiob.pb11.into_open_drain_output();
                platform::i2c_bus_reset(&mut sda, &mut scl, &mut delay);

                let scl = scl.into_alternate_af4_open_drain();
                let sda = sda.into_alternate_af4_open_drain();

                hal::i2c::I2c::i2c2(c.device.I2C2, (scl, sda), 100.khz(), clocks)
            };

            let eui = microchip_24aa02e48::Microchip24AA02E48::new(i2c2).unwrap();
            BoosterSettings::new(eui)
        };

        let identifier: String<heapless::consts::U32> = String::from(settings.id());

        let eth_mgr = {
            let mqtt_client = {
                let interface = {
                    let spi = {
                        let sck = gpioa.pa5.into_alternate_af5();
                        let miso = gpioa.pa6.into_alternate_af5();
                        let mosi = gpioa.pa7.into_alternate_af5();

                        let mode = hal::spi::Mode {
                            polarity: hal::spi::Polarity::IdleLow,
                            phase: hal::spi::Phase::CaptureOnFirstTransition,
                        };

                        hal::spi::Spi::spi1(
                            c.device.SPI1,
                            (sck, miso, mosi),
                            mode,
                            #[cfg(feature = "phy_w5500")]
                            1.mhz().into(),
                            #[cfg(feature = "phy_enc424j600")]
                            14.mhz().into(),
                            clocks,
                        )
                    };

                    let cs = {
                        let mut pin = gpioa.pa4.into_push_pull_output();
                        pin.set_high().unwrap();
                        pin
                    };

                    #[cfg(feature = "phy_w5500")]
                    {
                        let mut w5500 = w5500::W5500::new(
                            spi,
                            cs,
                            w5500::OnWakeOnLan::Ignore,
                            w5500::OnPingRequest::Respond,
                            w5500::ConnectionType::Ethernet,
                            w5500::ArpResponses::Cache,
                        )
                        .unwrap();

                        w5500.set_mac(settings.mac()).unwrap();

                        // Set default netmask and gateway.
                        w5500.set_gateway(settings.gateway()).unwrap();
                        w5500.set_subnet(settings.subnet()).unwrap();
                        w5500.set_ip(settings.ip()).unwrap();

                        w5500::Interface::new(w5500)
                    }

                    #[cfg(feature = "phy_enc424j600")]
                    {
                        let enc424j600 = Enc424j600::new(spi, cs).cpu_freq_mhz(168);
                        let interface = enc424j600_api::setup(enc424j600, &settings, &mut delay);
                        interface
                    }
                };

                minimq::MqttClient::<minimq::consts::U1024, Ethernet>::new(
                    minimq::embedded_nal::IpAddr::V4(settings.broker()),
                    settings.id(),
                    interface,
                )
                .unwrap()
            };

            EthernetManager {
                mqtt_client,
                #[cfg(feature = "phy_enc424j600")]
                nal_clock: NalClock::new(),
            }
        };

        let mut fans = {
            let fan1 =
                max6639::Max6639::new(i2c_bus_manager.acquire_i2c(), max6639::AddressPin::Pulldown)
                    .unwrap();
            let fan2 =
                max6639::Max6639::new(i2c_bus_manager.acquire_i2c(), max6639::AddressPin::Float)
                    .unwrap();
            let fan3 =
                max6639::Max6639::new(i2c_bus_manager.acquire_i2c(), max6639::AddressPin::Pullup)
                    .unwrap();

            ChassisFans::new([fan1, fan2, fan3])
        };

        assert!(fans.self_test(&mut delay));

        // Set up the USB bus.
        let usb_terminal = {
            let usb = hal::otg_fs::USB {
                usb_global: c.device.OTG_FS_GLOBAL,
                usb_device: c.device.OTG_FS_DEVICE,
                usb_pwrclk: c.device.OTG_FS_PWRCLK,
                pin_dm: gpioa.pa11.into_alternate_af10(),
                pin_dp: gpioa.pa12.into_alternate_af10(),
                hclk: clocks.hclk(),
            };

            *USB_BUS = Some(hal::otg_fs::UsbBus::new(usb, unsafe { &mut USB_EP_MEMORY }));

            let usb_serial = usbd_serial::SerialPort::new(USB_BUS.as_ref().unwrap());

            // Generate a device serial number from the MAC address.
            *USB_SERIAL = {
                let mut serial_string: String<heapless::consts::U64> = String::new();

                #[cfg(feature = "phy_w5500")]
                let octets = settings.mac().octets;
                #[cfg(feature = "phy_enc424j600")]
                let octets: [u8; 6] = {
                    let mut array = [0; 6];
                    array.copy_from_slice(settings.mac().as_bytes());
                    array
                };

                write!(
                    &mut serial_string,
                    "{:02x}-{:02x}-{:02x}-{:02x}-{:02x}-{:02x}",
                    octets[0], octets[1], octets[2], octets[3], octets[4], octets[5]
                )
                .unwrap();
                Some(serial_string)
            };

            let usb_device = UsbDeviceBuilder::new(
                USB_BUS.as_ref().unwrap(),
                // TODO: Look into sub-licensing from ST.
                UsbVidPid(0x0483, 0x5740),
            )
            .manufacturer("ARTIQ/Sinara")
            .product("Booster")
            .serial_number(USB_SERIAL.as_ref().unwrap().as_str())
            .device_class(usbd_serial::USB_CLASS_CDC)
            .build();

            SerialTerminal::new(usb_device, usb_serial, settings)
        };

        info!("Startup complete");

        let watchdog_manager = WatchdogManager::new(watchdog);

        // Kick-start the periodic software tasks.
        c.schedule.channel_monitor(c.start).unwrap();
        c.schedule.telemetry(c.start).unwrap();
        c.schedule.button(c.start).unwrap();
        c.schedule.usb(c.start).unwrap();
        c.schedule.fans(c.start).unwrap();

        // Clear the reset flags now that initialization has completed.
        platform::clear_reset_flags();

        init::LateResources {
            // Note that these share a resource because they both exist on the same I2C bus.
            main_bus: MainBus { fans, channels },
            buttons: buttons,
            leds: leds,
            eth_mgr,
            usb_terminal,
            watchdog: watchdog_manager,
            identifier,
            delay: delay,
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
        // Schedule to run this task periodically at 50Hz.
        c.schedule
            .channel_monitor(c.scheduled + Duration::from_cycles(168_000_000 / 50))
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
            .fans(c.scheduled + Duration::from_cycles(168_000_000))
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
            .telemetry(c.scheduled + Duration::from_cycles(168_000_000 / 2))
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
            .button(c.scheduled + Duration::from_cycles(3 * (168_000_000 / 1000)))
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
            .usb(c.scheduled + Duration::from_cycles(10 * (168_000_000 / 1_000)))
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
