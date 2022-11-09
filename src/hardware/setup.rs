//! Booster hardware setup and configuration routines.
//!
//! # Copyright
///! Copyright (C) 2020-2022 QUARTIQ GmbH

use super::{
    booster_channels::BoosterChannels,
    chassis_fans::ChassisFans,
    delay::AsmDelay,
    external_mac,
    metadata::ApplicationMetadata,
    net_interface, platform,
    rf_channel::{AdcPin, ChannelPins as RfChannelPins},
    user_interface::{UserButtons, UserLeds},
    HardwareVersion, NetworkManager, NetworkStack, SystemTimer, Systick, UsbBus, CPU_FREQ, I2C,
};

use crate::settings::BoosterSettings;

use stm32f4xx_hal as hal;

use bit_field::BitField;
use core::fmt::Write;
use hal::prelude::*;
use heapless::String;
use usb_device::prelude::*;

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
/// * `gpioa` - The GPIO port used to instantiate analog pins.
/// * `tx_power` - The name of the pin to instantiate for the TX power measurement.
/// * `reflected_power` - The name of the pin to instantiate for the reflected power measurement.
///
/// # Returns
/// An option containing the RfChannelPins structure.
macro_rules! channel_pins {
    ($gpiod:ident, $gpioe:ident, $gpiog:ident, $enable:ident, $alert:ident, $reflected_overdrive:ident,
     $output_overdrive:ident, $signal_on:ident, $gpioa:ident, $tx_power:ident, $reflected_power:ident) => {{
        let enable_power = $gpiod.$enable.into_push_pull_output().downgrade();
        let alert = $gpiod.$alert.into_floating_input().downgrade();
        let reflected_overdrive = $gpioe
            .$reflected_overdrive
            .into_floating_input()
            .downgrade();
        let output_overdrive = $gpioe.$output_overdrive.into_pull_down_input().downgrade();
        let signal_on = $gpiog.$signal_on.into_push_pull_output().downgrade();
        let tx_power = AdcPin::$tx_power($gpioa.$tx_power.into_analog());
        let reflected_power = AdcPin::$reflected_power($gpioa.$reflected_power.into_analog());

        RfChannelPins::new(
            enable_power,
            alert,
            reflected_overdrive,
            output_overdrive,
            signal_on,
            tx_power,
            reflected_power,
        )
    }};
}

/// Container method for all devices on the main I2C bus.
pub struct MainBus {
    pub channels: BoosterChannels,
    pub fans: ChassisFans,
}

/// Configured Booster hardware devices.
pub struct BoosterDevices {
    pub leds: UserLeds,
    pub buttons: UserButtons,
    pub main_bus: MainBus,
    pub network: NetworkDevices,
    pub watchdog: hal::watchdog::IndependentWatchdog,
    pub usb_device: UsbDevice<'static, UsbBus>,
    pub usb_serial: usbd_serial::SerialPort<'static, UsbBus>,
    pub settings: BoosterSettings,
    pub metadata: &'static ApplicationMetadata,
    pub systick: Systick,
}

pub struct NetworkDevices {
    pub network_stack: NetworkStack,
    pub manager: NetworkManager,
}

/// Configure Booster hardware peripherals and RF channels.
///
/// # Note
/// It is only acceptable to call this function once per boot.
///
/// # Args
/// * `core` - The RTIC core peripherals
/// * `device` - The RTIC STM32 device peripherals.
///
/// # Returns
/// The configured [BoosterDevices].
pub fn setup(
    mut core: rtic::export::Peripherals,
    device: stm32f4xx_hal::stm32::Peripherals,
    clock: SystemTimer,
) -> BoosterDevices {
    // Configure RTT logging.
    device.DBGMCU.cr.modify(|_, w| w.dbg_sleep().set_bit());
    rtt_target::rtt_init_print!();

    // Install the logger
    log::set_logger(&crate::LOGGER)
        .map(|()| log::set_max_level(log::LevelFilter::Info))
        .unwrap();

    log::info!("Starting initialization");

    core.DWT.enable_cycle_counter();
    core.DCB.enable_trace();

    // Initialize the chip
    let rcc = device.RCC.constrain();

    // Note: CPU frequency is currently set to maximum, but it could potentially be lowered if
    // needed.
    let clocks = rcc
        .cfgr
        .use_hse(8.mhz())
        .sysclk(168.mhz())
        .hclk(CPU_FREQ.hz())
        .pclk1(42.mhz())
        .require_pll48clk()
        .freeze();

    let systick = Systick::new(core.SYST, clocks.sysclk().0);

    // Start the watchdog during the initialization process.
    let mut watchdog = hal::watchdog::IndependentWatchdog::new(device.IWDG);
    watchdog.start(30_000_u32.ms());

    let mut delay = AsmDelay::new(clocks.sysclk().0);

    let gpioa = device.GPIOA.split();
    let gpiob = device.GPIOB.split();
    let gpioc = device.GPIOC.split();
    let gpiod = device.GPIOD.split();
    let gpioe = device.GPIOE.split();
    let gpiof = device.GPIOF.split();
    let gpiog = device.GPIOG.split();

    let mut pa_ch_reset_n = gpiob.pb9.into_push_pull_output();
    pa_ch_reset_n.set_high().unwrap();

    // Manually reset all of the I2C buses across the RF channels using a bit-bang reset.
    let mut i2c_mux_reset = gpiob.pb14.into_push_pull_output();

    let i2c_bus_manager: &'static _ = {
        let mut mux = {
            let i2c = {
                let scl = gpiob.pb6.into_alternate_af4_open_drain();
                let sda = gpiob.pb7.into_alternate_af4_open_drain();
                hal::i2c::I2c::i2c1(device.I2C1, (scl, sda), 100.khz(), clocks)
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

        shared_bus::new_atomic_check!(I2C = i2c).unwrap()
    };

    // Instantiate the I2C interface to the I2C mux. Use a shared-bus so we can share the I2C
    // bus with all of the Booster peripheral devices.
    let channels = {
        let pins = [
            channel_pins!(gpiod, gpioe, gpiog, pd0, pd8, pe8, pe0, pg8, gpioa, pa0, pa1),
            channel_pins!(gpiod, gpioe, gpiog, pd1, pd9, pe9, pe1, pg9, gpioa, pa2, pa3),
            channel_pins!(gpiod, gpioe, gpiog, pd2, pd10, pe10, pe2, pg10, gpiof, pf6, pf7),
            channel_pins!(gpiod, gpioe, gpiog, pd3, pd11, pe11, pe3, pg11, gpiof, pf8, pf9),
            channel_pins!(gpiod, gpioe, gpiog, pd4, pd12, pe12, pe4, pg12, gpiof, pf10, pf3),
            channel_pins!(gpiod, gpioe, gpiog, pd5, pd13, pe13, pe5, pg13, gpioc, pc0, pc1),
            channel_pins!(gpiod, gpioe, gpiog, pd6, pd14, pe14, pe6, pg14, gpioc, pc2, pc3),
            channel_pins!(gpiod, gpioe, gpiog, pd7, pd15, pe15, pe7, pg15, gpiof, pf4, pf5),
        ];

        let mut mux = {
            tca9548::Tca9548::default(
                i2c_bus_manager.acquire_i2c(),
                &mut i2c_mux_reset,
                &mut delay,
            )
            .unwrap()
        };

        // Test scanning and reading back MUX channels.
        assert!(mux.self_test().unwrap());

        let config = hal::adc::config::AdcConfig::default().reference_voltage(2500);

        let adc = hal::adc::Adc::adc3(device.ADC3, true, config);

        BoosterChannels::new(mux, adc, i2c_bus_manager, pins, clock, &mut delay)
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
                device.SPI2,
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

    // Read the EUI48 identifier and configure the ethernet MAC address.
    let settings = {
        let i2c2 = {
            // Manually reset the I2C bus
            let mut scl = gpiob.pb10.into_open_drain_output();
            let mut sda = gpiob.pb11.into_open_drain_output();
            platform::i2c_bus_reset(&mut sda, &mut scl, &mut delay);

            let scl = scl.into_alternate_af4_open_drain();
            let sda = sda.into_alternate_af4_open_drain();

            hal::i2c::I2c::i2c2(device.I2C2, (scl, sda), 100.khz(), clocks)
        };

        let eui = microchip_24aa02e48::Microchip24AA02E48::new(i2c2).unwrap();
        BoosterSettings::new(eui)
    };

    let metadata = {
        // Read the hardware version pins.
        let hardware_version = {
            let hwrev0 = gpiof.pf0.into_pull_down_input();
            let hwrev1 = gpiof.pf1.into_pull_down_input();
            let hwrev2 = gpiof.pf2.into_pull_down_input();

            HardwareVersion::from(
                *0u8.set_bit(0, hwrev0.is_high().unwrap())
                    .set_bit(1, hwrev1.is_high().unwrap())
                    .set_bit(2, hwrev2.is_high().unwrap()),
            )
        };

        ApplicationMetadata::new(hardware_version)
    };

    let (manager, network_stack) = {
        let spi = {
            let sck = gpioa.pa5.into_alternate_af5();
            let miso = gpioa.pa6.into_alternate_af5();
            let mosi = gpioa.pa7.into_alternate_af5();

            let mode = hal::spi::Mode {
                polarity: hal::spi::Polarity::IdleLow,
                phase: hal::spi::Phase::CaptureOnFirstTransition,
            };

            hal::spi::Spi::spi1(
                device.SPI1,
                (sck, miso, mosi),
                mode,
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
        let mac = {
            // Reset the W5500.
            let mut mac_reset_n = gpiog.pg5.into_push_pull_output();

            // Ensure the reset is registered.
            mac_reset_n.set_low().unwrap();
            delay.delay_ms(1u32);
            mac_reset_n.set_high().unwrap();

            // Wait for the W5500 to achieve PLL lock.
            delay.delay_ms(1u32);

            w5500::UninitializedDevice::new(w5500::bus::FourWire::new(spi, cs))
                .initialize_macraw(w5500::MacAddress {
                    octets: settings.mac().0,
                })
                .unwrap()
        };

        #[cfg(feature = "phy_enc424j600")]
        let mac = {
            let mut mac = enc424j600::Enc424j600::new(spi, cs).cpu_freq_mhz(CPU_FREQ / 1_000_000);
            mac.init(&mut delay).expect("PHY initialization failed");
            mac.write_mac_addr(settings.mac().as_bytes()).unwrap();

            mac
        };

        let (interface, manager) = external_mac::Manager::new(mac);

        let interface = net_interface::setup(interface, &settings);

        (manager, smoltcp_nal::NetworkStack::new(interface, clock))
    };

    let mut fans = {
        let main_board_leds = {
            let mut led1 = gpioc.pc8.into_push_pull_output();
            let mut led2 = gpioc.pc9.into_push_pull_output();
            let mut led3 = gpioc.pc10.into_push_pull_output();

            led1.set_low().unwrap();
            led2.set_low().unwrap();
            led3.set_low().unwrap();

            (led1, led2, led3)
        };

        let fan1 =
            max6639::Max6639::new(i2c_bus_manager.acquire_i2c(), max6639::AddressPin::Pulldown)
                .unwrap();
        let fan2 = max6639::Max6639::new(i2c_bus_manager.acquire_i2c(), max6639::AddressPin::Float)
            .unwrap();
        let fan3 =
            max6639::Max6639::new(i2c_bus_manager.acquire_i2c(), max6639::AddressPin::Pullup)
                .unwrap();

        ChassisFans::new([fan1, fan2, fan3], main_board_leds, settings.fan_speed())
    };

    assert!(fans.self_test(&mut delay));

    // Set up the USB bus.
    let (usb_device, usb_serial) = {
        // Note(unwrap): The setup function is only safe to call once, so these unwraps should never
        // fail.
        let endpoint_memory = cortex_m::singleton!(: [u32; 1024] = [0; 1024]).unwrap();
        let usb_bus =
            cortex_m::singleton!(: Option<usb_device::bus::UsbBusAllocator<UsbBus>> = None)
                .unwrap();
        let serial_number = cortex_m::singleton!(: Option<String<64>> = None).unwrap();

        let usb = hal::otg_fs::USB {
            usb_global: device.OTG_FS_GLOBAL,
            usb_device: device.OTG_FS_DEVICE,
            usb_pwrclk: device.OTG_FS_PWRCLK,
            pin_dm: gpioa.pa11.into_alternate_af10(),
            pin_dp: gpioa.pa12.into_alternate_af10(),
            hclk: clocks.hclk(),
        };

        usb_bus.replace(hal::otg_fs::UsbBus::new(usb, &mut endpoint_memory[..]));

        let usb_serial = usbd_serial::SerialPort::new(usb_bus.as_ref().unwrap());

        // Generate a device serial number from the MAC address.
        {
            let mut serial_string: String<64> = String::new();

            let octets = settings.mac().0;

            write!(
                &mut serial_string,
                "{:02x}-{:02x}-{:02x}-{:02x}-{:02x}-{:02x}",
                octets[0], octets[1], octets[2], octets[3], octets[4], octets[5]
            )
            .unwrap();
            serial_number.replace(serial_string);
        }

        let usb_device =
            // USB VID/PID registered at https://pid.codes/1209/3933/
            UsbDeviceBuilder::new(usb_bus.as_ref().unwrap(), UsbVidPid(0x1209, 0x3933))
                .manufacturer("ARTIQ/Sinara")
                .product("Booster")
                .serial_number(serial_number.as_ref().unwrap().as_str())
                .device_class(usbd_serial::USB_CLASS_CDC)
                .build();

        (usb_device, usb_serial)
    };

    info!("Startup complete");

    BoosterDevices {
        leds,
        buttons,
        // Note: These devices are within a containing structure because they exist on the same
        // shared I2C bus.
        main_bus: MainBus { channels, fans },
        network: NetworkDevices {
            network_stack,
            manager,
        },
        settings,
        usb_device,
        usb_serial,
        watchdog,
        metadata,
        systick,
    }
}
