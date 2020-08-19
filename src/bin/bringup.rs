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

// Convenience type definition for the I2C bus used for booster RF channels.
type I2C = hal::i2c::I2c<
    hal::stm32::I2C1,
    (
        hal::gpio::gpiob::PB6<hal::gpio::AlternateOD<hal::gpio::AF4>>,
        hal::gpio::gpiob::PB7<hal::gpio::AlternateOD<hal::gpio::AF4>>,
    ),
>;

struct FanTest {
    fans: [max6639::Max6639<BusProxy<I2C>>; 3],
}

impl FanTest {
    pub fn new(fans: [max6639::Max6639<BusProxy<I2C>>; 3]) -> Self {
        FanTest { fans }
    }

    fn set_duty_cycles(&mut self, duty_cycle: f32) {
        for fan in self.fans.iter_mut() {
            fan.set_duty_cycle(max6639::Fan::Fan1, duty_cycle).unwrap();
            fan.set_duty_cycle(max6639::Fan::Fan2, duty_cycle).unwrap();
        }
    }

    fn read_rpms(&mut self) -> [u16; 6] {
        let mut rpms: [u16; 6] = [0; 6];
        rpms[0] = self.fans[0].current_rpms(max6639::Fan::Fan1).unwrap();
        rpms[1] = self.fans[0].current_rpms(max6639::Fan::Fan2).unwrap();
        rpms[2] = self.fans[1].current_rpms(max6639::Fan::Fan1).unwrap();
        rpms[3] = self.fans[1].current_rpms(max6639::Fan::Fan2).unwrap();
        rpms[4] = self.fans[2].current_rpms(max6639::Fan::Fan1).unwrap();
        rpms[5] = self.fans[2].current_rpms(max6639::Fan::Fan2).unwrap();
        rpms
    }

    pub fn test(&mut self, delay: &mut impl embedded_hal::blocking::delay::DelayMs<u16>) {
        delay.delay_ms(7000);
        let dead_rpms = self.read_rpms();

        self.set_duty_cycles(0.1);
        delay.delay_ms(7000);
        let low_rpms = self.read_rpms();

        self.set_duty_cycles(1.0);
        delay.delay_ms(2000);
        let high_rpms = self.read_rpms();

        self.set_duty_cycles(0.0);

        // Check that all dead RPMS are zero.
        assert!(dead_rpms.iter().fold(0, |count, rpms| {
            if *rpms == 0 {
                count + 1
            } else {
                count
            }
        }) == 6);

        // Check all the low RPMs are lower than 3200 RPMs.
        assert!(low_rpms.iter().fold(0, |count, rpms| {
            if *rpms <= 3200 {
                count + 1
            } else {
                count
            }
        }) >= 5);

        // Check all the high RPMs are higher than 4800 RPMs.
        assert!(high_rpms.iter().fold(0, |count, rpms| {
            if *rpms >= 4800 {
                count + 1
            } else {
                count
            }
        }) == 5);
    }
}

#[rtic::app(device = stm32f4xx_hal::stm32, peripherals = true)]
const APP: () = {
    struct Resources {}

    #[init]
    fn init(c: init::Context) {
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

        let mut pa_ch_reset_n = gpiob.pb9.into_push_pull_output();
        pa_ch_reset_n.set_high().unwrap();

        let i2c_bus_manager = {
            let i2c = {
                let scl = gpiob.pb6.into_alternate_af4_open_drain();
                let sda = gpiob.pb7.into_alternate_af4_open_drain();
                hal::i2c::I2c::i2c1(c.device.I2C1, (scl, sda), 100.khz(), clocks)
            };

            shared_bus_rtic::new!(i2c, I2C)
        };

        let mut mux = {
            let mut i2c_mux_reset = gpiob.pb14.into_push_pull_output();
            tca9548::Tca9548::default(i2c_bus_manager.acquire(), &mut i2c_mux_reset, &mut delay)
                .unwrap()
        };

        // Test scanning and reading back MUX channels.
        assert!(mux.self_test().unwrap() == true);

        let i2c2 = {
            let scl = gpiob.pb10.into_alternate_af4_open_drain();
            let sda = gpiob.pb11.into_alternate_af4_open_drain();
            hal::i2c::I2c::i2c2(c.device.I2C2, (scl, sda), 100.khz(), clocks)
        };

        // Read the EUI48 identifier.
        let mut eui = microchip_24aa02e48::Microchip24AA02E48::new(i2c2).unwrap();
        let mut eui48: [u8; 6] = [0; 6];
        eui.read_eui48(&mut eui48).unwrap();

        for channel in 0..8 {
            mux.enable(1 << channel).unwrap();

            // Set the DAC bias voltage.
            let mut dac = dac7571::Dac7571::default(i2c_bus_manager.acquire());
            dac.set_voltage(0.0).unwrap();

            let mut monitor = ads7924::Ads7924::default(i2c_bus_manager.acquire(), &mut delay).unwrap();
            monitor
                .set_thresholds(ads7924::Channel::Three, 0.0, 1.0)
                .unwrap();

            // Verify the ALARM is set.
            delay.delay_ms(10u8);
            assert!(monitor.clear_alarm().unwrap() != 0);

            monitor
                .set_thresholds(ads7924::Channel::Three, 1.0, 2.5)
                .unwrap();

            // Verify the ALARM is not set.
            delay.delay_ms(10u8);
            assert!(monitor.clear_alarm().unwrap() == 0);
        }

        let fan1 =
            max6639::Max6639::new(i2c_bus_manager.acquire(), max6639::AddressPin::Pulldown)
                .unwrap();
        let fan2 =
            max6639::Max6639::new(i2c_bus_manager.acquire(), max6639::AddressPin::Float).unwrap();
        let fan3 =
            max6639::Max6639::new(i2c_bus_manager.acquire(), max6639::AddressPin::Pullup).unwrap();

        let mut fan_test = FanTest::new([fan1, fan2, fan3]);
        fan_test.test(&mut delay);

        // Reset here.
        cortex_m::peripheral::SCB::sys_reset();
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            asm::nop();
        }
    }
};
