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
use shared_bus_rtic::{
    self,
    BusProxy
};
use stm32f4xx_hal as hal;

use hal::prelude::*;
use tca9548::Tca9548;

// Convenience type definition for the I2C bus used for booster RF channels.
type I2C = hal::i2c::I2c<
    hal::stm32::I2C1,
    (
        hal::gpio::gpiob::PB6<hal::gpio::AlternateOD<hal::gpio::AF4>>,
        hal::gpio::gpiob::PB7<hal::gpio::AlternateOD<hal::gpio::AF4>>,
    ),
>;

#[rtic::app(device = stm32f4xx_hal::stm32, peripherals = true)]
const APP: () = {
    struct Resources {
        mux: Tca9548<BusProxy<I2C>>,
    }

    #[init]
    fn init(c: init::Context) -> init::LateResources {
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

        let gpiob = c.device.GPIOB.split();

        let mut delay = hal::delay::Delay::new(cp.SYST, clocks);

        let i2c_bus_manager = {
            let i2c = {
                let scl = gpiob.pb6.into_alternate_af4_open_drain();
                let sda = gpiob.pb7.into_alternate_af4_open_drain();
                hal::i2c::I2c::i2c1(c.device.I2C1, (scl, sda), 100.khz(), clocks)
            };

            shared_bus_rtic::new!(i2c, I2C)
        };

        let mux = {
            let mut i2c_mux_reset = gpiob.pb14.into_push_pull_output();
            tca9548::Tca9548::default(i2c_bus_manager.acquire(), &mut i2c_mux_reset, &mut delay)
                .unwrap()
        };

        info!("Startup complete");

        init::LateResources { mux }
    }

    #[idle(resources=[mux])]
    fn idle(_: idle::Context) -> ! {
        loop {
            asm::nop();
        }
    }
};
