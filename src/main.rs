#![deny(warnings)]
#![no_std]
#![no_main]

use panic_halt as _;

use cortex_m::asm;

// Use default interrupt handlers from the HAL
use stm32f4xx_hal as hal;

use hal::prelude::*;

mod booster_channels;
mod error;
mod rf_channel;
use booster_channels::BoosterChannels;

use shared_bus_rtic::{self, BusProxy};

type I2C = hal::i2c::I2c<
    hal::stm32::I2C1,
    (
        hal::gpio::gpiob::PB6<hal::gpio::AlternateOD<hal::gpio::AF4>>,
        hal::gpio::gpiob::PB7<hal::gpio::AlternateOD<hal::gpio::AF4>>,
    ),
>;

// Convenience type definition for the shared bus BusManager type.
type BusManager = shared_bus_rtic::shared_bus::BusManager<shared_bus_rtic::Mutex<I2C>, I2C>;

#[rtic::app(device = stm32f4xx_hal::stm32, peripherals = true)]
const APP: () = {
    struct Resources {
        channels: BoosterChannels,
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

        let mut delay = hal::delay::Delay::new(cp.SYST, clocks);

        let gpiob = c.device.GPIOB.split();

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
            let mux = {
                let mut i2c_mux_reset = gpiob.pb14.into_push_pull_output();
                tca9548::Tca9548::default(i2c_bus_manager.acquire(), &mut i2c_mux_reset, &mut delay)
                    .unwrap()
            };

            BoosterChannels::new(mux, &i2c_bus_manager)
        };

        init::LateResources { channels: channels }
    }

    #[idle(resources=[channels])]
    fn idle(_: idle::Context) -> ! {
        loop {
            asm::nop();
        }
    }
};
