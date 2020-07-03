#![no_std]
#![no_main]

use panic_halt as _;

use cortex_m::asm;

mod tca9548;
use tca9548::Tca9548;

// Use default interrupt handlers from the HAL
use stm32f4xx_hal as hal;

use hal::prelude::*;

type I2c = hal::i2c::I2c<hal::stm32::I2C1,
     (hal::gpio::gpiob::PB6<hal::gpio::AlternateOD<hal::gpio::AF4>>,
      hal::gpio::gpiob::PB7<hal::gpio::AlternateOD<hal::gpio::AF4>>)>;

type MutexInner = core::cell::RefCell<I2c>;
type Mutex = cortex_m::interrupt::Mutex<MutexInner>;
type BusManager = shared_bus::CortexMBusManager<MutexInner, I2c>;
type BusProxy = shared_bus::proxy::BusProxy<'static, Mutex, I2c>;

pub struct RfChannels {
    mux: Option<Tca9548<BusProxy>>,
    i2c: BusManager,
}

#[rtic::app(device = stm32f4xx_hal::stm32, peripherals = true)]
const APP: () = {
    struct Resources {
        channels: RfChannels,
    }

    #[init]
    fn init(c: init::Context) -> init::LateResources {
        // Initialize the chip
        let rcc = c.device.RCC.constrain();

        // TODO: Determine an ideal operating point for the system clock. Currently set to max.
        let clocks = rcc.cfgr
            .use_hse(8.mhz())
            .sysclk(168.mhz())
            .hclk(168.mhz())
            .require_pll48clk()
            .freeze();

        let gpiob = c.device.GPIOB.split();

        // Instantiate the I2C interface to the I2C mux. Use a shared-bus so we can share the I2C
        // bus with all of the Booster peripheral devices.
        let channels = {
            let manager = {
                let i2c = {
                    let scl = gpiob.pb6.into_alternate_af4_open_drain();
                    let sda = gpiob.pb7.into_alternate_af4_open_drain();
                    hal::i2c::I2c::i2c1(c.device.I2C1, (scl, sda), 100.khz(), clocks)
                };

                shared_bus::CortexMBusManager::new(i2c)
            };

            RfChannels { mux: None, i2c: manager }
        };

        init::LateResources {
            channels: channels,
        }
    }

    #[idle(resources=[channels])]
    fn idle(c: idle::Context) -> ! {

        c.resources.channels.mux.replace(Tca9548::default(c.resources.channels.i2c.acquire()));

        loop {
            asm::nop();
        }
    }
};
