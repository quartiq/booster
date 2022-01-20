use minimq::embedded_time::{
    clock::{Clock, Error},
    fraction::Fraction,
    Instant,
};

use core::convert::TryInto;

#[derive(Copy, Clone, Debug, Default)]
pub struct SystemTimer {}

impl SystemTimer {
    pub fn initialize(regs: stm32f4xx_hal::stm32::TIM2, clocks: &stm32f4xx_hal::rcc::Clocks) {
        // Reset and enable the timer in the RCC.
        let rcc = unsafe { &*stm32f4xx_hal::stm32::RCC::ptr() };
        rcc.apb1rstr.modify(|_, w| w.tim2rst().set_bit());
        rcc.apb1enr.modify(|_, w| w.tim2en().set_bit());

        // Stop the counter and reset it.
        regs.cr1.modify(|_, w| w.cen().clear_bit());
        regs.cnt.reset();

        // Get the frequency of the peripheral source clock.
        let frequency = {
            // TIM2 is on APB1.
            let pclk_mul = if clocks.ppre1() == 1 { 1 } else { 2 };
            clocks.pclk1().0 * pclk_mul
        };

        // Configure the timer for a frequency of 10KHz, max count.
        let prescaler: u16 = (frequency / 10_000 - 1).try_into().unwrap();
        regs.psc.write(|w| w.psc().bits(prescaler));
        regs.arr.reset();

        // Trigger an update.
        regs.egr.write(|w| w.ug().set_bit());

        // Enable the counter.
        regs.cr1.modify(|_, w| w.cen().set_bit());
    }
}

impl Clock for SystemTimer {
    type T = u32;

    const SCALING_FACTOR: Fraction = Fraction::new(1, 10_000);

    fn try_now(&self) -> Result<Instant<Self>, Error> {
        let regs = unsafe { &*stm32f4xx_hal::stm32::TIM2::ptr() };

        // The TIM2 peripheral is a 32-bit counter.
        Ok(Instant::new(regs.cnt.read().bits()))
    }
}
