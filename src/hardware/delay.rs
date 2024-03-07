use stm32f4xx_hal::{hal::delay::DelayNs, hal_02::blocking::delay::DelayUs};

#[derive(Clone)]
pub struct AsmDelay {
    frequency_us: u32,
}

impl AsmDelay {
    pub fn new(freq: u32) -> AsmDelay {
        AsmDelay {
            frequency_us: (freq / 1_000_000),
        }
    }
}

impl DelayNs for AsmDelay {
    fn delay_ns(&mut self, ns: u32) {
        cortex_m::asm::delay(self.frequency_us * (ns / 1000))
    }
}

impl<U> DelayUs<U> for AsmDelay
where
    U: Into<u32>,
{
    fn delay_us(&mut self, us: U) {
        cortex_m::asm::delay(self.frequency_us * us.into())
    }
}
