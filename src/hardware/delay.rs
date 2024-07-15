use stm32f4xx_hal::hal::delay::DelayNs;

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
