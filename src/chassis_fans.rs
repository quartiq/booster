//! Booster NGFW Application
//!
//! # Copyright
//! Copyright (C) 2020 QUARTIQ GmbH - All Rights Reserved
//! Unauthorized usage, editing, or copying is strictly prohibited.
//! Proprietary and confidential.
use super::I2C;
use max6639::Max6639;
use shared_bus_rtic::BusProxy;

/// Provides control of the chassis-mounted cooling fans.
pub struct ChassisFans {
    fans: [Max6639<BusProxy<I2C>>; 3],
}

impl ChassisFans {
    /// Create a new fan controller.
    ///
    /// # Args
    /// * `fans` - The fan controllers to use.
    ///
    /// # Returns
    /// A new fan controller.
    pub fn new(fans: [Max6639<BusProxy<I2C>>; 3]) -> Self {
        ChassisFans { fans }
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

    /// Perform a self-test of the fan operation.
    ///
    /// # Args
    /// * `delay` - An object to implement delays during the test.
    ///
    /// # Returns
    /// True if 5 of the six fans properly spun up to a high-speed RPM, 5 were below a specific
    /// RPM at 10% duty cycle, and all fans had no speed when disabled.
    pub fn self_test(
        &mut self,
        delay: &mut impl embedded_hal::blocking::delay::DelayMs<u16>,
    ) -> bool {
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
        let fans_powered_down = dead_rpms.iter().filter(|&rpms| *rpms == 0).count();

        // Check all the low RPMs are lower than 3200 RPMs.
        let fans_spun_low = low_rpms
            .iter()
            .filter(|&rpms| *rpms <= 3200 && *rpms > 0)
            .count();

        // Check all the high RPMs are higher than 4800 RPMs.
        let fans_spun_high = high_rpms.iter().filter(|&rpms| *rpms >= 4800).count();

        // If 5 fans (the count mounted on the chassis) spun up to a nominal high speed RPM, 5
        // fans were at a nominal low RPM, and 6 fans were not spinning when powered down, fans are
        // operating nominally.
        (fans_spun_high == 5) && (fans_spun_low == 5) && (fans_powered_down == 6)
    }
}
