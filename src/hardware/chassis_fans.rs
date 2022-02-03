//! Booster NGFW Application
//!
//! # Copyright
//! Copyright (C) 2020 QUARTIQ GmbH - All Rights Reserved
//! Unauthorized usage, editing, or copying is strictly prohibited.
//! Proprietary and confidential.
use super::{I2cError, I2cProxy};
use max6639::Max6639;

/// Provides control of the chassis-mounted cooling fans.
pub struct ChassisFans {
    fans: [Max6639<I2cProxy>; 3],
    duty_cycle: f32,
}

impl ChassisFans {
    /// Create a new fan controller.
    ///
    /// # Args
    /// * `fans` - The fan controllers to use.
    /// * `duty_cycle` - The default (normalized) duty cycle to use when enabling fans.
    ///
    /// # Returns
    /// A new fan controller.
    pub fn new(fans: [Max6639<I2cProxy>; 3]) -> Self {
        ChassisFans {
            fans,
            duty_cycle: 0.2,
        }
    }

    /// Configure the duty cycle that fans operate at.
    ///
    /// # Args
    /// * `duty_cycle` - The default duty cycle to use when enabling fans.
    pub fn set_default_duty_cycle(&mut self, duty_cycle: f32) {
        self.duty_cycle = duty_cycle.clamp(0.0, 1.0);
    }

    /// Enable all fans.
    pub fn turn_on(&mut self) {
        self.set_duty_cycles(self.duty_cycle)
    }

    /// Turn off fans.
    pub fn turn_off(&mut self) {
        self.set_duty_cycles(0.0)
    }

    /// Set the duty cycle of the fans.
    ///
    /// # Args
    /// * `duty_cycle` - The normalized desired duty cycle of fans. Will be bounded to [0, 1].
    fn set_duty_cycles(&mut self, duty_cycle: f32) {
        // Bound the duty cycle to a normalized range.
        let duty_cycle = duty_cycle.clamp(0.0, 1.0);

        // Keep retrying until the configuration succeeds or the maximum number of retry
        // attempts is exhausted.
        let retry_set = |fan: &mut Max6639<I2cProxy>, subfan, duty_cycle| {
            for _ in 0..2 {
                match fan.set_duty_cycle(subfan, duty_cycle) {
                    Err(max6639::Error::Interface(I2cError::NACK)) => {}
                    Err(e) => Err(e).unwrap(),
                    Ok(_) => break,
                }
            }
        };

        for fan in self.fans.iter_mut() {
            retry_set(fan, max6639::Fan::Fan1, duty_cycle);
            retry_set(fan, max6639::Fan::Fan2, duty_cycle);
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
    /// True if 5 of the six fans properly spun up to a high-speed RPM and all fans had no speed
    /// when disabled.
    pub fn self_test(
        &mut self,
        delay: &mut impl embedded_hal::blocking::delay::DelayMs<u16>,
    ) -> bool {
        self.set_duty_cycles(1.0);
        delay.delay_ms(5000);
        let high_rpms = self.read_rpms();

        self.set_duty_cycles(0.0);
        delay.delay_ms(7000);
        let dead_rpms = self.read_rpms();

        // Check that all dead RPMS are zero.
        let fans_powered_down = dead_rpms.iter().filter(|&rpms| *rpms == 0).count();

        // Check all the high RPMs are higher than 4800 RPMs.
        let fans_spun_high = high_rpms.iter().filter(|&rpms| *rpms >= 4800).count();

        // If 5 fans (the count mounted on the chassis) spun up to a nominal high speed RPM and 6
        // fans were not spinning when powered down, fans are operating nominally.
        (fans_spun_high == 5) && (fans_powered_down == 6)
    }
}
