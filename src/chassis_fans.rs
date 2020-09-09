//! Booster NGFW Application
//!
//! # Copyright
//! Copyright (C) 2020 QUARTIQ GmbH - All Rights Reserved
//! Unauthorized usage, editing, or copying is strictly prohibited.
//! Proprietary and confidential.
use super::I2cProxy;
use max6639::Max6639;

/// Provides control of the chassis-mounted cooling fans.
pub struct ChassisFans {
    fans: [Max6639<I2cProxy>; 3],
    threshold_temperature: f32,
    proportional_gain: f32,
}

impl ChassisFans {
    /// Create a new fan controller.
    ///
    /// # Args
    /// * `fans` - The fan controllers to use.
    ///
    /// # Returns
    /// A new fan controller.
    pub fn new(fans: [Max6639<I2cProxy>; 3]) -> Self {
        ChassisFans {
            fans,
            threshold_temperature: 30.0,

            // Set the Kp parameter such that fans will be at 100% speed when 30.0 *C above the
            // temperature threshold.
            proportional_gain: 1.0 / 30.0,
        }
    }

    /// Update the fans based on the current channel temperatures.
    ///
    /// # Args
    /// * `channel_temps` - The current channel temperatures in degrees celsius.
    pub fn update(&mut self, channel_temps: [f32; 8]) {
        let max_temperature =
            channel_temps.iter().fold(
                f32::NEG_INFINITY,
                |acc, &temp| {
                    if acc > temp {
                        acc
                    } else {
                        temp
                    }
                },
            );

        let temperature_error = {
            let error = max_temperature - self.threshold_temperature;
            if error > 0.0 {
                error
            } else {
                0.0
            }
        };

        // Calculate the desired duty cycle based on the temperature error.
        let duty_cycle = {
            let duty = self.proportional_gain * temperature_error;

            // Cap the duty cycle to 100%
            if duty > 1.0 {
                1.0
            } else {
                duty
            }
        };

        // Ensure that fan speed is always greater than 15% to prevent clicking.
        let final_duty = if duty_cycle > 0.0 {
            if duty_cycle < 0.15 {
                0.15
            } else {
                duty_cycle
            }
        } else {
            0.0
        };

        self.set_duty_cycles(final_duty);
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
