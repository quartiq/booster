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
    threshold_temperature: f32,
    proportional_gain: f32,
    fans_spinning: bool,
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

            fans_spinning: false,
        }
    }

    /// Update the fans based on the current channel temperatures.
    ///
    /// # Args
    /// * `channel_temps` - The current channel temperatures in degrees celsius.
    pub fn update(&mut self, channel_temps: [f32; 8]) {
        // Calculate the maximum temperature encountered across all of the channels.
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

        // Determine the maximum temperature error from the hottest channel to use in the fan
        // control algorithm.
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

        // Only turn on the fans if temperature has been exceeded by at least 3 degrees. Fan speeds
        // at below 20% can cause clicking. This algorithm ensures that fans will turn on once
        // temperatures reach 34.5 *C (with 30 *C threshold and 30 *C range to 100% cycle) and will
        // not disable until temperature falls below 30 *C (while maintaining at least 20% duty
        // cycle).

        // This algorithm is intended to prevent issues with the fan turning on/off constantly when
        // right at the temperature threshold.
        let final_duty = if temperature_error > 3.0 {
            // When the temperature is above threshold, turn on the fans to at least 20% duty cycle.
            if duty_cycle > 0.20 {
                duty_cycle
            } else {
                0.20
            }
        } else if temperature_error > 0.01 && self.fans_spinning {
            // If the temperature error is above 0 and the fans are currently spinning, we
            // need to continue driving the fans at least a minimum drive to prevent clicking.
            // Comparison is done to 0.01 degrees to avoid floating-point comparison issues.
            0.20
        } else {
            0.0
        };

        self.set_duty_cycles(final_duty);
    }

    fn set_duty_cycles(&mut self, duty_cycle: f32) {
        // Keep retrying until the configuration succeeds or the maximum number of retry
        // attempts is exhausted.
        let retry_set = |fan: &mut Max6639<I2cProxy>, subfan, duty_cycle| -> () {
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

        // Deem the fans to be spinning if the duty cycle is greater than 5%. This is to avoid
        // floating-point comparison errors at near-zero duty cycle. The driver should not be
        // configuring fans to a non-zero duty cycle below 20%.
        self.fans_spinning = duty_cycle > 0.05;
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
