//! Booster NGFW Application

use super::{I2cError, I2cProxy, MainboardLeds};
use max6639::Max6639;

/// The default fan speed on power-up.
pub const DEFAULT_FAN_SPEED: f32 = 0.2;

/// Provides control of the chassis-mounted cooling fans.
pub struct ChassisFans {
    fans: [Max6639<I2cProxy>; 3],
    duty_cycle: f32,
    leds: MainboardLeds,
}

impl ChassisFans {
    /// Create a new fan controller.
    ///
    /// # Args
    /// * `fans` - The fan controllers to use.
    /// * `leds` - The LEDs on Booster's main board.
    /// * `duty_cycle` - The default (normalized) duty cycle to use when enabling fans.
    ///
    /// # Returns
    /// A new fan controller.
    pub fn new(fans: [Max6639<I2cProxy>; 3], leds: MainboardLeds, default_speed: f32) -> Self {
        ChassisFans {
            fans,
            duty_cycle: default_speed.clamp(0.0, 1.0),
            leds,
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

        let leds = &mut self.leds;

        // Keep retrying until the configuration succeeds or the maximum number of retry
        // attempts is exhausted.
        let mut retry_set = |fan: &mut Max6639<I2cProxy>, subfan, duty_cycle| {
            for _ in 0..2 {
                match fan.set_duty_cycle(subfan, duty_cycle) {
                    Err(max6639::Error::Interface(embedded_hal_bus::i2c::AtomicError::Other(
                        I2cError::NoAcknowledge(_),
                    ))) => {
                        leds.0.set_high();
                        leds.1.set_high();
                        leds.2.set_high();
                    }
                    Ok(_) => {
                        leds.0.set_low();
                        leds.1.set_low();
                        leds.2.set_low();
                    }
                    Err(e) => panic!("{:?}", e),
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
    pub fn self_test(&mut self, delay: &mut impl stm32f4xx_hal::hal::delay::DelayNs) -> bool {
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
