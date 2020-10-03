//! Booster NGFW Application
//!
//! # Copyright
//! Copyright (C) 2020 QUARTIQ GmbH - All Rights Reserved
//! Unauthorized usage, editing, or copying is strictly prohibited.
//! Proprietary and confidential.
use super::hal;

use embedded_hal::{blocking::delay::DelayUs, digital::v2::OutputPin};

/// Unconditionally disable and power-off all channels.
pub fn shutdown_channels() {
    let gpiod = unsafe { &*hal::stm32::GPIOD::ptr() };
    let gpiog = unsafe { &*hal::stm32::GPIOG::ptr() };

    unsafe {
        // Disable all SIG_ON outputs. Note that the upper 16 bits of this register are the ODR
        // reset bits.
        gpiog.bsrr.write(|w| w.bits(0xFF00_0000));

        // Disable all EN_PWR outputs. Note that the upper 16 bits of this register are the ODR
        // reset bits.
        gpiod.bsrr.write(|w| w.bits(0x00FF_0000));
    }
}

/// Generate a manual I2C bus reset to clear the bus.
///
/// # Args
/// * `sda` - The I2C data line.
/// * `scl` - The I2C clock line.
/// * `delay` - A means of delaying time.
pub fn i2c_bus_reset(
    sda: &mut impl OutputPin,
    scl: &mut impl OutputPin,
    delay: &mut impl DelayUs<u16>,
) {
    // Start by pulling SDA/SCL high.
    scl.set_low().ok();
    delay.delay_us(5);
    sda.set_high().ok();
    delay.delay_us(5);
    scl.set_high().ok();
    delay.delay_us(5);

    // Next, send 9 clock pulses on the I2C bus.
    for _ in 0..9 {
        scl.set_low().ok();
        delay.delay_us(5);
        scl.set_high().ok();
        delay.delay_us(5);
    }

    // Generate a stop condition by pulling SDA high while SCL is high.

    // First, get SDA into a low state without generating a start condition.
    scl.set_low().ok();
    delay.delay_us(5);
    sda.set_low().ok();
    delay.delay_us(5);

    // Next, generate the stop condition.
    scl.set_high().ok();
    delay.delay_us(5);
    sda.set_high().ok();
    delay.delay_us(5);
}

/// Check if a watchdog reset has been detected.
///
/// # Returns
/// True if a watchdog reset has been detected. False otherwise.
pub fn watchdog_detected() -> bool {
    let rcc = unsafe { &*hal::stm32::RCC::ptr() };

    rcc.csr.read().wdgrstf().bit_is_set()
}

/// Clear all of the reset flags in the device.
pub fn clear_reset_flags() {
    let rcc = unsafe { &*hal::stm32::RCC::ptr() };

    rcc.csr.modify(|_, w| w.rmvf().set_bit());
}
