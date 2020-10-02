//! Booster NGFW Application
//!
//! # Copyright
//! Copyright (C) 2020 QUARTIQ GmbH - All Rights Reserved
//! Unauthorized usage, editing, or copying is strictly prohibited.
//! Proprietary and confidential.
use super::hal;

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
