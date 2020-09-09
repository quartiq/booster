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
        // Disable all SIG_ON outputs.
        gpiog.bsrr.write(|w| w.bits(0xFF00));

        // Disable all EN_PWR outputs.
        gpiod.bsrr.write(|w| w.bits(0x00FF));
    }
}
