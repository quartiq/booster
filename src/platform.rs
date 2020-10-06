//! Booster NGFW Application
//!
//! # Copyright
//! Copyright (C) 2020 QUARTIQ GmbH - All Rights Reserved
//! Unauthorized usage, editing, or copying is strictly prohibited.
//! Proprietary and confidential.
use super::hal;

use embedded_hal::{blocking::delay::DelayUs, digital::v2::OutputPin};

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    cortex_m::interrupt::disable();

    // Shutdown all of the RF channels.
    shutdown_channels();

    // Reset the device in `release` configuration.
    #[cfg(not(debug_assertions))]
    cortex_m::peripheral::SCB::sys_reset();

    #[cfg(debug_assertions)]
    loop {}
}

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

/// Check if the device should reset to the DFU bootloader.
///
/// # Notes
/// This function checks to see if the backup SRAM is enabled. This is used a sa flag by the
/// application to indicate that the device should reset to the DFU bootloader.
///
/// # Args
/// * `device` - The device peripherals.
///
/// # Returns
/// True if a reset to DFU has been requested.
pub fn is_reset_to_dfu_bootloader_requested(device: &hal::stm32::Peripherals) -> bool {
    // Enable the PWR peripheral.
    let pwr_was_enabled = device.RCC.apb1enr.read().pwren().bit_is_set();
    if pwr_was_enabled == false {
        device.RCC.apb1enr.modify(|_, w| w.pwren().set_bit());
    }

    // Check if the backup ram is enabled. If it is, this indicates we need to jump to the
    // bootloader.
    let is_requested = if device.PWR.csr.read().bre().bit_is_set() {
        // Disable the backup regulator.
        device.PWR.cr.modify(|_, w| w.dbp().set_bit());
        device.PWR.csr.modify(|_, w| w.bre().clear_bit());
        true
    } else {
        false
    };

    // Power down the PWR peripheral if it wasn't being used.
    if pwr_was_enabled == false {
        device.RCC.apb1enr.modify(|_, w| w.pwren().clear_bit());
    }

    is_requested
}

/// Initiate a reset to DFU bootloader mode.
///
/// # Notes
/// This enables the backup SRAM as a means of flagging that the device should reboot to DFU mode on
/// the next reboot cycle. A reboot is used to ensure the device is in a default state before
/// executing the DFU bootloader.
///
/// This function resets the device.
pub fn initiate_reset_to_dfu_bootloader() -> ! {
    // Enable control of the PWR peripheral.
    let rcc = unsafe { &*hal::stm32::RCC::ptr() };
    rcc.apb1enr.modify(|_, w| w.pwren().set_bit());

    // Enable the backup SRAM
    let pwr = unsafe { &*hal::stm32::PWR::ptr() };
    pwr.cr.modify(|_, w| w.dbp().set_bit());
    pwr.csr.modify(|_, w| w.bre().set_bit());

    // Wait for BRR to be set.
    while pwr.csr.read().brr().bit_is_clear() {}

    cortex_m::peripheral::SCB::sys_reset();
}

#[cfg(feature = "unstable")]
/// Reset the device to the internal DFU bootloader.
///
/// # Args
/// * `device` - The device peripherals.
pub fn reset_to_dfu_bootloader(device: &hal::stm32::Peripherals) {
    // Remap to system memory.
    device.RCC.apb2enr.modify(|_, w| w.syscfgen().set_bit());
    device
        .SYSCFG
        .memrm
        .write(|w| unsafe { w.mem_mode().bits(0b01) });

    // Now that the remap is complete, impose instruction and memory barriers on the
    // CPU.
    cortex_m::asm::isb();
    cortex_m::asm::dsb();

    // It appears that they must be enabled for the USB-based DFU bootloader to operate.
    unsafe { cortex_m::interrupt::enable() };

    // The STM32F4xx does not provide a means to modify the BOOT pins during
    // run-time. Instead, we manually load the bootloader stack pointer and start
    // address from system memory and begin execution. The datasheet indices that
    // the initial stack pointer is stored at an offset of 0x0000 and the first
    // instruction begins at an offset of 0x0004.
    let system_memory_address: u32 = 0x1FFF_0000;
    unsafe {
        llvm_asm!(
            "MOV r3, $0\n
             MSR msp, r4\n
             LDR sp, [r3, #0]\n
             LDR r3, [r3, #4]\n
             MOV pc, r3\n"
             :
             : "r"(system_memory_address)
             : "r3","r4"
             :
        );
    }
}
