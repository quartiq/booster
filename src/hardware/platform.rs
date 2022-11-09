//! Booster NGFW Application
//!
//! # Copyright
///! Copyright (C) 2020-2022 QUARTIQ GmbH
use super::hal;

use embedded_hal::{blocking::delay::DelayUs, digital::v2::OutputPin};

// Booster hardware channels are capable of withstanding up to 1W of reflected RF power. This
// corresponds with a value of 30 dBm.
pub const MAXIMUM_REFLECTED_POWER_DBM: f32 = 30.0;

// The maximum allowable output power. This is calculated as:
// 38dBm (nominal) + 6 dB (glitches) + 3 dB (margin)
pub const MAX_OUTPUT_POWER_DBM: f32 = 47.0;

// The voltage supply to the RF transitor bias DAC.
pub const BIAS_DAC_VCC: f32 = 3.2;

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    use core::fmt::Write;
    use rtt_target::{ChannelMode, UpChannel};

    cortex_m::interrupt::disable();

    // Shutdown all of the RF channels.
    shutdown_channels();

    if let Some(mut channel) = unsafe { UpChannel::conjure(0) } {
        channel.set_mode(ChannelMode::BlockIfFull);
        writeln!(channel, "{}", info).ok();
    }

    // Write panic info to RAM.
    panic_persist::report_panic_info(info);

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

/// Reset the device to the internal DFU bootloader.
pub fn reset_to_dfu_bootloader() {
    // Disable the SysTick peripheral.
    let systick = unsafe { &*cortex_m::peripheral::SYST::ptr() };
    unsafe {
        systick.csr.write(0);
        systick.rvr.write(0);
        systick.cvr.write(0);
    }

    // Disable the USB peripheral.
    let usb_otg = unsafe { &*hal::stm32::OTG_FS_GLOBAL::ptr() };
    usb_otg.gccfg.write(|w| unsafe { w.bits(0) });

    // Reset the RCC configuration.
    let rcc = unsafe { &*hal::stm32::RCC::ptr() };

    // Enable the HSI - we will be switching back to it shortly for the DFU bootloader.
    rcc.cr.modify(|_, w| w.hsion().set_bit());

    // Reset the CFGR and begin using the HSI for the system bus.
    rcc.cfgr.reset();

    // Reset the configuration register.
    rcc.cr.reset();

    // Reset the PLL configuration now that PLLs are unused.
    rcc.pllcfgr.reset();

    // Remap to system memory.
    rcc.apb2enr.modify(|_, w| w.syscfgen().set_bit());

    let syscfg = unsafe { &*hal::stm32::SYSCFG::ptr() };
    syscfg.memrm.write(|w| unsafe { w.mem_mode().bits(0b01) });

    // Now that the remap is complete, impose instruction and memory barriers on the
    // CPU.
    cortex_m::asm::isb();
    cortex_m::asm::dsb();

    // It appears that they must be enabled for the USB-based DFU bootloader to operate.
    unsafe { cortex_m::interrupt::enable() };

    // The STM32F4xx does not provide a means to modify the BOOT pins during
    // run-time. Instead, we manually load the bootloader stack pointer and start
    // address from system memory and begin execution. The datasheet indicates that
    // the initial stack pointer is stored at an offset of 0x0000 and the first
    // instruction begins at an offset of 0x0004.
    unsafe {
        let system_memory_address: *const u32 = 0x1FFF_0000 as *const u32;
        cortex_m::asm::bootload(system_memory_address);
    }
}
