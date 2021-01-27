//! Booster NGFW Application
//!
//! # Copyright
//! Copyright (C) 2020 QUARTIQ GmbH - All Rights Reserved
//! Unauthorized usage, editing, or copying is strictly prohibited.
//! Proprietary and confidential.
use super::hal;

use embedded_hal::{blocking::delay::DelayUs, digital::v2::OutputPin};

use asm_delay::bitrate::U32BitrateExt;

// Booster hardware channels are capable of withstanding up to 1W of reflected RF power. This
// corresponds with a value of 30 dBm.
pub const MAXIMUM_REFLECTED_POWER_DBM: f32 = 30.0;

struct GpioBOutput {
    index: usize,
    original_otyper: u32,
    original_pupdr: u32,
    original_moder: u32,
}

impl GpioBOutput {
    /// Construct a GPIO_B output pin based on the pin index.
    pub fn new(index: usize) -> Self {
        assert!(index <= 16);
        // Preserve MODER, PUPDR, OTYPER
        let gpiob = unsafe { &*hal::stm32::GPIOB::ptr() };

        let original_pupdr = gpiob.pupdr.read().bits();
        let original_moder = gpiob.moder.read().bits();
        let original_otyper = gpiob.otyper.read().bits();

        let offset = 2 * index;

        // Note(unsafe): We check that offset is always less than or equal to 16 so that it will fit
        // in the 32-bit register.
        unsafe {
            // Force the pin to open-drain configuration.
            gpiob
                .pupdr
                .modify(|r, w| w.bits(r.bits() & !(0b11 << offset)));
            gpiob
                .otyper
                .modify(|r, w| w.bits(r.bits() | (0b1 << index)));
            gpiob
                .moder
                .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b1 << offset)));
        }

        Self {
            index,
            original_moder,
            original_pupdr,
            original_otyper,
        }
    }

    /// Release the GPIO_B pin and reconfigure it to whatever configuration was present before
    /// construction.
    pub fn free(self) {
        // Restore MODER, PUPDR, OTYPER
        let gpiob = unsafe { &*hal::stm32::GPIOB::ptr() };

        // Note(unsafe): We are writing back a previous bit configuration, so this is always valid.
        unsafe {
            gpiob.pupdr.write(|w| w.bits(self.original_pupdr));
            gpiob.otyper.write(|w| w.bits(self.original_otyper));
            gpiob.moder.write(|w| w.bits(self.original_moder));
        }
    }
}

impl OutputPin for GpioBOutput {
    type Error = ();

    fn set_low(&mut self) -> Result<(), ()> {
        // Note(unsafe): We are accessing a single register atomically here. We have a logical
        // contract with the user that we own this specific pin setting.
        unsafe {
            let gpiob = &*hal::stm32::GPIOB::ptr();
            gpiob.bsrr.write(|w| w.bits(1 << (self.index + 16)));
        }
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), ()> {
        // Note(unsafe): We are accessing a single register atomically here. We have a logical
        // contract with the user that we own this specific pin setting.
        unsafe {
            let gpiob = &*hal::stm32::GPIOB::ptr();
            gpiob.bsrr.write(|w| w.bits(1 << self.index));
        }
        Ok(())
    }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    cortex_m::interrupt::disable();

    // Shutdown all of the RF channels.
    shutdown_channels();

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

/// An unsafe means of performing a forced I2C bus reset on the shared RF channel I2C bus.
///
/// # Safety
/// This function is only safe to call if it is guaranteed that it will not be interrupted by
/// another context that will attempt to use the I2C bus.
///
/// If a synchronization mechanism is not used (e.g. via RTIC resource sharing), interrupts should
/// be disabled before calling this function.
///
/// # Note
/// This function will manually overwrite the I2C pin configurations into push/pull mode in order to
/// perform a bus reset. Once the reset is complete, pin configurations will be restored.
pub unsafe fn reset_shared_i2c_bus() {
    let mut delay = asm_delay::AsmDelay::new(168.mhz());

    // Manually configure SCL/SDA into GPIO mode.
    let mut scl = GpioBOutput::new(6);
    let mut sda = GpioBOutput::new(7);

    i2c_bus_reset(&mut sda, &mut scl, &mut delay);

    // Restore SCL/SDA pin configurations. Note: Ordering matters, since these structures restore
    // previous configurations. Freeing order is the exact opposite of the construction order.
    sda.free();
    scl.free();

    let i2c1 = unsafe { &*hal::stm32::I2C1::ptr() };

    let cr2 = i2c1.cr2.read();
    let ccr = i2c1.ccr.read();
    let trise = i2c1.trise.read();
    let cr1 = i2c1.cr1.read();

    // Reset the I2C peripheral state now that we've finished a bus clear.
    // Note: We are currently not preserving own-address-register.
    i2c1.cr1.modify(|_, w| w.swrst().set_bit());
    i2c1.cr1.modify(|_, w| w.swrst().clear_bit());

    // Note(unsafe): All bit setings are preserving register configurations after reset.
    unsafe {
        i2c1.cr2.write(|w| w.bits(cr2.bits()));
        i2c1.trise.write(|w| w.bits(trise.bits()));
        i2c1.ccr.write(|w| w.bits(ccr.bits()));
        i2c1.cr1.write(|w| w.bits(cr1.bits()));
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

#[cfg(feature = "unstable")]
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
    // address from system memory and begin execution. The datasheet indices that
    // the initial stack pointer is stored at an offset of 0x0000 and the first
    // instruction begins at an offset of 0x0004.
    let system_memory_address: u32 = 0x1FFF_0000;
    unsafe {
        llvm_asm!(
            "MOV r3, $0\n
             LDR sp, [r3, #0]\n
             LDR r3, [r3, #4]\n
             BX r3\n"
             :
             : "r"(system_memory_address)
             : "r3","r4"
             :
        );
    }
}
