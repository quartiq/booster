//! Booster NGFW Application

// Flag used to indicate that a reboot to DFU is requested.
const DFU_REBOOT_FLAG: u32 = 0xDEAD_BEEF;

use super::hal;

use hal::hal::{delay::DelayNs, digital::OutputPin};

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
    cortex_m::interrupt::disable();

    // Shutdown all of the RF channels.
    shutdown_channels();

    // Write panic info to RAM.
    panic_persist::report_panic_info(info);

    #[cfg(feature = "rtt")]
    if let Some(mut channel) = unsafe { rtt_target::UpChannel::conjure(0) } {
        use core::fmt::Write;

        channel.set_mode(rtt_target::ChannelMode::BlockIfFull);
        writeln!(channel, "{}", info).ok();
    }

    // Reset the device in `release` configuration.
    #[cfg(not(debug_assertions))]
    cortex_m::peripheral::SCB::sys_reset();

    #[cfg(debug_assertions)]
    loop {}
}

/// Unconditionally disable and power-off all channels.
pub fn shutdown_channels() {
    let gpiod = unsafe { &*hal::pac::GPIOD::ptr() };
    let gpiog = unsafe { &*hal::pac::GPIOG::ptr() };

    unsafe {
        // Disable all SIG_ON outputs. Note that the upper 16 bits of this register are the ODR
        // reset bits.
        gpiog.bsrr().write(|w| w.bits(0xFF00_0000));

        // Disable all EN_PWR outputs. Note that the upper 16 bits of this register are the ODR
        // reset bits.
        gpiod.bsrr().write(|w| w.bits(0x00FF_0000));
    }
}

/// Generate a manual I2C bus reset to clear the bus.
///
/// # Args
/// * `sda` - The I2C data line.
/// * `scl` - The I2C clock line.
/// * `delay` - A means of delaying time.
pub fn i2c_bus_reset(sda: &mut impl OutputPin, scl: &mut impl OutputPin, delay: &mut impl DelayNs) {
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
    let rcc = unsafe { &*hal::pac::RCC::ptr() };

    rcc.csr().read().wdgrstf().bit_is_set()
}

/// Clear all of the reset flags in the device.
pub fn clear_reset_flags() {
    let rcc = unsafe { &*hal::pac::RCC::ptr() };

    rcc.csr().modify(|_, w| w.rmvf().set_bit());
}

pub fn start_dfu_reboot() {
    extern "C" {
        static mut _bootflag: u32;
    }

    unsafe {
        let start_ptr = core::ptr::addr_of_mut!(_bootflag);
        core::ptr::write_unaligned(start_ptr, DFU_REBOOT_FLAG)
    }

    cortex_m::peripheral::SCB::sys_reset();
}

pub fn dfu_bootflag() -> bool {
    // Obtain panic region start and end from linker symbol _panic_dump_start and _panic_dump_end
    extern "C" {
        static mut _bootflag: u32;
    }

    unsafe {
        let start_ptr = core::ptr::addr_of_mut!(_bootflag);
        let set = DFU_REBOOT_FLAG == core::ptr::read_unaligned(start_ptr);

        // Clear the boot flag after checking it to ensure it doesn't stick between reboots.
        core::ptr::write_unaligned(start_ptr, 0);
        set
    }
}

/// Reset the device to the internal DFU bootloader.
pub fn execute_system_bootloader() -> ! {
    cortex_m::interrupt::disable();

    // Disable the SysTick peripheral.
    let systick = unsafe { &*cortex_m::peripheral::SYST::PTR };
    unsafe {
        systick.csr.write(0);
        systick.rvr.write(0);
        systick.cvr.write(0);
    }

    // Clear NVIC interrupt flags and enables.
    let nvic = unsafe { &*cortex_m::peripheral::NVIC::PTR };
    for reg in nvic.icer.iter() {
        unsafe {
            reg.write(u32::MAX);
        }
    }

    for reg in nvic.icpr.iter() {
        unsafe {
            reg.write(u32::MAX);
        }
    }

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
