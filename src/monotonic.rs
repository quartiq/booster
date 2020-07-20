//! Booster NGFW monotonic implementation
//!
//! # Copyright
//! Copyright (C) 2020 QUARTIQ GmbH - All Rights Reserved
//! Unauthorized usage, editing, or copying is strictly prohibited.
//! Proprietary and confidential.
use rtic;
use super::hal;
use core::{
    cmp::Ordering,
    convert::TryInto,
    ops,
};

use hal::prelude::*;

#[derive(Clone, Copy, Eq, PartialEq)]
pub struct Instant {
    inner: i32,
}

impl Instant {
    pub fn now() -> Self {
        let timer = unsafe { &*hal::stm32::TIM5::ptr() };

        // Trigger a capture event to channel 1.
        timer.egr.write(|w| w.cc1g().set_bit());

        // Read the value on channel 1 that was captured as the current instant.
        Instant { inner: timer.ccr1.read().bits() as i32 }
    }

    pub fn elapsed(&self) -> Duration {
        Instant::now() - *self
    }

    pub fn counts(&self) -> u32 {
        self.inner as u32
    }

    pub fn duration_since(&self, earlier: Instant) -> Duration {
        let diff = self.inner - earlier.inner;
        assert!(diff >= 0, "Second instant later than self!");

        Duration { inner: diff as u32 }
    }
}

impl core::fmt::Debug for Instant {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_tuple("Instant")
            .field(&(self.inner as u32))
            .finish()
    }
}

impl core::ops::AddAssign<Duration> for Instant {
    fn add_assign(&mut self, dur: Duration) {
        // NOTE this is a debug assertion because there's no foolproof way to detect a wrap
        // around;
        // the user may write `(instant + dur) + dur` where `dur` is `(1<<31)-1` ticks.
        debug_assert!(dur.inner < (1 << 31));
        self.inner = self.inner.wrapping_add(dur.inner as i32);
    }
}

impl ops::SubAssign<Duration> for Instant {
    fn sub_assign(&mut self, dur: Duration) {
        // NOTE see the NOTE in `<Instant as AddAssign<Duration>>::add_assign`
        debug_assert!(dur.inner < (1 << 31));
        self.inner = self.inner.wrapping_sub(dur.inner as i32);
    }
}

impl ops::Sub<Duration> for Instant {
    type Output = Self;

    fn sub(mut self, dur: Duration) -> Self {
        self -= dur;
        self
    }
}

impl ops::Sub<Instant> for Instant {
    type Output = Duration;

    fn sub(self, other: Instant) -> Duration {
        self.duration_since(other)
    }
}

impl Ord for Instant {
    fn cmp(&self, rhs: &Self) -> Ordering {
        self.inner.wrapping_sub(rhs.inner).cmp(&0)
    }
}

impl PartialOrd for Instant {
    fn partial_cmp(&self, rhs: &Self) -> Option<Ordering> {
        Some(self.cmp(rhs))
    }
}

/// A `Duration` type to represent a span of time.
///
/// # Correctness
///
/// Each tick is 1ms, so this duration can hold up to 49.7 days.
#[derive(Clone, Copy, Default, Eq, Ord, PartialEq, PartialOrd)]
pub struct Duration {
    inner: u32,
}

impl Duration {
    /// Creates a new `Duration` from the specified number of clock cycles
    pub fn from_cycles(cycles: u32) -> Self {
        Duration { inner: cycles }
    }

    /// Returns the total number of clock cycles contained by this `Duration`
    pub fn as_cycles(&self) -> u32 {
        self.inner
    }
}

// Used internally by RTIC to convert the duration into a known type
impl TryInto<u32> for Duration {
    type Error = core::convert::Infallible;

    fn try_into(self) -> Result<u32, Self::Error> {
        Ok(self.as_cycles())
    }
}

impl ops::AddAssign for Duration {
    fn add_assign(&mut self, dur: Duration) {
        self.inner += dur.inner;
    }
}

impl ops::Add<Duration> for Duration {
    type Output = Self;

    fn add(self, other: Self) -> Self {
        Duration {
            inner: self.inner + other.inner,
        }
    }
}

impl ops::SubAssign for Duration {
    fn sub_assign(&mut self, rhs: Duration) {
        self.inner -= rhs.inner;
    }
}

impl ops::Sub<Duration> for Duration {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self {
        Duration {
            inner: self.inner - rhs.inner,
        }
    }
}

pub trait U32Ext {
    fn secs(self) -> Duration;

    fn millis(self) -> Duration;
}

impl U32Ext for u32 {
    fn secs(self) -> Duration {
        Duration {
            inner: self * 1_000,
        }
    }

    fn millis(self) -> Duration {
        Duration {
            inner: self
        }
    }

}

pub struct Tim5;

impl Tim5 {
    pub fn new(timer: hal::stm32::TIM5, clocks: hal::rcc::Clocks) {

        let rcc = unsafe { &(*hal::stm32::RCC::ptr()) };
        rcc.apb1enr.modify(|_, w| w.tim5en().set_bit());
        rcc.apb1rstr.modify(|_, w| w.tim5rst().set_bit());
        rcc.apb1rstr.modify(|_, w| w.tim5rst().clear_bit());

        // Disable timer.
        timer.cr1.modify(|_, w| w.cen().clear_bit());

        // Clear the timer value.
        timer.cnt.reset();

        // Assert PCLK1 is 42MHz here so we update the fraction later if necessary.
        assert!(clocks.ppre1() == 1);
        assert!(clocks.pclk1() == 42_000_000_u32.hz());

        // Calculate and set the prescaler such that the timer operates at 1ms ticks.
        timer.psc.modify(|_, w| w.psc().bits(42_000u16 - 1u16));

        // Configure Capture/Compare channel 1 as input on TI1 (unused).
        timer.ccmr1_output().modify(|_, w| unsafe { w.cc1s().bits(1)});

        // Start the timer.
        timer.cr1.modify(|_, w| w.cen().set_bit());

        drop(timer)
    }
}

impl rtic::Monotonic for Tim5 {
    type Instant = Instant;

    fn ratio() -> rtic::Fraction {
        // Monotonic * fraction = sysclk
        rtic::Fraction {
            numerator: 42_000,
            denominator: 1,
        }
    }

    fn now() -> Self::Instant {
        Instant::now()
    }

    unsafe fn reset() {
        let timer = &*hal::stm32::TIM5::ptr();

        // Disable timer.
        timer.cr1.modify(|_, w| w.cen().clear_bit());

        // Clear the timer value.
        timer.cnt.reset();

        // Enable timer.
        timer.cr1.modify(|_, w| w.cen().set_bit());

    }

    fn zero() -> Self::Instant {
        Instant { inner: 0 }
    }
}
