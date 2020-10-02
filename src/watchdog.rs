//! Booster NGFW watchdog manager
//!
//! # Copyright
//! Copyright (C) 2020 QUARTIQ GmbH - All Rights Reserved
//! Unauthorized usage, editing, or copying is strictly prohibited.
//! Proprietary and confidential.

use crate::hal;
use embedded_hal::watchdog::{Watchdog, WatchdogEnable};
use hal::time::U32Ext;

/// Represents various clients that can check in with the watchdog.
pub enum WatchdogClient {
    TelemetryTask = 0,
    IdleTask = 1,
    UsbTask = 2,
    ButtonTask = 3,
    FanTask = 4,
    MonitorTask = 5,
}

/// A manager for the device indepedent watchdog.
///
/// The manager waits for a number of clients to check in before feeding the watchdog.
pub struct WatchdogManager {
    watchdog: hal::watchdog::IndependentWatchdog,
    check_ins: [bool; 6],
}

impl WatchdogManager {
    /// Construct a new watchdog manager.
    ///
    /// # Args
    /// * `watchdog` - The inedpdent watchdog timer.
    pub fn new(mut watchdog: hal::watchdog::IndependentWatchdog) -> Self {
        watchdog.feed();
        watchdog.start(2_000_u32.ms());

        Self {
            watchdog,
            check_ins: [false; 6],
        }
    }

    /// Check in with the watchdog.
    ///
    /// # Args
    /// * `client` - The client who is checking in with the watchdog manager.
    pub fn check_in(&mut self, client: WatchdogClient) {
        self.check_ins[client as usize] = true;

        // If all clients have checked in, service the watchdog.
        if self.check_ins.iter().all(|&x| x) {
            self.watchdog.feed();
            self.check_ins = [false; 6];
        }
    }
}