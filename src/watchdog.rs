//! Booster NGFW watchdog manager

use crate::hal;
use hal::prelude::*;

/// Represents various clients that can check in with the watchdog.
pub enum WatchdogClient {
    Idle = 0,
    Usb = 1,
    Button = 2,
    Monitor = 3,
}

/// A manager for the device indepedent watchdog.
///
/// The manager waits for a number of clients to check in before feeding the watchdog.
pub struct WatchdogManager {
    watchdog: hal::watchdog::IndependentWatchdog,
    check_ins: [bool; 4],
}

impl WatchdogManager {
    /// Construct a new watchdog manager.
    ///
    /// # Args
    /// * `watchdog` - The inedpdent watchdog timer.
    pub fn new(mut watchdog: hal::watchdog::IndependentWatchdog) -> Self {
        watchdog.feed();
        //watchdog.start(4.secs());

        Self {
            watchdog,
            check_ins: [false; 4],
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
            //self.watchdog.feed();
            self.check_ins = [false; 4];
        }
    }
}
