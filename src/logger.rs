//! Booster NGFW logging utilities
//!
//! # Copyright
//! Copyright (C) 2020 QUARTIQ GmbH - All Rights Reserved
//! Unauthorized usage, editing, or copying is strictly prohibited.
//! Proprietary and confidential.
use heapless::{consts, String};

use super::SerialTerminal;
use core::fmt::Write;

/// A logging buffer for storing serialized logs pending transmission.
///
/// # Notes
/// The LoBufferedLog contains a character buffer of the log data waiting to be written. It is
/// intended to be consumed asynchronously. In the case of booster, this log data is consumed in the
/// USB task.
pub struct BufferedLog {
    logs: heapless::mpmc::Q16<heapless::String<consts::U128>>,
}

impl BufferedLog {
    /// Construct a new buffered log object.
    pub const fn new() -> Self {
        Self {
            logs: heapless::mpmc::Q16::new(),
        }
    }

    /// Process all of the available log data.
    ///
    /// # Args
    /// * `terminal` - The serial terminal to write log data into.
    pub fn process(&self, terminal: &mut SerialTerminal) {
        let mut count = 0;
        while let Some(log) = self.logs.dequeue() {
            terminal.write(&log.as_bytes());
            count += 1;
        }

        if count > 0 {
            let mut string: String<consts::U32> = String::new();
            write!(&mut string, "Wrote {} logs\n", count).ok();
            terminal.write(&string.as_bytes());
        }
    }
}

impl log::Log for BufferedLog {
    fn enabled(&self, _metadata: &log::Metadata) -> bool {
        true
    }

    fn log(&self, record: &log::Record) {
        let source_file = record.file().unwrap_or("Unknown");
        let source_line = record.line().unwrap_or(u32::MAX);

        // Print the record into the buffer.
        let mut string: String<consts::U128> = String::new();
        match write!(
            &mut string,
            "[{}] {}:{} - {}\n",
            record.level(),
            source_file,
            source_line,
            record.args()
        ) {
            Err(_) => warn!("Log buffer overflow"),
            _ => {}
        };

        self.logs.enqueue(string).ok();
    }

    // The log is not capable of being flushed as it does not own the data consumer.
    fn flush(&self) {}
}
