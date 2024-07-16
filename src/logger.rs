//! Booster NGFW logging utilities
use heapless::String;

use super::SerialTerminal;
use core::fmt::Write;
use log::LevelFilter;
use rtt_target::rprintln;

/// A logging buffer for storing serialized logs pending transmission.
///
/// # Notes
/// The BufferedLog contains a character buffer of the log data waiting to be written. It is
/// intended to be consumed asynchronously. In the case of booster, this log data is consumed in the
/// USB task.
pub struct BufferedLog {
    logs: heapless::mpmc::Q16<heapless::String<256>>,
    level_filter: LevelFilter,
}

impl BufferedLog {
    /// Construct a new buffered log object.
    pub const fn new() -> Self {
        Self {
            logs: heapless::mpmc::Q16::new(),
            level_filter: LevelFilter::Info,
        }
    }

    /// Process all of the available log data.
    ///
    /// # Args
    /// * `terminal` - The serial terminal to write log data into.
    pub fn process(&self, terminal: &mut SerialTerminal) {
        while let Some(log) = self.logs.dequeue() {
            terminal
                .interface_mut()
                .inner_mut()
                .write(log.as_bytes())
                .ok();
        }
    }
}

impl log::Log for BufferedLog {
    fn enabled(&self, metadata: &log::Metadata) -> bool {
        self.level_filter.ge(&metadata.level())
    }

    fn log(&self, record: &log::Record) {
        if !self.enabled(record.metadata()) {
            return;
        }

        rprintln!("{} - {}", record.level(), record.args());
        let source_file = record.file().unwrap_or("Unknown");
        let source_line = record.line().unwrap_or(u32::MAX);

        // Print the record into the buffer.
        let mut string: String<256> = String::new();
        if writeln!(
            &mut string,
            "[{}] {}:{} - {}\n",
            record.level(),
            source_file,
            source_line,
            record.args()
        )
        .is_err()
        {
            // If we cannot encode the log entry, note this in the output log to indicate the log
            // was dropped.
            error!("Log entry overflow");
            return;
        };

        self.logs.enqueue(string).ok();
    }

    // The log is not capable of being flushed as it does not own the data consumer.
    fn flush(&self) {}
}
