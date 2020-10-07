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
    buffer: core::cell::UnsafeCell<LogBuffer>,
}

// Critical sections are used to manage the buffer access. This type is safe to share across contexts.
unsafe impl Sync for BufferedLog {}

impl BufferedLog {
    /// Construct a new buffered log object.
    pub const fn new() -> Self {
        Self {
            buffer: core::cell::UnsafeCell::new(LogBuffer::new()),
        }
    }

    /// Process all of the available log data.
    ///
    /// # Args
    /// * `terminal` - The serial terminal to write log data into.
    pub fn process(&self, terminal: &mut SerialTerminal) {
        cortex_m::interrupt::free(|_cs| {
            let buffer = unsafe { &mut *self.buffer.get() };
            terminal.write(buffer.data());
            buffer.clear();
        });
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

        cortex_m::interrupt::free(|_cs| {
            let buffer = unsafe { &mut *self.buffer.get() };
            buffer.append(&string.into_bytes());
        });
    }

    // The log is not capable of being flushed as it does not own the data consumer.
    fn flush(&self) {}
}

/// An internal buffer for storing serialized log data. This is essentially a vector of u8.
struct LogBuffer {
    data: [u8; 1024],
    index: usize,
}

impl LogBuffer {
    /// Construct the buffer.
    pub const fn new() -> Self {
        Self {
            data: [0u8; 1024],
            index: 0,
        }
    }

    /// Append data into the buffer.
    ///
    /// # Args
    /// * `data` - The data to append. If space isn't available, as much data will be appended as
    ///   possible.
    pub fn append(&mut self, data: &[u8]) {
        let tail = &mut self.data[self.index..];
        self.index += if data.len() > tail.len() {
            tail.copy_from_slice(&data[..tail.len()]);
            tail.len()
        } else {
            tail[..data.len()].copy_from_slice(data);
            data.len()
        }
    }

    /// Get the data in the buffer.
    pub fn data(&self) -> &[u8] {
        &self.data[..self.index]
    }

    /// Clear contents of the buffer.
    pub fn clear(&mut self) {
        self.index = 0;
    }
}
