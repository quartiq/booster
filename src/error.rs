//! Error type definitions for Booster NGFW
//!
//! # Copyright
//! Copyright (C) 2020 QUARTIQ GmbH - All Rights Reserved
//! Unauthorized usage, editing, or copying is strictly prohibited.
//! Proprietary and confidential.

/// An enumeration of possible errors with the device.
#[derive(Debug, Copy, Clone)]
pub enum Error {
    NotImplemented,
    Invalid,
    NotPresent,
    Interface,
    Bounds,
}
