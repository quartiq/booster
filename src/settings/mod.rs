//! Booster NGFW NVM settings
//!
//! # Copyright
//! Copyright (C) 2020 QUARTIQ GmbH - All Rights Reserved
//! Unauthorized usage, editing, or copying is strictly prohibited.
//! Proprietary and confidential.

pub mod channel_settings;
pub mod global_settings;
mod sinara;

use sinara::{BoardId as SinaraBoardId, SinaraConfiguration};

pub use channel_settings::BoosterChannelSettings;
pub use global_settings::BoosterSettings;

/// A semantic version control for recording software versions.
#[derive(serde::Serialize, serde::Deserialize, PartialEq, Copy, Clone)]
pub struct SemVersion {
    major: u8,
    minor: u8,
    patch: u8,
}

impl SemVersion {
    /// Determine if this version is compatible with `rhs`.
    pub fn is_compatible(&self, rhs: &SemVersion) -> bool {
        (self.major == rhs.major) && (self.minor <= rhs.minor)
    }
}
