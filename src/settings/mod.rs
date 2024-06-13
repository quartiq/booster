//! Booster NGFW NVM settings

pub mod channel_settings;
pub mod flash;
pub mod global_settings;
pub mod runtime_settings;
mod sinara;
use encdec::{Decode, DecodeOwned, Encode};
use serde::{Deserialize, Serialize};

use sinara::{BoardId as SinaraBoardId, SinaraConfiguration};

pub use channel_settings::BoosterChannelSettings;
pub use global_settings::BoosterSettings;

/// A semantic version control for recording software versions.
#[derive(Encode, DecodeOwned, Serialize, Deserialize, Debug, PartialEq, Copy, Clone)]
pub struct SemVersion {
    major: u8,
    minor: u8,
    patch: u8,
}

impl SemVersion {
    /// Determine if this version is compatible with `rhs`.
    pub fn is_compatible_with(&self, rhs: &SemVersion) -> bool {
        (self.major == rhs.major) && (self.minor <= rhs.minor)
    }
}
