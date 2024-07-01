pub mod main_board;
pub mod rf_channel;
mod sinara;

use core::cmp::Ordering;
use encdec::{Decode, DecodeOwned, Encode};
use serde::{Deserialize, Serialize};

/// A semantic version control for recording software versions.
#[derive(Encode, DecodeOwned, Serialize, Deserialize, Debug, PartialEq, Eq, Copy, Clone)]
pub struct SemVersion {
    major: u8,
    minor: u8,
    patch: u8,
}

impl SemVersion {
    /// Determine if this version is compatible with `rhs`.
    pub fn is_compatible_with(&self, rhs: &SemVersion) -> bool {
        self.major == rhs.major
    }
}

impl core::cmp::PartialOrd for SemVersion {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl core::cmp::Ord for SemVersion {
    fn cmp(&self, other: &Self) -> Ordering {
        match self.major.cmp(&other.major) {
            Ordering::Equal => {}
            other => return other,
        }

        match self.minor.cmp(&other.minor) {
            Ordering::Equal => {}
            other => return other,
        }

        self.patch.cmp(&other.patch)
    }
}
