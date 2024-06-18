pub mod main_board;
pub mod rf_channel;
mod sinara;

use encdec::{Decode, DecodeOwned, Encode};
use serde::{Deserialize, Serialize};

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
