//! Booster run-time application metadata

use serde::Serialize;

use super::{platform, HardwareVersion};

mod build_info {
    include!(concat!(env!("OUT_DIR"), "/built.rs"));
}

#[derive(Serialize)]
pub struct ApplicationMetadata {
    pub firmware_version: &'static str,
    pub rust_version: &'static str,
    pub profile: &'static str,
    pub git_dirty: bool,
    pub features: &'static str,
    pub panic_info: &'static str,
    pub watchdog: bool,
    pub hardware_version: HardwareVersion,
}

impl ApplicationMetadata {
    /// Construct the global metadata.
    ///
    /// # Note
    /// This may only be called once.
    ///
    /// # Args
    /// * `hardware_version` - The hardware version detected.
    ///
    /// # Returns
    /// A reference to the global metadata.
    pub fn new(hardware_version: HardwareVersion) -> &'static ApplicationMetadata {
        let mut meta = cortex_m::singleton!(: ApplicationMetadata = ApplicationMetadata {
            firmware_version: "Unspecified",
            rust_version: build_info::RUSTC_VERSION,
            profile: build_info::PROFILE,
            git_dirty: true,
            features: build_info::FEATURES_STR,
            panic_info: "None",
            watchdog: platform::watchdog_detected(),
            hardware_version,
        })
        .unwrap();

        if let Some(panic_data) = panic_persist::get_panic_message_utf8() {
            meta.panic_info = panic_data;
        }

        if let Some(dirty) = build_info::GIT_DIRTY {
            meta.git_dirty = dirty;
        }

        if let Some(version) = build_info::GIT_VERSION {
            meta.firmware_version = version;
        }

        meta
    }
}
