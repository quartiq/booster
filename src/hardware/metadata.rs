use serde::Serialize;

use super::{platform, HardwareVersion};

mod build_info {
    include!(concat!(env!("OUT_DIR"), "/built.rs"));
}

static mut METADATA: ApplicationMetadata = ApplicationMetadata {
    firmware_version: "Unspecified",
    build_time_utc: build_info::BUILT_TIME_UTC,
    rust_version: build_info::RUSTC_VERSION,
    profile: build_info::PROFILE,
    git_revision: "Unspecified",
    git_dirty: true,
    features: build_info::FEATURES_STR,
    panic_info: "None",
    watchdog: false,
    hardware_version: HardwareVersion::Unknown(0),
};

#[derive(Serialize)]
pub struct ApplicationMetadata {
    pub firmware_version: &'static str,
    pub build_time_utc: &'static str,
    pub rust_version: &'static str,
    pub git_revision: &'static str,
    pub profile: &'static str,
    pub git_dirty: bool,
    pub features: &'static str,
    pub panic_info: &'static str,
    pub watchdog: bool,
    pub hardware_version: HardwareVersion,
}

impl ApplicationMetadata {
    pub fn new(hardware_version: HardwareVersion) -> &'static ApplicationMetadata {
        let mut meta = unsafe { &mut METADATA };
        meta.hardware_version = hardware_version;
        meta.watchdog = platform::watchdog_detected();

        if let Some(panic_data) = panic_persist::get_panic_message_utf8() {
            meta.panic_info = panic_data;
        }

        if let Some(git_revision) = build_info::GIT_COMMIT_HASH {
            meta.git_revision = git_revision;
        }

        if let Some(dirty) = build_info::GIT_DIRTY {
            meta.git_dirty = dirty;
        }

        if let Some(version) = build_info::GIT_VERSION {
            meta.firmware_version = version;
        }

        platform::clear_reset_flags();

        meta
    }
}
