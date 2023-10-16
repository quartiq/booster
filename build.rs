//! Booster NGFW build script
//!
//! # Note
//! This build script is run immediately before the build is completed and is used to inject
//! environment variables into the build.
fn main() {
    built::write_built_file().expect("Failed to acquire build-time information");
}
