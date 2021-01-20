//! Booster NGFW build script
//!
//! # Copyright
//! Copyright (C) 2020 QUARTIQ GmbH - All Rights Reserved
//! Unauthorized usage, editing, or copying is strictly prohibited.
//! Proprietary and confidential.
//!
//! # Note
//! This build script is run immediately before the build is completed and is used to inject
//! environment variables into the build.
use std::env;

fn main() {
    let mut options = built::Options::default();
    options.set_git(true);
    let src = env::var("CARGO_MANIFEST_DIR").unwrap();
    let dst = std::path::Path::new(&env::var("OUT_DIR").unwrap()).join("built.rs");
    built::write_built_file_with_opts(&options, src.as_ref(), &dst).expect("Failed to acquire build-time information");
}
