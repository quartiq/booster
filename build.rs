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
use std::process::Command;

fn main() {
    // Inject the git revision into an environment variable for compilation.
    let dirty_flag = if !Command::new("git")
        .args(&["diff", "--quiet"])
        .status()
        .unwrap()
        .success()
    {
        "-dirty"
    } else {
        ""
    };

    let output = Command::new("git")
        .args(&["rev-parse", "HEAD"])
        .output()
        .unwrap();
    let revision = String::from_utf8(output.stdout).unwrap();
    println!(
        "cargo:rustc-env=GIT_REVISION={}{}",
        revision.trim(),
        dirty_flag
    );

    let output = Command::new("git")
        .args(&["describe", "--tags"])
        .output()
        .unwrap();
    let version = String::from_utf8(output.stdout).unwrap();
    println!("cargo:rustc-env=VERSION={}", version.trim());

    // Collect all of the enabled features and inject them as an environment variable.
    let mut features: Vec<String> = Vec::new();
    for (key, _) in std::env::vars() {
        let strings: Vec<&str> = key.split("CARGO_FEATURE_").collect();
        if strings.len() > 1 {
            println!("{}", strings[1]);
            features.push(String::from(strings[1]));
        }
    }

    println!("cargo:rustc-env=ALL_FEATURES={}", features.join(", "));
}
