//! Builds the common crate as a cdylib and copies libcommon.so into flight2/

use std::{
    env, fs,
    path::PathBuf,
    process::Command,
};

fn main() {
    // Directory of the package that has this build.rs (flight2/)
    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    // Parent directory of the manifest_dir (luna/)
    let workspace_root = manifest_dir
        .parent()
        .expect("flight2 must be under workspace root")
        .to_path_buf();
    // Type of build that we specified when invoking this script (debug or release)
    let profile = env::var("PROFILE").unwrap();


    // Directory that the build shared object file will be placed in
    // This is a separate target directory which ensures that a deadlock 
    // does not occur with the child waiting for the parent to release the 
    // package lock and the parent waiting for the child to build common
    let common_target_dir = workspace_root.join("target").join("flight2-common-build");

    // Build the common library
    let mut cmd = Command::new("cargo");
    cmd.arg("build")
        .arg("-p")
        .arg("common")
        .current_dir(&workspace_root)
        .env("CARGO_TARGET_DIR", &common_target_dir);
    if profile == "release" {
        cmd.arg("--release");
    }

    // More deadlock prevention that could occur based on Cargo jobserver behavior
    cmd.env_remove("MAKEFLAGS");
    cmd.env_remove("CARGO_MAKEFLAGS");

    // Build the common library
    let status = cmd.status().expect("Failed to execute cargo build for 'common'");
    if !status.success() {
        panic!("Build script failed: 'common' did not compile.");
    }

    // The generated shared object will be named libcommon.so, but we want it 
    // to be named common.so
    let built_so = "libcommon.so";
    let src = common_target_dir.join(&profile).join("libcommon.so");
    let dst = manifest_dir.join("common.so");

    // Copy the generated shared object file into the flight2 directory
    if src.exists() {
        fs::copy(&src, &dst).unwrap_or_else(|_| panic!("Failed to copy {} into flight2", built_so));
        println!("cargo:warning=Synced {} -> {}", built_so, dst.display());
    } else {
        println!(
            "cargo:warning={} not found at {}. Check common crate type (cdylib).",
            built_so,
            src.display()
        );
    }
}
