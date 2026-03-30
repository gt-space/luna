use std::{env, path::PathBuf, process::Command};

fn main() {
  // Directory of the package that has this build.rs (flight2/)
  let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
  // Parent directory of the manifest_dir (luna/)
  let workspace_root = manifest_dir
    .parent()
    .expect("flight2 must be under workspace root")
    .to_path_buf();
  // Directory of the common library (luna/common)
  let common_dir = workspace_root.join("common");
  // Type of build that we specified when invoking this script (debug or release)
  let profile = env::var("PROFILE").unwrap();

  // Rerun conditions for the build script. The build script reruns if:
  // build.rs changes, common/Cargo.toml changes, common/src changes, or the
  // build profile changes
  println!("cargo:rerun-if-changed=build.rs");
  println!(
    "cargo:rerun-if-changed={}",
    common_dir.join("Cargo.toml").display()
  );
  println!(
    "cargo:rerun-if-changed={}",
    common_dir.join("src").display()
  );
  println!("cargo:rerun-if-env-changed=PROFILE");

  // Directory that the build shared object file will be placed in
  // This is a separate target directory which ensures that a deadlock
  // does not occur with the child waiting for the parent to release the
  // package lock and the parent waiting for the child to build common
  let common_target_dir =
    workspace_root.join("target").join("flight-common-build");

  // Command to build the common library
  let mut cmd = Command::new("cargo");
  cmd
    .arg("build")
    .arg("-p")
    .arg("common")
    .arg("-F")
    .arg("sequences")
    .current_dir(&workspace_root)
    .env("CARGO_TARGET_DIR", &common_target_dir);
  if profile == "release" {
    cmd.arg("--release");
  }
  // More deadlock prevention that could occur based on Cargo jobserver behavior
  cmd.env_remove("MAKEFLAGS");
  cmd.env_remove("CARGO_MAKEFLAGS");

  // Build the common library
  let status = cmd
    .status()
    .expect("Failed to execute cargo build for 'common'");
  if !status.success() {
    panic!("Build script failed: 'common' did not compile.");
  }

  // Expose the built shared library path so the flight binary can embed the
  // bytes directly at compile time.
  let built_so_path = common_target_dir.join(&profile).join("libcommon.so");
  println!(
    "cargo:rustc-env=COMMON_SO_SOURCE_PATH={}",
    built_so_path.display()
  );
}
