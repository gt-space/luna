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
  let common_target_dir = workspace_root.join("target").join("flight-common-build");

  // Build the common library
  let mut cmd = Command::new("cargo");
  cmd.arg("build")
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
  let status = cmd.status().expect("Failed to execute cargo build for 'common'");
  if !status.success() {
      panic!("Build script failed: 'common' did not compile.");
  }

  // Rename libcommon.so to common.so
  let so_path = common_target_dir.join(&profile).join("libcommon.so");
  let common_so_path = common_target_dir.join(&profile).join("common.so");
  fs::rename(so_path, common_so_path.clone()).expect("Failed to rename libcommon.so to common.so");
  println!("COMMON_SO_PATH: {}", common_so_path.display());

  // This creates an environment variable available at compile-time via env!("COMMON_SO_PATH")
  println!("cargo:rustc-env=COMMON_SO_PATH={}", common_target_dir.join(&profile).display());
}