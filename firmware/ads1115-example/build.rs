//! Build script for the ads1115-example crate.
//!
//! The linker script does not check the crate source directory by default, so
//! we need to tell it to look in the crate root to find the memory.x file.

use std::env;

fn main() {
    let crate_dir = env::var("CARGO_MANIFEST_DIR").unwrap();

    // Include the source directory in the linker search path.
    println!("cargo:rustc-link-search={crate_dir}");
    println!("cargo:rerun-if-changed=memory.x");
}
