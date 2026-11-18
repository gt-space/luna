//! Compiles the plant C code that the hopper repo's
//! `sims/hopper_6dof_sizer/fsw_bridge/build_plant_c.m` generated.
//!
//! Environment variables:
//! - `HOPPER_PLANT_CODE`: the build_plant_c output folder (`src/`,
//!   `sources.txt`, `defines.txt`, `plant.json`), by default
//!   `<hopper>/sims/hopper_6dof_sizer/fsw_bridge/build/plant`. The generated
//!   code isn't committed anywhere; rebuild it in MATLAB. Without it the crate
//!   builds as a stub whose `Plant::new` returns `Error::NotBuilt`, so `cargo
//!   clippy --all-features` (luna's CI) works without the plant.
//!
//! The compiler comes from the `cc` crate's usual variables, so a cross
//! build only needs the target's C compiler, for example
//! `CC_aarch64_unknown_linux_gnu=aarch64-linux-gnu-gcc` with
//! `--target aarch64-unknown-linux-gnu`. No MATLAB install is needed:
//! build_plant_c packs every header into `src/`.

use std::{
    env, fs,
    path::{Path, PathBuf},
};

fn main() {
    let manifest = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    let out = PathBuf::from(env::var("OUT_DIR").unwrap());
    println!("cargo:rerun-if-env-changed=HOPPER_PLANT_CODE");
    println!("cargo::rustc-check-cfg=cfg(hopper_plant_stub)");
    let Some(code) = env::var_os("HOPPER_PLANT_CODE").map(PathBuf::from) else {
        println!("cargo:rustc-cfg=hopper_plant_stub");
        write_consts(&out, 0.001, 0.01, "(stub: HOPPER_PLANT_CODE was not set)");
        return;
    };
    let read = |name: &str| {
        let path = code.join(name);
        println!("cargo:rerun-if-changed={}", path.display());
        fs::read_to_string(&path).unwrap_or_else(|e| {
            panic!(
                "cannot read {} ({e}); run build_plant_c in MATLAB first or set HOPPER_PLANT_CODE",
                path.display()
            )
        })
    };

    let info = read("plant.json");
    let target = env::var("CARGO_CFG_TARGET_ARCH").unwrap();
    let built_for = json_str(&info, "target").unwrap_or_default();
    let matches = matches!(
        (built_for.as_str(), target.as_str()),
        ("arm64", "aarch64") | ("x86_64", "x86_64")
    );
    if !matches {
        // Both are LP64, so the code compiles either way, but say so
        println!(
            "cargo:warning=plant code was generated for {built_for}, building for {target}; regenerate with build_plant_c('Target', ...) to match"
        );
    }
    let step = json_number(&info, "fixed_step").expect("plant.json has no fixed_step");
    let period = json_number(&info, "fsw_period").expect("plant.json has no fsw_period");
    let out = PathBuf::from(env::var("OUT_DIR").unwrap());
    let info_path = code.join("plant.json").display().to_string();
    let consts = [
        "/// Plant step, s".to_string(),
        format!("pub const STEP: f64 = {step:?};"),
        "/// Flight-software control period the plant was built for, s".to_string(),
        format!("pub const FSW_PERIOD: f64 = {period:?};"),
        "/// `plant.json` the library was built from (what vehicle, which commit)".to_string(),
        format!("pub const INFO_PATH: &str = {info_path:?};"),
    ];
    fs::write(out.join("plant_consts.rs"), consts.join("\n") + "\n").unwrap();

    let src = code.join("src");
    let mut build = cc::Build::new();
    for name in read("sources.txt")
        .lines()
        .map(str::trim)
        .filter(|l| !l.is_empty())
    {
        let file = src.join(name);
        println!("cargo:rerun-if-changed={}", file.display());
        build.file(file);
    }
    for def in read("defines.txt")
        .lines()
        .map(str::trim)
        .filter(|l| !l.is_empty())
    {
        match def.split_once('=') {
            Some((name, value)) => build.define(name, value),
            None => build.define(def, None),
        };
    }
    println!("cargo:rerun-if-changed=csrc/plant_shim.c");
    build
        .file(manifest.join("csrc/plant_shim.c"))
        .include(&src)
        .warnings(false) // generated code; not ours to clean up
        .compile("hopper_plant_c");
    println!("cargo:rustc-link-lib=m");
}

/// STEP, FSW_PERIOD and INFO_PATH for lib.rs
fn write_consts(out: &Path, step: f64, period: f64, info_path: &str) {
    let consts = [
        "/// Plant step, s".to_string(),
        format!("pub const STEP: f64 = {step:?};"),
        "/// Flight-software control period the plant was built for, s".to_string(),
        format!("pub const FSW_PERIOD: f64 = {period:?};"),
        "/// `plant.json` the library was built from (what vehicle, which commit)".to_string(),
        format!("pub const INFO_PATH: &str = {info_path:?};"),
    ];
    fs::write(
        out.join("plant_consts.rs"),
        consts.join(
            "
",
        ) + "
",
    )
    .unwrap();
}

/// `"key": "value"` from plant.json (flat lookup; the file is ours)
fn json_str(json: &str, key: &str) -> Option<String> {
    let rest = &json[json.find(&format!("\"{key}\""))? + key.len() + 2..];
    let rest = &rest[rest.find('"')? + 1..];
    Some(rest[..rest.find('"')?].to_string())
}

/// `"key": number` from plant.json
fn json_number(json: &str, key: &str) -> Option<f64> {
    let rest = &json[json.find(&format!("\"{key}\""))? + key.len() + 2..];
    let rest = rest.trim_start_matches(|c: char| c == ':' || c.is_whitespace());
    let end = rest
        .find(|c: char| !(c.is_ascii_digit() || matches!(c, '.' | '-' | '+' | 'e' | 'E')))
        .unwrap_or(rest.len());
    rest[..end].parse().ok()
}
