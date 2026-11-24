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
//! The plant must have been built with `CONTROLLER=fsw`, so that it takes the
//! flight software's command instead of running a controller of its own.
//! `plant.json` names the controller (`"controller": "fsw"`), and the build
//! fails if it is missing or says anything else. The name is checked, never a
//! variant index, because the indices differ between hopper branches.
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

use serde_json::Value;

fn main() {
    let manifest = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    let out = PathBuf::from(env::var("OUT_DIR").unwrap());
    println!("cargo:rerun-if-env-changed=HOPPER_PLANT_CODE");
    println!("cargo::rustc-check-cfg=cfg(hopper_plant_stub)");
    let Some(code) = env::var_os("HOPPER_PLANT_CODE").map(PathBuf::from) else {
        println!("cargo:rustc-cfg=hopper_plant_stub");
        write_consts(
            &out,
            &Consts {
                step: 0.001,
                period: 0.01,
                info_path: "(stub: HOPPER_PLANT_CODE was not set)".to_string(),
                controller: "(stub)".to_string(),
                git_commit: "(stub)".to_string(),
                sensors: false,
                sensor_period: 0.0,
            },
        );
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

    let info_text = read("plant.json");
    let info: Value = serde_json::from_str(&info_text).unwrap_or_else(|e| {
        panic!(
            "{} is not valid JSON ({e})",
            code.join("plant.json").display()
        )
    });

    // The plant has to pass the flight software's command through.
    let controller = info_str(&info, "controller").unwrap_or_else(|| {
        panic!(
            "plant.json has no top-level \"controller\" name. It was built by a build_plant_c \
             that does not record it; regenerate the plant with a current hopper \
             fsw_bridge/build_plant_c.m (CONTROLLER=fsw)"
        )
    });
    assert!(
        controller == "fsw",
        "plant.json says the plant was built with controller {controller:?}, not \"fsw\". \
         Such a plant runs its own controller and would ignore the flight software's \
         command, so any SIL result from it is meaningless. Rebuild it with \
         build_plant_c('Variants', variantConfig('VEHICLE', 'ctv', 'CONTROLLER', 'fsw'))"
    );
    let git_commit = info_str(&info, "git_commit")
        .expect("plant.json has no top-level \"git_commit\" (hopper commit the plant was built from)");

    let target = env::var("CARGO_CFG_TARGET_ARCH").unwrap();
    let built_for = info_str(&info, "target").unwrap_or_default();
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
    // The sensor outputs, if the plant has them, in the order this crate reads
    let sensors_claimed = info.get("sensors").and_then(Value::as_bool).unwrap_or(false);
    if sensors_claimed {
        let names = |key: &str| -> Vec<String> {
            info.get("outputs")
                .and_then(|o| o.get(key))
                .and_then(Value::as_array)
                .map(|a| a.iter().filter_map(|v| v.as_str().map(str::to_string)).collect())
                .unwrap_or_default()
        };
        assert!(
            names("sens_meas") == SENSOR_CHANNELS && names("sens_valid") == SENSOR_FLAGS,
            "plant.json's sensor outputs are not in the order lib.rs reads (sensor_layout.m);              sens_meas must be {SENSOR_CHANNELS:?} and sens_valid {SENSOR_FLAGS:?}"
        );
    }
    let sensor_period = info_number(&info, "sensor_step");
    let step = info_number(&info, "fixed_step").expect("plant.json has no numeric top-level fixed_step");
    let period = info_number(&info, "fsw_period").expect("plant.json has no numeric top-level fsw_period");
    write_consts(
        &out,
        &Consts {
            step,
            period,
            info_path: code.join("plant.json").display().to_string(),
            controller,
            git_commit,
            sensors: sensors_claimed,
            sensor_period: sensor_period.unwrap_or(0.0),
        },
    );

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
    // Which model globals this plant has. A plant built without block I/O, for
    // one, has no hopper_6dof_B, and the shim must not touch what isn't there.
    let header = fs::read_to_string(src.join("hopper_6dof.h")).unwrap_or_else(|e| {
        panic!("cannot read {} ({e})", src.join("hopper_6dof.h").display())
    });
    if header.contains(" sens_meas[") {
        build.define("PLANT_HAS_SENSORS", None);
    } else {
        assert!(
            !sensors_claimed,
            "plant.json says the plant has sensor outputs, but hopper_6dof.h has no sens_meas"
        );
    }
    for (suffix, define) in [
        ("B", "PLANT_HAS_B"),
        ("X", "PLANT_HAS_X"),
        ("XDis", "PLANT_HAS_XDIS"),
        ("DW", "PLANT_HAS_DW"),
    ] {
        if header.contains(&format!(" hopper_6dof_{suffix};")) {
            build.define(define, None);
        }
    }
    build
        .file(manifest.join("csrc/plant_shim.c"))
        .include(&src)
        .warnings(false) // generated code; not ours to clean up
        .compile("hopper_plant_c");
    println!("cargo:rustc-link-lib=m");
}

/// What lib.rs gets about the plant it was built from
struct Consts {
    step: f64,
    period: f64,
    info_path: String,
    controller: String,
    git_commit: String,
    sensors: bool,
    sensor_period: f64,
}

/// `sens_meas`, in the order of hopper's `sensor_layout.m`
const SENSOR_CHANNELS: [&str; 26] = [
    "accel_x", "accel_y", "accel_z", "gyro_x", "gyro_y", "gyro_z", "baro_alt", "lidar_1",
    "lidar_2", "lidar_3", "lidar_4", "gps_pos_n", "gps_pos_e", "gps_pos_d", "gps_fix", "gps_sats",
    "gps_age", "mag_x", "mag_y", "mag_z", "gps_vel_n", "gps_vel_e", "gps_vel_d", "dgps_n", "dgps_e",
    "dgps_d",
];

/// `sens_valid` and `sens_stamp`, one per sensor
const SENSOR_FLAGS: [&str; 9] = [
    "imu", "baro", "lidar_1", "lidar_2", "lidar_3", "lidar_4", "gps", "mag", "dgps",
];

/// STEP, FSW_PERIOD, INFO_PATH, PLANT_CONTROLLER and PLANT_GIT_COMMIT for lib.rs
fn write_consts(out: &Path, c: &Consts) {
    let consts = [
        "/// Plant step, s".to_string(),
        format!("pub const STEP: f64 = {:?};", c.step),
        "/// Flight-software control period the plant was built for, s".to_string(),
        format!("pub const FSW_PERIOD: f64 = {:?};", c.period),
        "/// `plant.json` the library was built from (what vehicle, which commit)".to_string(),
        format!("pub const INFO_PATH: &str = {:?};", c.info_path),
        "/// Controller the plant was built with. The build fails unless it is `fsw`".to_string(),
        format!("pub const PLANT_CONTROLLER: &str = {:?};", c.controller),
        "/// Hopper commit the plant was generated from, with `-dirty` if the tree was".to_string(),
        format!("pub const PLANT_GIT_COMMIT: &str = {:?};", c.git_commit),
        "/// The plant has the Sensors block's outputs (`Plant::sensors`)".to_string(),
        format!("pub const HAS_SENSORS: bool = {:?};", c.sensors),
        "/// The Sensors block's step, s: its fastest sensor's period".to_string(),
        format!("pub const SENSOR_STEP: f64 = {:?};", c.sensor_period),
    ];
    fs::write(out.join("plant_consts.rs"), consts.join("\n") + "\n").unwrap();
}

/// `"key": "value"` at the top level of plant.json
fn info_str(info: &Value, key: &str) -> Option<String> {
    info.get(key)?.as_str().map(str::to_string)
}

/// `"key": number` at the top level of plant.json
fn info_number(info: &Value, key: &str) -> Option<f64> {
    info.get(key)?.as_f64()
}
