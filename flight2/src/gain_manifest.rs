//! Checks that a folder of exported controller gains is the set the flight
//! software expects, before any of it is loaded.
//!
//! The hopper repo's `export_plant_gains.m` writes the gain tables
//! (`tgrid`, `xref`, `k1flat`, `k2grid`, `unom`, each a `.csv`) next to a
//! `gains.json` that says what they are:
//!
//! ```json
//! {
//!   "controller": "lqr_ctv_tuning",
//!   "git_commit": "325194b",
//!   "k1flat_layout": "row-major",
//!   "sha256": { "tgrid.csv": "...", "xref.csv": "...", "k1flat.csv": "...",
//!               "k2grid.csv": "...", "unom.csv": "..." }
//! }
//! ```
//!
//! Loose CSV files carry no record of which controller they were designed for
//! or which plant they were tuned against, so a stale or mixed-up folder would
//! load without complaint. This refuses a folder unless
//!
//! - `controller` is the one the caller expects (`lqr_ctv_tuning` for the CTV),
//! - `k1flat_layout` is `row-major`, the layout the controllers unpack,
//! - every table has a `sha256` entry and the file on disk matches it,
//! - `git_commit` does not end in `-dirty`, unless the caller allows it, and
//! - it equals the commit the plant was generated from, when the caller has
//!   one (the sim-plant bench passes `plant.json`'s `git_commit`).
//!
//! The key names above are what the hopper side has described; they have not
//! been checked against a real export yet. If `export_plant_gains.m` uses
//! different ones, `verify` fails on the first missing key, naming it.

use std::{fs, path::Path};

use serde_json::Value;
use sha2::{Digest, Sha256};

/// The tables a gain folder must contain, without the `.csv`.
pub const TABLES: [&str; 5] = ["tgrid", "xref", "k1flat", "k2grid", "unom"];

/// The K1 layout the controllers unpack: all of input 0's state gains first.
const K1_LAYOUT: &str = "row-major";

/// What a gain folder has to match.
#[derive(Debug, Clone, Copy)]
pub struct Policy<'a> {
    /// The controller the gains must have been designed for.
    pub controller: &'a str,
    /// The commit of the plant the gains will fly, if there is one. The
    /// gains' own `git_commit` must be the same.
    pub plant_commit: Option<&'a str>,
    /// Accept a `-dirty` commit (gains exported from a modified hopper tree).
    /// Flight builds leave this off.
    pub allow_dirty: bool,
}

/// What `verify` learned about an accepted gain folder.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Manifest {
    pub controller: String,
    pub git_commit: String,
}

/// Reads `<dir>/gains.json` and checks it, and the tables next to it, against
/// `policy`. The error names what is wrong.
pub fn verify(dir: &Path, policy: &Policy) -> Result<Manifest, String> {
    let path = dir.join("gains.json");
    let text = fs::read_to_string(&path).map_err(|e| {
        format!(
            "{}: {e} (the gain folder needs a gains.json)",
            path.display()
        )
    })?;
    let json: Value = serde_json::from_str(&text)
        .map_err(|e| format!("{}: not valid JSON: {e}", path.display()))?;
    let name = path.display().to_string();

    let controller = string(&json, "controller", &name)?;
    if controller != policy.controller {
        return Err(format!(
            "{name}: the gains are for controller {controller:?}, but {:?} is expected",
            policy.controller
        ));
    }

    let layout = string(&json, "k1flat_layout", &name)?;
    if layout != K1_LAYOUT {
        return Err(format!(
            "{name}: k1flat_layout is {layout:?}, but the controllers unpack {K1_LAYOUT:?}"
        ));
    }

    let hashes = json
        .get("sha256")
        .and_then(Value::as_object)
        .ok_or_else(|| format!("{name}: no \"sha256\" object with one hash per table"))?;
    for table in TABLES {
        let file = format!("{table}.csv");
        // Accept the key with or without the extension
        let want = hashes
            .get(&file)
            .or_else(|| hashes.get(table))
            .and_then(Value::as_str)
            .ok_or_else(|| format!("{name}: \"sha256\" has no entry for {file}"))?;
        let table_path = dir.join(&file);
        let bytes = fs::read(&table_path).map_err(|e| format!("{}: {e}", table_path.display()))?;
        let got = hex(&Sha256::digest(&bytes));
        if !got.eq_ignore_ascii_case(want.trim()) {
            return Err(format!(
                "{}: sha256 is {got}, but gains.json says {want}. The file changed after the \
                 gains were exported",
                table_path.display()
            ));
        }
    }

    let git_commit = string(&json, "git_commit", &name)?;
    if git_commit.ends_with("-dirty") && !policy.allow_dirty {
        return Err(format!(
            "{name}: the gains were exported from a modified hopper tree ({git_commit}); \
             export them again from a clean commit"
        ));
    }
    if let Some(plant) = policy.plant_commit {
        if plant.ends_with("-dirty") && !policy.allow_dirty {
            return Err(format!(
                "the plant was generated from a modified hopper tree ({plant}); \
                 rebuild it from a clean commit"
            ));
        }
        if plant != git_commit {
            return Err(format!(
                "{name}: the gains come from hopper commit {git_commit}, but the plant was \
                 generated from {plant}. Gains tuned against one plant must not fly another"
            ));
        }
    }

    Ok(Manifest {
        controller,
        git_commit,
    })
}

fn string(json: &Value, key: &str, name: &str) -> Result<String, String> {
    match json.get(key).and_then(Value::as_str) {
        Some(value) if !value.trim().is_empty() => Ok(value.to_string()),
        _ => Err(format!("{name}: no \"{key}\" string")),
    }
}

fn hex(bytes: &[u8]) -> String {
    bytes.iter().map(|b| format!("{b:02x}")).collect()
}

#[cfg(test)]
mod tests {
    use tempfile::{tempdir, TempDir};

    use super::*;

    const POLICY: Policy = Policy {
        controller: "lqr_ctv_tuning",
        plant_commit: None,
        allow_dirty: false,
    };

    /// A gain folder with valid tables and a gains.json built from `edit`.
    fn folder(edit: impl FnOnce(&mut Value)) -> TempDir {
        let dir = tempdir().unwrap();
        let mut hashes = serde_json::Map::new();

        for table in TABLES {
            let file = format!("{table}.csv");
            let bytes = format!("contents of {table}\n");

            fs::write(dir.path().join(&file), &bytes).unwrap();
            hashes.insert(file, Value::from(hex(&Sha256::digest(bytes.as_bytes()))));
        }

        let mut json = serde_json::json!({
            "controller": "lqr_ctv_tuning",
            "git_commit": "325194b",
            "k1flat_layout": "row-major",
            "sha256": hashes,
        });

        edit(&mut json);
        fs::write(dir.path().join("gains.json"), json.to_string()).unwrap();

        dir
    }

    #[test]
    fn accepts_a_matching_folder() {
        let dir = folder(|_| {});
        let manifest = verify(dir.path(), &POLICY).unwrap();

        assert_eq!(manifest.controller, "lqr_ctv_tuning");
        assert_eq!(manifest.git_commit, "325194b");
    }

    #[test]
    fn rejects_the_wrong_controller() {
        let dir = folder(|j| j["controller"] = "lqr_sched".into());
        let error = verify(dir.path(), &POLICY).unwrap_err();

        assert!(error.contains("\"lqr_sched\""), "{error}");
        assert!(error.contains("\"lqr_ctv_tuning\""), "{error}");
    }

    #[test]
    fn a_different_expected_controller_is_an_explicit_choice() {
        let dir = folder(|j| j["controller"] = "lqr_sched".into());
        let policy = Policy {
            controller: "lqr_sched",
            ..POLICY
        };

        assert!(verify(dir.path(), &policy).is_ok());
    }

    #[test]
    fn rejects_a_missing_controller_key() {
        let dir = folder(|j| {
            j.as_object_mut().unwrap().remove("controller");
        });

        assert!(verify(dir.path(), &POLICY)
            .unwrap_err()
            .contains("controller"));
    }

    #[test]
    fn rejects_a_column_major_k1() {
        let dir = folder(|j| j["k1flat_layout"] = "column-major".into());

        assert!(verify(dir.path(), &POLICY)
            .unwrap_err()
            .contains("column-major"));
    }

    #[test]
    fn rejects_a_table_that_changed_after_export() {
        let dir = folder(|_| {});

        fs::write(dir.path().join("k2grid.csv"), "edited by hand\n").unwrap();

        let error = verify(dir.path(), &POLICY).unwrap_err();

        assert!(error.contains("k2grid.csv"), "{error}");
        assert!(error.contains("sha256"), "{error}");
    }

    #[test]
    fn rejects_a_table_with_no_hash() {
        let dir = folder(|j| {
            j["sha256"].as_object_mut().unwrap().remove("unom.csv");
        });

        assert!(verify(dir.path(), &POLICY)
            .unwrap_err()
            .contains("unom.csv"));
    }

    #[test]
    fn rejects_a_missing_table() {
        let dir = folder(|_| {});

        fs::remove_file(dir.path().join("xref.csv")).unwrap();

        assert!(verify(dir.path(), &POLICY)
            .unwrap_err()
            .contains("xref.csv"));
    }

    #[test]
    fn accepts_hashes_keyed_without_the_extension_and_in_capitals() {
        let dir = folder(|j| {
            let hashes = j["sha256"].as_object_mut().unwrap();
            let k = hashes.remove("tgrid.csv").unwrap();

            hashes.insert("tgrid".into(), k.as_str().unwrap().to_uppercase().into());
        });

        assert!(verify(dir.path(), &POLICY).is_ok());
    }

    #[test]
    fn rejects_a_folder_without_gains_json() {
        let dir = tempdir().unwrap();

        assert!(verify(dir.path(), &POLICY)
            .unwrap_err()
            .contains("gains.json"));
    }

    #[test]
    fn rejects_dirty_gains_unless_allowed() {
        let dir = folder(|j| j["git_commit"] = "325194b-dirty".into());

        assert!(verify(dir.path(), &POLICY)
            .unwrap_err()
            .contains("modified"));

        let allowed = Policy {
            allow_dirty: true,
            ..POLICY
        };

        assert!(verify(dir.path(), &allowed).is_ok());
    }

    #[test]
    fn rejects_gains_from_a_different_commit_than_the_plant() {
        let dir = folder(|_| {});
        let policy = Policy {
            plant_commit: Some("deadbee"),
            ..POLICY
        };
        let error = verify(dir.path(), &policy).unwrap_err();

        assert!(error.contains("325194b"), "{error}");
        assert!(error.contains("deadbee"), "{error}");
    }

    #[test]
    fn accepts_gains_from_the_plants_commit() {
        let dir = folder(|_| {});
        let policy = Policy {
            plant_commit: Some("325194b"),
            ..POLICY
        };

        assert!(verify(dir.path(), &policy).is_ok());
    }

    #[test]
    fn rejects_a_dirty_plant_unless_allowed() {
        let dir = folder(|j| j["git_commit"] = "325194b-dirty".into());
        let strict = Policy {
            plant_commit: Some("325194b-dirty"),
            ..POLICY
        };

        assert!(verify(dir.path(), &strict).is_err());

        let allowed = Policy {
            allow_dirty: true,
            ..strict
        };

        assert!(verify(dir.path(), &allowed).is_ok());
    }
}
