use std::{
    collections::HashSet,
    fs::{self, OpenOptions},
    io::{self, Write},
    path::Path,
    sync::atomic::{AtomicU64, Ordering},
    time::{SystemTime, UNIX_EPOCH},
};

const INVENTORY_HEADER: &str = "LUNA_SAMS_V1 ";
static TEMP_COUNTER: AtomicU64 = AtomicU64::new(0);

/// Reserve an epoch before any revision is sent. Never reuse an epoch after reboot.
pub(crate) fn reserve(dir: &Path) -> io::Result<u64> {
    fs::create_dir_all(dir)?;
    let mut latest = 0u64;
    for entry in fs::read_dir(dir)? {
        let entry = entry?;
        let name = entry.file_name();
        if let Some(value) = name.to_str().and_then(|s| s.strip_prefix("epoch-")) {
            latest = latest.max(value.parse::<u64>().map_err(|_| {
                io::Error::new(io::ErrorKind::InvalidData, "invalid revision epoch")
            })?);
        }
    }
    let next = latest
        .checked_add(1)
        .ok_or_else(|| io::Error::new(io::ErrorKind::Other, "revision epoch exhausted"))?;
    let file = OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(dir.join(format!("epoch-{next}")))?;
    file.sync_all()?;
    #[cfg(unix)]
    fs::File::open(dir)?.sync_all()?;
    Ok(next)
}

fn inventory_hash(bytes: &[u8]) -> u64 {
    bytes.iter().fold(0xcbf29ce484222325u64, |hash, byte| {
        (hash ^ u64::from(*byte)).wrapping_mul(0x100000001b3)
    })
}

/// An atomic snapshot is authoritative once written. Older two-file inventories
/// are read for migration, never modified in place.
pub(crate) fn load_required(dir: &Path) -> io::Result<HashSet<String>> {
    let path = dir.join("required-sams");
    let file = fs::File::open(&path)?;
    if !file.metadata()?.is_file() {
        return Err(io::Error::new(io::ErrorKind::InvalidData, "invalid SAM inventory"));
    }
    let contents = fs::read_to_string(path)?;
    let (state, body) = if let Some(snapshot) = contents.strip_prefix(INVENTORY_HEADER) {
        let (metadata, body) = snapshot.split_once('\n').ok_or_else(|| {
            io::Error::new(io::ErrorKind::InvalidData, "incomplete SAM inventory snapshot")
        })?;
        (metadata.to_string(), body)
    } else {
        // Preserve the previously deployed inventory before its first atomic update.
        let state = fs::read_to_string(dir.join("required-sams-state"))?;
        let state = state.strip_suffix('\n').ok_or_else(|| {
            io::Error::new(io::ErrorKind::InvalidData, "invalid SAM inventory state")
        })?;
        (state.to_string(), contents.as_str())
    };
    if state.len() != 32 || !state.is_ascii() {
        return Err(io::Error::new(io::ErrorKind::InvalidData, "invalid SAM inventory state"));
    }
    let parse = |bytes: &[u8]| -> io::Result<u64> {
        u64::from_str_radix(
            std::str::from_utf8(bytes).map_err(|_| io::ErrorKind::InvalidData)?,
            16,
        )
        .map_err(|_| io::Error::new(io::ErrorKind::InvalidData, "invalid SAM inventory state"))
    };
    let (expected, checksum) = (parse(&state.as_bytes()[..16])?, parse(&state.as_bytes()[16..32])?);
    if inventory_hash(body.as_bytes()) != checksum {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "damaged SAM inventory",
        ));
    }
    if !body.is_empty() && !body.ends_with('\n') {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "incomplete SAM inventory",
        ));
    }
    let mut boards = HashSet::new();
    for hex in body.lines() {
        if hex.is_empty() || hex.len() % 2 != 0 {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "invalid SAM inventory entry",
            ));
        }
        let bytes: Vec<u8> = (0..hex.len())
            .step_by(2)
            .map(|i| {
                u8::from_str_radix(&hex[i..i + 2], 16).map_err(|_| {
                    io::Error::new(io::ErrorKind::InvalidData, "invalid SAM inventory entry")
                })
            })
            .collect::<io::Result<_>>()?;
        let id = String::from_utf8(bytes).map_err(|_| {
            io::Error::new(io::ErrorKind::InvalidData, "invalid SAM inventory entry")
        })?;
        if !id.starts_with("sam") {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "invalid SAM inventory entry",
            ));
        }
        if !boards.insert(id) {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "duplicate SAM inventory entry",
            ));
        }
    }
    if boards.len() as u64 != expected {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "incomplete SAM inventory",
        ));
    }
    Ok(boards)
}

/// Persist new requirements before any command that could change a SAM's configuration.
pub(crate) fn record_required(
    dir: &Path,
    known: &mut HashSet<String>,
    boards: impl IntoIterator<Item = String>,
) -> io::Result<()> {
    if load_required(dir)? != *known {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "SAM inventory changed",
        ));
    }
    let mut updated = known.clone();
    for id in boards {
        if !id.starts_with("sam") {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "invalid SAM ID",
            ));
        }
        updated.insert(id);
    }
    let path = dir.join("required-sams");
    if *known == updated && fs::read_to_string(&path)?.starts_with(INVENTORY_HEADER) {
        return Ok(());
    }

    let mut ids: Vec<_> = updated.iter().collect();
    ids.sort();
    let mut body = String::new();
    for id in ids {
        for byte in id.bytes() {
            use std::fmt::Write as _;
            write!(body, "{byte:02x}").expect("writing to String cannot fail");
        }
        body.push('\n');
    }
    let snapshot = format!(
        "{INVENTORY_HEADER}{:016x}{:016x}\n{body}",
        updated.len(),
        inventory_hash(body.as_bytes())
    );
    let nonce = SystemTime::now().duration_since(UNIX_EPOCH)
        .map_err(|_| io::Error::new(io::ErrorKind::Other, "system clock unavailable"))?
        .as_nanos();
    let temporary = dir.join(format!(
        "required-sams.tmp-{}-{nonce}-{}",
        std::process::id(),
        TEMP_COUNTER.fetch_add(1, Ordering::Relaxed)
    ));
    let result = (|| -> io::Result<()> {
        let mut file = OpenOptions::new().write(true).create_new(true).open(&temporary)?;
        file.set_permissions(fs::metadata(&path)?.permissions())?;
        file.write_all(snapshot.as_bytes())?;
        file.sync_all()?;
        fs::rename(&temporary, &path)?;
        #[cfg(unix)]
        fs::File::open(dir)?.sync_all()?;
        Ok(())
    })();
    if result.is_err() {
        let _ = fs::remove_file(&temporary);
    }
    result?;
    *known = updated;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn consecutive_boots_reserve_increasing_epochs_without_clock_ordering() {
        let dir = std::env::temp_dir().join(format!(
            "luna-epoch-test-{}-{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ));
        assert_eq!(reserve(&dir).unwrap(), 1);
        assert_eq!(reserve(&dir).unwrap(), 2);
        assert_eq!(reserve(&dir).unwrap(), 3);
        fs::remove_dir_all(dir).unwrap();
    }
    #[test]
    fn corruption_fails_closed() {
        let dir = std::env::temp_dir().join(format!("luna-epoch-invalid-{}", std::process::id()));
        fs::create_dir_all(&dir).unwrap();
        fs::write(dir.join("epoch-invalid"), []).unwrap();
        assert!(reserve(&dir).is_err());
        fs::remove_dir_all(dir).unwrap();
    }

    #[test]
    fn inventory_survives_restart_and_rejects_missing_or_corrupt_state() {
        let dir = std::env::temp_dir().join(format!("luna-inventory-{}", std::process::id()));
        fs::create_dir_all(&dir).unwrap();
        assert!(load_required(&dir).is_err());
        fs::write(dir.join("required-sams"), []).unwrap();
        fs::write(
            dir.join("required-sams-state"),
            b"0000000000000000cbf29ce484222325\n",
        )
        .unwrap();
        let mut known = load_required(&dir).unwrap();
        record_required(&dir, &mut known, ["sam-old".into(), "sam-new".into()]).unwrap();
        assert_eq!(load_required(&dir).unwrap(), known);
        assert!(fs::read_to_string(dir.join("required-sams"))
            .unwrap().starts_with(INVENTORY_HEADER));
        fs::write(
            dir.join("required-sams"),
            b"73616d2d6f6c65\n73616d2d6e6577\n",
        )
        .unwrap();
        assert!(load_required(&dir).is_err()); // valid count and encoding, damaged ID
        fs::write(dir.join("required-sams"), b"zz\n").unwrap();
        assert!(load_required(&dir).is_err());
        fs::write(dir.join("required-sams"), b"73616d2d6f6c64").unwrap();
        assert!(load_required(&dir).is_err());
        fs::write(dir.join("required-sams"), b"73616d2d6f6c64\n").unwrap();
        assert!(load_required(&dir).is_err()); // valid prefix, but count still includes two boards
        fs::write(dir.join("required-sams-state"), b"corrupt\n").unwrap();
        assert!(load_required(&dir).is_err());
        fs::remove_dir_all(dir).unwrap();
    }

    #[test]
    fn atomic_snapshot_survives_obsolete_state_and_incomplete_temp_file() {
        let dir = std::env::temp_dir().join(format!("luna-inventory-atomic-{}", std::process::id()));
        fs::create_dir_all(&dir).unwrap();
        fs::write(dir.join("required-sams"), []).unwrap();
        fs::write(dir.join("required-sams-state"), b"0000000000000000cbf29ce484222325\n").unwrap();
        let mut known = load_required(&dir).unwrap();
        record_required(&dir, &mut known, ["sam-old".into()]).unwrap();
        fs::write(dir.join("required-sams-state"), b"corrupt\n").unwrap();
        fs::write(dir.join("required-sams.tmp-interrupted"), b"partial update").unwrap();
        assert_eq!(load_required(&dir).unwrap(), known);
        record_required(&dir, &mut known, ["sam-new".into()]).unwrap();
        assert_eq!(load_required(&dir).unwrap(), known);
        assert_eq!(known.len(), 2);
        let snapshot = fs::read_to_string(dir.join("required-sams")).unwrap();
        fs::write(dir.join("required-sams"), &snapshot[..snapshot.len() - 2]).unwrap();
        assert!(load_required(&dir).is_err());
        fs::remove_dir_all(dir).unwrap();
    }

    #[test]
    fn inventory_write_failure_does_not_confirm_new_board() {
        let dir =
            std::env::temp_dir().join(format!("luna-inventory-failure-{}", std::process::id()));
        fs::create_dir_all(&dir).unwrap();
        fs::write(dir.join("required-sams"), []).unwrap();
        fs::write(
            dir.join("required-sams-state"),
            b"0000000000000000cbf29ce484222325\n",
        )
        .unwrap();
        let mut known = load_required(&dir).unwrap();
        fs::remove_file(dir.join("required-sams")).unwrap();
        assert!(record_required(&dir, &mut known, ["sam-new".into()]).is_err());
        assert!(!known.contains("sam-new"));
        fs::remove_dir_all(dir).unwrap();
    }

    #[test]
    fn missing_directory_is_created_but_missing_inventory_is_unverified() {
        let dir = std::env::temp_dir()
            .join(format!("luna-epoch-missing-{}", std::process::id()))
            .join("new");
        assert_eq!(reserve(&dir).unwrap(), 1);
        assert!(load_required(&dir).is_err());
        fs::remove_dir_all(dir.parent().unwrap()).unwrap();
    }

    #[cfg(unix)]
    #[test]
    fn inventory_permission_failure_blocks_recording() {
        use std::os::unix::fs::PermissionsExt;

        let dir =
            std::env::temp_dir().join(format!("luna-epoch-permission-{}", std::process::id()));
        fs::create_dir_all(&dir).unwrap();
        fs::write(dir.join("required-sams"), []).unwrap();
        fs::write(
            dir.join("required-sams-state"),
            b"0000000000000000cbf29ce484222325\n",
        )
        .unwrap();
        let mut known = load_required(&dir).unwrap();
        fs::set_permissions(&dir, fs::Permissions::from_mode(0o500)).unwrap();
        let result = record_required(&dir, &mut known, ["sam-new".into()]);
        fs::set_permissions(&dir, fs::Permissions::from_mode(0o700)).unwrap();
        if result.is_err() {
            assert!(known.is_empty());
        } else {
            eprintln!("permission check skipped: process can write mode-0500 directories");
        }
        fs::remove_dir_all(dir).unwrap();
    }
}
