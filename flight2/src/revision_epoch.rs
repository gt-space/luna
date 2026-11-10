use std::{
    collections::HashSet,
    fs::{self, OpenOptions},
    io::{self, Seek, SeekFrom, Write},
    path::Path,
};

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

/// Both inventory files must be provisioned on first deployment. Missing data is not an empty inventory.
pub(crate) fn load_required(dir: &Path) -> io::Result<HashSet<String>> {
    let state = fs::read(dir.join("required-sams-state"))?;
    if state.len() != 33 || state[32] != b'\n' {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "invalid SAM inventory state",
        ));
    }
    let parse = |bytes: &[u8]| -> io::Result<u64> {
        u64::from_str_radix(
            std::str::from_utf8(bytes).map_err(|_| io::ErrorKind::InvalidData)?,
            16,
        )
        .map_err(|_| io::Error::new(io::ErrorKind::InvalidData, "invalid SAM inventory state"))
    };
    let (expected, checksum) = (parse(&state[..16])?, parse(&state[16..32])?);
    let path = dir.join("required-sams");
    let file = fs::File::open(&path)?;
    if !file.metadata()?.is_file() {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "invalid SAM inventory",
        ));
    }
    let contents = fs::read_to_string(path)?;
    if inventory_hash(contents.as_bytes()) != checksum {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "damaged SAM inventory",
        ));
    }
    if !contents.is_empty() && !contents.ends_with('\n') {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "incomplete SAM inventory",
        ));
    }
    let mut boards = HashSet::new();
    for hex in contents.lines() {
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
    let mut file = OpenOptions::new()
        .append(true)
        .open(dir.join("required-sams"))?;
    if !file.metadata()?.is_file() {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "invalid SAM inventory",
        ));
    }
    let mut state = OpenOptions::new()
        .write(true)
        .open(dir.join("required-sams-state"))?;
    let mut contents = fs::read(dir.join("required-sams"))?;
    for id in boards {
        if known.contains(&id) {
            continue;
        }
        if !id.starts_with("sam") {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "invalid SAM ID",
            ));
        }
        let next = (known.len() as u64)
            .checked_add(1)
            .ok_or_else(|| io::Error::new(io::ErrorKind::Other, "SAM inventory exhausted"))?;
        let line = format!(
            "{}\n",
            id.as_bytes()
                .iter()
                .map(|byte| format!("{byte:02x}"))
                .collect::<String>()
        );
        contents.extend_from_slice(line.as_bytes());
        state.seek(SeekFrom::Start(0))?;
        write!(state, "{next:016x}{:016x}\n", inventory_hash(&contents))?;
        state.sync_all()?;
        file.write_all(line.as_bytes())?;
        file.sync_all()?;
        known.insert(id);
    }
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
        fs::set_permissions(dir.join("required-sams"), fs::Permissions::from_mode(0o400)).unwrap();
        let result = record_required(&dir, &mut known, ["sam-new".into()]);
        fs::set_permissions(dir.join("required-sams"), fs::Permissions::from_mode(0o600)).unwrap();
        if result.is_err() {
            assert!(known.is_empty());
        } else {
            eprintln!("permission check skipped: process can write mode-0400 files");
        }
        fs::remove_dir_all(dir).unwrap();
    }
}
