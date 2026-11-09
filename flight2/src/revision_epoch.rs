use std::{fs::{self, OpenOptions}, io, path::Path};

/// Reserve an epoch before any revision is sent. Never reuse an epoch after reboot.
pub(crate) fn reserve(dir: &Path) -> io::Result<u64> {
    fs::create_dir_all(dir)?;
    let mut latest = 0u64;
    for entry in fs::read_dir(dir)? {
        let entry = entry?;
        let name = entry.file_name();
        if let Some(value) = name.to_str().and_then(|s| s.strip_prefix("epoch-")) {
            latest = latest.max(value.parse::<u64>().map_err(|_| io::Error::new(io::ErrorKind::InvalidData, "invalid revision epoch"))?);
        }
    }
    let next = latest.checked_add(1).ok_or_else(|| io::Error::new(io::ErrorKind::Other, "revision epoch exhausted"))?;
    let file = OpenOptions::new().write(true).create_new(true).open(dir.join(format!("epoch-{next}")))?;
    file.sync_all()?;
    #[cfg(unix)]
    fs::File::open(dir)?.sync_all()?;
    Ok(next)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn reboot_and_clock_rollback_do_not_reuse_revision() {
        let dir = std::env::temp_dir().join(format!("luna-epoch-test-{}-{}", std::process::id(), std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap().as_nanos()));
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
}
