use std::{
  env,
  ffi::OsString,
  fs,
  hash::Hasher,
  io::{self, Write},
  path::{Path, PathBuf},
  sync::OnceLock,
};
use tempfile::Builder;
use wyhash::WyHash;

// Directory that holds the built libcommon.so file
static COMMON_SO_DIR: OnceLock<PathBuf> = OnceLock::new();
// Bytes of the built libcommon.so file
static COMMON_SO_BYTES: &[u8] = include_bytes!(env!("COMMON_SO_SOURCE_PATH"));

/// Ensures that a copy of the built libcommon.so file has been created in a 
/// temporary directory on disk.
pub(crate) fn materialize_common_so() -> io::Result<&'static PathBuf> {
  if let Some(path) = COMMON_SO_DIR.get() {
    return Ok(path);
  }

  // Get the path to the temporary directory that holds the copied libcommon.so file
  let extracted = extract_common_so()?;
  let _ = COMMON_SO_DIR.set(extracted);

  Ok(COMMON_SO_DIR.get().expect("common.so path should be initialized"))
}

/// Returns the directory that already holds the extracted `common.so` file.
///
/// This should only be called after `materialize_common_so()` has succeeded
/// during program startup.
pub(crate) fn common_so_dir() -> &'static PathBuf {
  COMMON_SO_DIR
    .get()
    .expect("common.so must be materialized before Python sequences run")
}

/// Builds the `PYTHONPATH` value for Python processes that need to import
/// `common` from the extracted shared library directory.
pub(crate) fn python_path_for(common_so_dir: &Path) -> io::Result<OsString> {
  let mut paths = vec![common_so_dir.to_path_buf()];

  if let Some(existing) = env::var_os("PYTHONPATH") {
    paths.extend(env::split_paths(&existing));
  }

  env::join_paths(paths)
    .map_err(|e| io::Error::new(io::ErrorKind::InvalidInput, e))
}

/// Extracts the built libcommon.so file from the temporary directory and returns
/// the path to the file. If the temporary directory does not exist, it is created
/// and the bytes are copied to it.
fn extract_common_so() -> io::Result<PathBuf> {
  // Create the temporary directory to hold the built libcommon.so file
  let dir = env::temp_dir()
    .join(format!("flight-computer-common-{:016x}", common_so_hash(),));
  fs::create_dir_all(&dir)?;
  set_dir_permissions(&dir)?;

  // Check if the built libcommon.so file already exists in the temporary directory
  let so_path = dir.join("common.so");
  if library_matches(&so_path)? {
    Ok(dir)
  } else {
    write_library(&dir, &so_path)?;

    if library_matches(&so_path)? {
      Ok(dir)
    } else {
      Err(io::Error::other(format!(
        "materialized library at '{}' did not match embedded bytes",
        so_path.display()
      )))
    }
  }
}

/// Hashes the bytes of the built libcommon.so file.
fn common_so_hash() -> u64 {
  let mut hasher = WyHash::default();
  hasher.write(COMMON_SO_BYTES);
  hasher.finish()
}

/// Checks if the bytes of the built libcommon.so file at `path` match the
/// embedded bytes from `COMMON_SO_BYTES`. Returns `true` if a file exists at
/// `path` and its contents match the embedded bytes, `false` if a file
/// does not exist at `path`, or an error if the file cannot be read / does not
/// match the embedded bytes.
fn library_matches(path: &Path) -> io::Result<bool> {
  match fs::read(path) {
    Ok(existing) => Ok(existing == COMMON_SO_BYTES),
    Err(e) if e.kind() == io::ErrorKind::NotFound => Ok(false),
    Err(e) => Err(e),
  }
}

/// Writes the embedded bytes of the built libcommon.so file to a temporary file
/// in `dir` and returns the path to the file.
fn write_library(dir: &Path, so_path: &Path) -> io::Result<()> {
  let mut temp_file = Builder::new()
    .prefix(".common-")
    .suffix(".so")
    .tempfile_in(dir)?;

  temp_file.write_all(COMMON_SO_BYTES)?;
  temp_file.as_file().sync_all()?;

  temp_file.persist(so_path).map(|_| ()).map_err(|e| e.error)
}

/// Sets the permissions of `dir` to rwx for the owner.
#[cfg(unix)]
fn set_dir_permissions(dir: &Path) -> io::Result<()> {
  use std::os::unix::fs::PermissionsExt;

  fs::set_permissions(dir, fs::Permissions::from_mode(0o700))
}

#[cfg(not(unix))]
fn set_dir_permissions(_dir: &Path) -> io::Result<()> {
  Ok(())
}
