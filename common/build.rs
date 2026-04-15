use std::{
  collections::hash_map::DefaultHasher,
  fs,
  hash::{Hash, Hasher},
  path::{Path, PathBuf},
};

fn main() {
  let manifest_dir = PathBuf::from(std::env::var("CARGO_MANIFEST_DIR").unwrap());
  let src_dir = manifest_dir.join("src");

  let mut files = Vec::new();
  collect_rust_files(&src_dir, &mut files);
  files.sort();

  let mut hasher = DefaultHasher::new();
  for file in files {
    println!("cargo:rerun-if-changed={}", file.display());
    file.to_string_lossy().hash(&mut hasher);
    fs::read(&file).unwrap().hash(&mut hasher);
  }

  println!("cargo:rustc-env=COMMON_LAYOUT_FINGERPRINT={:016x}", hasher.finish());
}

fn collect_rust_files(dir: &Path, files: &mut Vec<PathBuf>) {
  for entry in fs::read_dir(dir).unwrap() {
    let entry = entry.unwrap();
    let path = entry.path();
    if path.is_dir() {
      collect_rust_files(&path, files);
    } else if path.extension().is_some_and(|ext| ext == "rs") {
      files.push(path);
    }
  }
}
