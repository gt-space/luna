# fs-protobuf-rust
Rust crate for compiling and exporting fs-protobuf messages

## Getting started

1. Add this repo as a submodule `git submodule add git@github-research.gatech.edu:YJSP/fs-protobuf-rust.git`

2. run `git submodule update --init --recursive --remote` to pull the latest version of the mcfs protocol buffer files
3. Add the dependencies to Cargo.toml
```
[dependencies]
quick-protobuf = "0.8.0"
fs-protobuf-rust = { path = "fs-protobuf-rust" }
```

See test cases in `tests` for examples of serialization and deserialization
