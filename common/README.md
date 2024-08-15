# common

## Example

You can serialize a struct with the `serde::Serialize` trait implemented with the following code. Note that the `alloc` feature of postcard must be enabled to use `to_allocvec`.

```rust
let buffer = postcard::to_allocvec(&vehicle_state);
```

Deserializing is similarly simple on a struct with `serde::Deserialize` implemented:

```rust
let vehicle_state = postcard::from_bytes::<VehicleState>(&buffer);
```

For these examples, the postcard crate must be included separately as a dependency; this library does not supply it.
