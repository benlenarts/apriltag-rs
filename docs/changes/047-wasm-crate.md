# 047 — Create `apriltag-wasm` crate skeleton

## Goal
Establish the WASM bindings crate with wasm-bindgen, tsify-next, and serde-wasm-bindgen dependencies.

## Preconditions
- 042 complete: `Detector` fully functional in `apriltag-detect`

## Postconditions
- `cargo build --target wasm32-unknown-unknown -p apriltag-wasm` compiles
- `crate-type = ["cdylib"]` in Cargo.toml
- Workspace `Cargo.toml` updated with new member

## Description
Create `apriltag-wasm/Cargo.toml`:
```toml
[package]
name = "apriltag-wasm"
version = "0.1.0"
edition = "2021"
description = "AprilTag detection WASM bindings"

[lib]
crate-type = ["cdylib"]

[dependencies]
apriltag = { path = "../apriltag" }
apriltag-detect = { path = "../apriltag-detect" }
wasm-bindgen = "0.2"
tsify-next = { version = "0.5", features = ["js"] }
serde = { version = "1", features = ["derive"] }
serde-wasm-bindgen = "0.6"
js-sys = "0.3"
```

Create `apriltag-wasm/src/lib.rs` with initial `use` declarations.

Add `"apriltag-wasm"` to workspace members.

## References
- Architecture decision — WASM crate with wasm-bindgen + tsify-next
