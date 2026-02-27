# 001 — Create `apriltag-detect` crate skeleton

## Goal
Establish the detection library crate with Cargo.toml, empty lib.rs, and workspace membership.

## Preconditions
- Workspace has 3 crates: `apriltag`, `apriltag-gen`, `apriltag-gen-cli`
- `Cargo.toml` workspace members list has 3 entries

## Postconditions
- `cargo check -p apriltag-detect` passes
- Workspace `Cargo.toml` has 4 members
- `apriltag-detect/Cargo.toml` exists with `apriltag` path dependency
- Features `std` (default) and `parallel` (optional rayon) defined

## Description
Create `apriltag-detect/Cargo.toml`:
```toml
[package]
name = "apriltag-detect"
version = "0.1.0"
edition = "2021"
description = "AprilTag detection pipeline"

[dependencies]
apriltag = { path = "../apriltag" }

[features]
default = ["std"]
std = []
parallel = ["rayon"]

[dependencies.rayon]
version = "1.10"
optional = true
```

Create `apriltag-detect/src/lib.rs` (empty or with a doc comment).

Add `"apriltag-detect"` to workspace members in root `Cargo.toml`.

## References
- `Cargo.toml:1-3` — existing workspace definition
- `apriltag/Cargo.toml` — pattern for crate manifest
