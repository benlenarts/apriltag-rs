# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- Core AprilTag detection library (`apriltag`)
- Tag family generation library (`apriltag-gen`) with codegen and upgrade modules
- CLI tools for generation (`apriltag-gen-cli`) and detection (`apriltag-detect-cli`)
- WASM bindings for detection (`apriltag-wasm`) and benchmarking (`apriltag-bench-wasm`)
- Detection test harness with web UI (`apriltag-bench`)
- Built-in tag family data for tag16h5, tag25h9, tag36h11
- Image preprocessing pipeline: grayscale conversion, decimation, gaussian blur
- Gradient computation with magnitude and direction
- Union-find segmentation and clustering
- Quad detection from clustered edge segments
- Homography-based decoding and tag family matching
- CI pipeline with check, clippy, test, fmt, and WASM build jobs
- Root `README.md` with project description, usage examples, and build instructions
- `LICENSE` file with BSD 2-Clause license (matching upstream AprilTag)
- `UnionFind::root_size()` method to read set size without redundant `find()` call

### Changed

- `QuickDecode::decode` returns named `QuickDecodeMatch` struct instead of opaque `(i32, i32, i32)` tuple
- `Detection.family_name` and `DecodeResult.family_name` replaced with `family_id: FamilyId` (an `Arc<str>` newtype), eliminating per-detection string allocation
- Pool threshold tile arrays, deglitch morph buffers, unsharp mask buffer, and refine_edges vals in `DetectorBuffers`, eliminating ~250 allocs/frame on reuse
- Hoist per-row `Vec<&[u8]>` allocation above blur vertical pass loop (~240 allocs → 1)
- Quad fitting: use unstable sort for angular sorting, eliminate allocation in `smooth_errors`, reuse buffers across clusters
- `gradient_clusters` sort uses index indirection (12-byte pairs instead of 20-byte), reducing data movement by 40%
- `gradient_clusters` inner loop restricted to interior pixels, uses direct buffer indexing and single `find()` per pixel
- WASM `Detector` reuses `DetectorBuffers` and grayscale conversion buffer across frames, eliminating ~850KB/frame allocation churn
- `#![forbid(unsafe_code)]` enforced across all crates (`apriltag-bench` uses `deny` for FFI module)
