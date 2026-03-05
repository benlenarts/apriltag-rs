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
- CI pipeline with check, clippy, test, fmt, coverage, and WASM build jobs
- Root `README.md` with project description, usage examples, and build instructions
- `LICENSE` file with BSD 2-Clause license (matching upstream AprilTag)

### Changed

- Coverage target now excludes CLI/WASM thin wrappers (`apriltag-gen-cli`, `apriltag-detect-cli`, `apriltag-wasm`, `apriltag-bench-wasm`, `apriltag-bench/src/main.rs`)
