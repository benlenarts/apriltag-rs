# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

#### Core Detection Library (`apriltag`)

- Full AprilTag detection pipeline: preprocessing, gradient computation, segmentation, quad detection, homography decoding, and pose estimation
- Image preprocessing: grayscale conversion, decimation, Gaussian blur, unsharp mask
- Adaptive thresholding with tile-based min/max dilation
- Gradient computation with magnitude and direction
- Weighted union-find segmentation with path splitting
- Connected component labeling on thresholded images
- `ClusterMap` hash table for gradient cluster grouping (replaces sort-based approach)
- Quad detection with line fitting, corner detection, and validation
- Homography computation via DLT with Gaussian elimination
- Edge refinement using gradient search on original image
- `QuickDecode` struct for fast tag family matching
- Tag decoding with gray model, bit sampling, polarity handling, and sharpening
- Pose estimation with SVD, orthogonal iteration, and ambiguity resolution
- Built-in tag family data for tag16h5, tag25h9, tag36h11 (TOML + binary in `apriltag/families/`)
- `GrayImage` trait and `ImageRef` borrowed image type for zero-copy pixel access
- `ImageU8` grayscale image type with bilinear interpolation (`interpolate_unclamped`, `interpolation_safe`)
- `ImageU8::row()` for direct row-slice access
- `ImageU8::new_reuse()` and `into_buf()` for buffer reuse
- `UnionFind::root_size()` to read set size without redundant `find()` call
- `UnionFind::empty()` and `reset()` for buffer reuse across frames
- Buffer-reusing pipeline variants: `decimate_into()`, `gaussian_blur_into()`, `threshold_into()`, `connected_components_into()`
- `DetectorBuffers` (formerly `DetectorState`) for pooling allocations across frames
- Detection deduplication with lexicographic tiebreaker

#### Tag Family Generation (`apriltag-gen`)

- Era 1 (classic) code generation: `upgrade.rs` converts old row-major codes from the Java source
- Era 2 (modern) code generation: greedy lexicode search with LCG seeding
- Classic, standard, and circle layout generators
- `Layout::from_data_string()` with validation
- Tag rendering with `renderToArray` algorithm
- PDF rendering for individual tags and mosaics
- `rotate90` and Hamming distance functions
- `min_complexity` parameter in `FamilyConfig`
- Progress reporting for code generation with adaptive resolution
- Hybrid `CodeSet`: flat scan for small N, BK-tree for large N
- Complexity check optimized with bitmask + popcount (7.5× speedup)
- `verify` subcommand to regenerate and compare built-in family codes

#### CLI Tools

- `apriltag-gen-cli`: list, info, render, mosaic, and generate commands
- `apriltag-detect-cli`: detect AprilTags in PNG/JPEG images with JSON output, family selection, preprocessing controls, and optional 6-DOF pose estimation

#### WASM & Web

- `apriltag-wasm`: WASM bindings for detection with tsify types, detect, and pose bindings
- `apriltag-bench-wasm`: WASM wrapper for scene generation, used by the web UI
- Web UI for interactive scene exploration with sliders for distortion, perspective, noise, and detector config

#### Benchmarking & Testing (`apriltag-bench`)

- Scene generation module with `SceneBuilder`, backgrounds, and tag compositing
- Distortion module with 8 image distortion types
- Metrics module for detection quality evaluation (detection rate, false positives, decode accuracy)
- Catalog module with pre-defined test scenarios
- Report module and CLI with `run`, `list`, `regression`, `explore`, `generate-images` commands
- Scaling benchmark category with targeted scenarios isolating image size, noise, tag count, and decimation
- Highres 4000×3000 benchmark with ~100 tags, realistic distortions, per-tag rotation and perspective
- Criterion benchmarks for individual pipeline stages and end-to-end detection
- C reference FFI comparison behind `reference` feature flag
- `benchmark` command for Rust vs C reference timing comparison
- `benchmark-sweep` command for comprehensive multi-tag performance grid sweep
- `PersistentReferenceDetector::with_families()` for multi-family C reference benchmarks
- `alloc_comparison` example showing allocation reduction across pipeline
- `profile_loop` binary for samply profiling

#### Infrastructure

- GitHub Actions CI pipeline with check, clippy, test, fmt, and WASM build jobs
- `justfile` with development recipes (`test`, `lint`, `coverage`, `wasm-check`, `regression`, `ci`, etc.)
- 100% test coverage policy with `cargo-llvm-cov` (excluding CLI/WASM thin wrappers)
- Root `README.md` with project description, usage examples, and build instructions
- `CLAUDE.md` with project tenets, architecture, and development workflow
- Detection and generation specs in `docs/`
- `fetch-references.sh` script to download papers and clone reference repos
- `LICENSE` file with BSD 2-Clause license (matching upstream AprilTag)

### Changed

#### Development Workflow

- Justfile: renamed `bench`/`bench-ref` → `sim`/`sim-ref` (simulation harness)
- Justfile: renamed `regression` → `verify-func`, `check-coverage` → `verify-coverage`
- Justfile: removed `compare` (use `just sim-ref compare` instead)
- Justfile: added `bench`, `bench-baseline`, `bench-compare` for Criterion microbenchmarks

#### Performance

- Gaussian blur converted from f32 to fixed-point Q15 integer arithmetic
- SIMD acceleration for Gaussian blur and unsharp mask via `wide` crate (~2× speedup)
- Threshold binarization restructured into two passes for auto-vectorization (~47% faster)
- Tile arrays padded to eliminate bounds checks in threshold dilation
- Connected components: direct buffer indexing (~3.8% faster) and cached left-neighbor read (~1.4% faster)
- Gradient clustering: index indirection sort (12-byte pairs instead of 20-byte, 40% less data movement), inner loop restricted to interior pixels, direct buffer indexing, single `find()` per pixel, `connected_last` deduplication
- Quad fitting: unstable sort for angular sorting, rolling window in `smooth_errors` (eliminates allocation), buffer reuse across clusters
- `refine_edges`: precomputed interpolations (~25% speedup), `interpolate_unclamped` fast-path
- `build_line_fit_pts`: sqrt lookup table replaces per-point sqrt
- Union-find: switched from path halving to path splitting, lazy init elimination, inline annotations
- Decimation switched from averaging to subsampling
- Pool threshold tile arrays, deglitch morph buffers, unsharp mask buffer, and `refine_edges` vals in `DetectorBuffers` (~250 allocs/frame eliminated on reuse)
- Hoist per-row `Vec<&[u8]>` allocation above blur vertical pass loop (~240 allocs → 1)
- WASM `Detector` reuses `DetectorBuffers` and grayscale conversion buffer across frames (~850KB/frame allocation churn eliminated)
- 69% total allocation reduction across detection pipeline

#### API

- `QuickDecode::decode` returns named `QuickDecodeMatch` struct instead of opaque `(i32, i32, i32)` tuple
- `Detection.family_name` and `DecodeResult.family_name` replaced with `family_id: FamilyId` (an `Arc<str>` newtype), eliminating per-detection string allocation
- `DetectorState` renamed to `DetectorBuffers`
- `Detector::detect()` accepts `&(impl GrayImage + Sync)` instead of requiring owned data
- `Detector` API simplified to single `detect` method
- Re-export `GrayImage` and `ImageRef` from detect module

#### Code Quality

- `#![forbid(unsafe_code)]` enforced across all crates (`apriltag-bench` uses `deny` for FFI module)
- `clippy::unwrap_used` and `clippy::expect_used` denied in `apriltag` crate; all `.unwrap()` removed from library code
- `partial_cmp().unwrap()` replaced with `total_cmp()` in `quad.rs`
- License changed from MIT to BSD 2-Clause to match upstream AprilTag

### Fixed

- Max cluster size filter (2× → 4×) to match C reference, fixing missed detections
- Corner index off-by-one in `refine_edges`
- Missing +0.5 pixel-center delta in `build_line_fit_pts`
- Large tag detection on gray-128 background
- Singular homography matrix inverse now returns `None` instead of producing garbage
- CLI reproduction of classic tag family `.bin` files
- C reference benchmark now loads all families for multi-family scenarios (was only loading the first)
- Benchmark reuses `DetectorBuffers` across iterations for fair Rust vs C comparison
- Reference corner ordering in benchmark comparison
