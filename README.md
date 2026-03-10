# apriltag-rs

Pure Rust implementation of the [AprilTag](https://april.eecs.umich.edu/software/apriltag) visual fiducial system. WASM-compatible.

<!-- Badges auto-updated by CI on push to main (see .github/workflows/stats.yml) -->
[![CI](https://github.com/benlenarts/apriltag-rs/actions/workflows/ci.yml/badge.svg)](https://github.com/benlenarts/apriltag-rs/actions/workflows/ci.yml)
[![tests](https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/benlenarts/apriltag-rs/main/.github/badges/tests.json)](https://github.com/benlenarts/apriltag-rs/actions/workflows/ci.yml)
[![coverage](https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/benlenarts/apriltag-rs/main/.github/badges/coverage.json)](https://github.com/benlenarts/apriltag-rs/actions/workflows/stats.yml)
[![regression](https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/benlenarts/apriltag-rs/main/.github/badges/regression.json)](https://github.com/benlenarts/apriltag-rs/actions/workflows/stats.yml)
[![unsafe](https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/benlenarts/apriltag-rs/main/.github/badges/unsafe.json)](#safety)
[![license](https://img.shields.io/badge/license-BSD--2--Clause-blue)](LICENSE)

## At a glance

| Metric | Value |
|--------|-------|
| **Performance** | 0.95× overall vs C reference (single-threaded, 59 scenarios) |
| **Tests** | 377 unit + integration tests |
| **Coverage** | 99.5% line coverage (cargo-llvm-cov) |
| **Regression suite** | 59 scenarios, all passing |
| **Safety** | `#![forbid(unsafe_code)]` in all production crates |
| **Code** | ~18k lines of pure Rust |

> All numbers above are auto-updated by CI on every push to main.
> See [`.github/badges/`](.github/badges/) for the raw data.

## Features

- **Pure Rust** — `forbid(unsafe_code)`, minimal dependencies, 99.5% test coverage
- **WASM-compatible** — runs in the browser via WebAssembly
- **All standard tag families** — Tag16h5, Tag25h9, Tag36h11, Standard41h12, Standard52h13, Circle21h7, Circle49h12, Custom48h12
- **Tag generation** — generate and render tag family bitmaps
- **Optional parallelism** — multi-threaded detection via Rayon

## Performance

Detection performance is benchmarked against the [reference C implementation](https://github.com/AprilRobotics/apriltag) (apriltag3) across 59 scenarios. Rust is **faster on clean scenes** and **slightly slower on noisy scenes** where gradient clustering generates more edges to process.

**Per-condition averages** (single-threaded, median of adaptive iterations):

| Condition | Rust | C reference | Ratio |
|-----------|------|-------------|-------|
| Clean scenes | 6.5 ms | 7.7 ms | **0.85×** |
| Rotation (30°) | 7.4 ms | 8.5 ms | **0.86×** |
| Blur (sigma 2) | 6.8 ms | 8.0 ms | **0.85×** |
| Contrast (25%) | 6.6 ms | 7.6 ms | **0.87×** |
| Perspective (20° tilt) | 9.9 ms | 10.7 ms | **0.93×** |
| Noise (sigma 20) | 126.4 ms | 114.7 ms | 1.10× |
| Combined distortions | 71.6 ms | 62.1 ms | 1.15× |

**Multi-threaded scaling** (59-scenario aggregate):

| Threads | Rust | C reference | Ratio |
|---------|------|-------------|-------|
| 1 | 798 ms | 844 ms | **0.95×** |
| 2 | 521 ms | 519 ms | 1.00× |
| 4 | 402 ms | 351 ms | 1.15× |

**Highlights:**
- **Single-threaded aggregate: 0.95×** (5% faster than C across all 59 scenarios)
- **4000×3000 with decimation:** 0.44× (2.3× faster than C)
- **2000×1500 clean:** 0.70× (30% faster than C)
- **Multi-threaded gap** is dominated by the `highres-4000x3000` noisy outlier (1.57× at 4T); clean scenes stay ~0.50× at all thread counts

The 59-scenario regression suite covers rotation, perspective, scale (16–200px tags), noise (sigma 5–40), contrast (10–50%), lighting gradients, blur, multi-tag, occlusion, decimation modes, and scaling benchmarks. Every scenario must pass on every commit.


### Streaming efficiency

The detection API is designed for real-time use. `DetectorBuffers` pools all internal allocations across frames:

- **69% allocation reduction** vs naive per-frame allocation
- Threshold tile arrays, deglitch morph buffers, unsharp mask buffer, and edge refinement scratch space are all reused
- WASM detector reuses buffers and grayscale conversion buffer across frames (~850 KB/frame allocation churn eliminated)
- `ImageU8::new_reuse()` and `into_buf()` enable zero-copy buffer recycling

Run the benchmarks yourself:

```bash
# Criterion micro-benchmarks (per-stage and end-to-end)
cargo bench -p apriltag

# Rust vs C reference comparison (requires scripts/fetch-references.sh first)
just bench-ref benchmark
```

## Safety

All production crates enforce `#![forbid(unsafe_code)]`:

| Crate | Policy |
|-------|--------|
| `apriltag` | `forbid(unsafe_code)` |
| `apriltag-gen` | `forbid(unsafe_code)` |
| `apriltag-gen-cli` | `forbid(unsafe_code)` |
| `apriltag-detect-cli` | `forbid(unsafe_code)` |
| `apriltag-wasm` | `forbid(unsafe_code)` |
| `apriltag-bench-wasm` | `forbid(unsafe_code)` |
| `apriltag-bench` | `deny(unsafe_code)` (FFI bridge to C reference only) |

The only `unsafe` code in the project is the optional C reference FFI bridge (`apriltag-bench/src/reference.rs`), used exclusively for benchmark comparison. It is never compiled unless the `reference` feature is explicitly enabled.

## Crates

| Crate | Description |
|-------|-------------|
| `apriltag` | Core detection library — types, families, rendering, and the full detection pipeline |
| `apriltag-gen` | Tag family generation (codegen and legacy upgrade) |
| `apriltag-gen-cli` | CLI for generating and rendering tag families |
| `apriltag-detect-cli` | CLI for detecting tags in images |
| `apriltag-wasm` | WASM bindings for detection |
| `apriltag-bench` | Detection test harness, benchmarks, and regression suite |
| `apriltag-bench-wasm` | WASM bindings for the benchmark scene generator |

## Quick start

```toml
[dependencies]
apriltag = { git = "https://github.com/benlenarts/apriltag-rs.git" }
```

### Detect tags in an image

```rust
use apriltag::detect::detector::{Detector, DetectorBuffers, DetectorConfig};
use apriltag::detect::image::ImageU8;
use apriltag::family;

// Load a grayscale image (width, height, stride, pixels)
let img = ImageU8::from_buf(width, height, width, pixels);

// Create a detector with default settings
let mut detector = Detector::new(DetectorConfig::default());
detector.add_family(family::tag36h11(), 2);

let detections = detector.detect(&img, &mut DetectorBuffers::new());
for det in &detections {
    println!("id={} center={:?}", det.id, det.center);
}
```

### Detect tags from the CLI

```bash
cargo run -p apriltag-detect-cli -- input.png
```

### Build for WASM

```bash
wasm-pack build apriltag-wasm --target web
```

## Tag families

| Family | Bits | Min Hamming | Tags |
|--------|------|-------------|------|
| Tag16h5 | 16 | 5 | 30 |
| Tag25h9 | 25 | 9 | 35 |
| Tag36h11 | 36 | 11 | 587 |
| Standard41h12 | 41 | 12 | 2,115 |
| Standard52h13 | 52 | 13 | 48,714 |
| Circle21h7 | 21 | 7 | 38 |
| Circle49h12 | 49 | 12 | 65,535 |
| Custom48h12 | 48 | 12 | 42,211 |

Enable only the families you need to reduce binary size:

```toml
[dependencies]
apriltag = { version = "0.1", default-features = false, features = ["family-tag36h11"] }
```

## Architecture

```
Image → Preprocessing → Gradients → Segmentation → Quads → Homography → Decode → Pose
         (decimate,      (magnitude,  (union-find,   (line    (DLT +      (gray    (SVD +
          blur)           direction)   clustering)    fitting, Gauss       model,   orthogonal
                                                     corners) elimination) bits)   iteration)
```

Each stage is independently benchmarked and tested. With the `parallel` feature, all major stages run on Rayon's thread pool.

## References

- Olson, E. "AprilTag: A robust and flexible visual fiducial system." ICRA 2011.
- Wang, J. and Olson, E. "AprilTag 2: Efficient and robust fiducial detection." IROS 2016.
- Krogius, M., Haggenmiller, A., and Olson, E. "Flexible Layouts for Fiducial Tags." IROS 2019.

## License

BSD 2-Clause — see [LICENSE](LICENSE) for details.
