# apriltag-rs

Pure Rust implementation of the [AprilTag](https://april.eecs.umich.edu/software/apriltag) visual fiducial system. No C dependencies. WASM-compatible.

## Features

- **Pure Rust** — no FFI, no C toolchain required
- **WASM-compatible** — runs in the browser via WebAssembly
- **All standard tag families** — Tag16h5, Tag25h9, Tag36h11, Standard41h12, Standard52h13, Circle21h7, Circle49h12, Custom48h12
- **Full detection pipeline** — grayscale conversion, decimation, blur, gradient computation, quad detection, homography, decoding, and pose estimation
- **Tag generation** — generate and render tag family bitmaps
- **Optional parallelism** — multi-threaded detection via Rayon

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
use apriltag::detect::detector::{Detector, DetectorConfig};
use apriltag::detect::image::ImageU8;
use apriltag::family;

// Load a grayscale image (width, height, stride, pixels)
let img = ImageU8::from_buf(width, height, width, pixels);

// Create a detector with default settings
let mut detector = Detector::new(DetectorConfig::default());
detector.add_family(family::tag36h11(), 2);

let detections = detector.detect(&img);
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

## References

- Olson, E. "AprilTag: A robust and flexible visual fiducial system." ICRA 2011.
- Wang, J. and Olson, E. "AprilTag 2: Efficient and robust fiducial detection." IROS 2016.
- Krogius, M., Haggenmiller, A., and Olson, E. "Flexible Layouts for Fiducial Tags." IROS 2019.

## License

MIT
