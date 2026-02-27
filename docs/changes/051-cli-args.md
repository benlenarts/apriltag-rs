# 051 — Create `apriltag-detect-cli` crate with clap args

## Goal
Set up the detection CLI with comprehensive argument parsing for all detector configuration options.

## Preconditions
- 042 complete: `Detector` in `apriltag-detect`

## Postconditions
- `cargo run -p apriltag-detect-cli -- --help` shows all options
- Options: `--family`, `--decimate`, `--blur`, `--sharpening`, `--no-refine`, `--max-hamming`, `--pretty`, `--quiet`
- Pose options: `--pose`, `--tag-size`, `--fx`, `--fy`, `--cx`, `--cy`
- Positional: `<IMAGE>...` (one or more files)
- Default family: `tag36h11`

## Description
Create `apriltag-detect-cli/Cargo.toml`:
```toml
[package]
name = "apriltag-detect-cli"
version = "0.1.0"
edition = "2021"
description = "CLI for AprilTag detection"

[[bin]]
name = "apriltag-detect"

[dependencies]
apriltag = { path = "../apriltag" }
apriltag-detect = { path = "../apriltag-detect", features = ["parallel"] }
clap = { version = "4.5", features = ["derive"] }
image = { version = "0.25", default-features = false, features = ["png", "jpeg"] }
serde = { version = "1", features = ["derive"] }
serde_json = "1"
anyhow = "1"
```

Create `apriltag-detect-cli/src/main.rs`:
```rust
use clap::Parser;

#[derive(Parser)]
#[command(name = "apriltag-detect", about = "Detect AprilTags in images")]
struct Args {
    #[arg(required = true)]
    images: Vec<PathBuf>,

    #[arg(short, long, default_value = "tag36h11")]
    family: Vec<String>,

    #[arg(short, long, default_value_t = 2.0)]
    decimate: f32,

    #[arg(short, long, default_value_t = 0.0)]
    blur: f32,

    #[arg(short, long, default_value_t = 0.25)]
    sharpening: f64,

    #[arg(long)]
    no_refine: bool,

    #[arg(long, default_value_t = 2)]
    max_hamming: u32,

    #[arg(long)]
    pretty: bool,

    #[arg(long)]
    quiet: bool,

    // Pose estimation options
    #[arg(long)]
    pose: bool,
    #[arg(long)]
    tag_size: Option<f64>,
    #[arg(long)]
    fx: Option<f64>,
    #[arg(long)]
    fy: Option<f64>,
    #[arg(long)]
    cx: Option<f64>,
    #[arg(long)]
    cy: Option<f64>,
}
```

Add `"apriltag-detect-cli"` to workspace members.

## References
- Architecture decision — CLI design
- `apriltag-gen-cli/Cargo.toml` — pattern for CLI crate
