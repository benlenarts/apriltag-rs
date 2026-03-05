# Commands Reference

## Testing

```bash
# Run all tests
cargo test

# Run tests with debug output
cargo test -- --nocapture
```

## Coverage

```bash
# Quick summary (excludes CLI/WASM thin wrappers)
cargo llvm-cov --ignore-filename-regex 'apriltag-gen-cli/|apriltag-detect-cli/|apriltag-wasm/|apriltag-bench-wasm/|apriltag-bench/src/main\.rs'

# Per-line detail (show uncovered lines)
cargo llvm-cov --text --ignore-filename-regex 'apriltag-gen-cli/|apriltag-detect-cli/|apriltag-wasm/|apriltag-bench-wasm/|apriltag-bench/src/main\.rs'

# HTML report
cargo llvm-cov --html --ignore-filename-regex 'apriltag-gen-cli/|apriltag-detect-cli/|apriltag-wasm/|apriltag-bench-wasm/|apriltag-bench/src/main\.rs' && open target/llvm-cov/html/index.html
```

## WASM

```bash
# Verify WASM compatibility
cargo build --target wasm32-unknown-unknown -p apriltag -p apriltag-gen

# Build WASM modules for web UI
wasm-pack build apriltag-bench-wasm --target web
wasm-pack build apriltag-wasm --target web
```

## Bench Test Harness

```bash
# Run bench test scenarios
cargo run -p apriltag-bench -- run --category baseline

# CI regression gate (exit 1 on failure)
cargo run -p apriltag-bench -- regression

# Compare against C reference (requires reference feature + fetch-references.sh)
cargo run -p apriltag-bench --features reference -- compare --format html

# Interactive single-scene exploration
cargo run -p apriltag-bench -- explore --tag-size 60 --tilt-x 30 --noise 15

# Generate test images (PGM + JSON ground truth) for all scenarios
cargo run -p apriltag-bench -- generate-images --output output/

# Generate images for a specific category
cargo run -p apriltag-bench -- generate-images --category rotation --output output/

# Launch web UI for interactive testing
cargo run -p apriltag-bench -- serve
```

## Assembly Inspection

```bash
# Install (one-time)
cargo install cargo-show-asm

# Inspect assembly for a specific function
cargo asm -p apriltag 'apriltag::detector::gradient::compute_gradient'

# Show LLVM IR instead of assembly
cargo asm -p apriltag --llvm 'apriltag::detector::gradient::compute_gradient'

# List available functions matching a pattern
cargo asm -p apriltag --search gradient
```

## Linting

```bash
cargo clippy -- -D warnings
```

## Release

See [docs/release-process.md](release-process.md) for the full release workflow.

```bash
# Tag a release
git tag -a vX.Y.Z -m "Release vX.Y.Z"

# Push with tags
git push origin main --follow-tags
```

## Reference Setup

```bash
# Download papers and clone reference repos (required for `reference` feature)
scripts/fetch-references.sh

# Verify reference feature builds
cargo check -p apriltag-bench --features reference
```
