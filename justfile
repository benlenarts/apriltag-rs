# Install dev tools (cargo-llvm-cov, wasm-pack, cargo-asm, wasm32 target)
setup:
    cargo install cargo-llvm-cov wasm-pack cargo-asm
    rustup target add wasm32-unknown-unknown

# Run all tests
test:
    cargo test

# Run clippy lints
lint:
    cargo clippy -- -D warnings

# Check formatting
fmt-check:
    cargo fmt --all -- --check

# Coverage summary (excludes CLI crate)
coverage:
    cargo llvm-cov --ignore-filename-regex 'apriltag-gen-cli/'

# Coverage with per-line detail
coverage-text:
    cargo llvm-cov --text --ignore-filename-regex 'apriltag-gen-cli/'

# Coverage as HTML report (opens in browser)
coverage-html:
    cargo llvm-cov --html --ignore-filename-regex 'apriltag-gen-cli/' && open target/llvm-cov/html/index.html

# Verify WASM compatibility (core crates only)
wasm-check:
    cargo build --target wasm32-unknown-unknown -p apriltag -p apriltag-gen

# Full local CI suite
ci: test lint fmt-check wasm-check regression

# Run the bench CLI (forwards all arguments)
bench *ARGS:
    cargo run -p apriltag-bench --bin apriltag-bench -- {{ARGS}}

# Run bench with reference feature enabled (forwards all arguments)
bench-ref *ARGS:
    cargo run -p apriltag-bench --bin apriltag-bench --features reference -- {{ARGS}}

# CI regression gate (exit 1 on failure)
regression:
    cargo run -p apriltag-bench --bin apriltag-bench -- regression

# Compare against C reference (requires reference feature + fetch-references.sh)
compare *ARGS:
    cargo run -p apriltag-bench --bin apriltag-bench --features reference -- compare {{ARGS}}

# Download reference papers and clone reference implementations
fetch-references:
    ./scripts/fetch-references.sh

# Build WASM module for bench scene generation
wasm-bench:
    wasm-pack build apriltag-bench-wasm --target web

# Build WASM module for AprilTag detection
wasm-detect:
    wasm-pack build apriltag-wasm --target web

# Build all WASM modules
wasm: wasm-bench wasm-detect

# Launch the bench web UI (builds WASM modules first)
serve: wasm
    cargo run -p apriltag-bench --bin apriltag-bench -- serve
