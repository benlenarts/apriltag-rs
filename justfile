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

# Full local CI suite
ci: test lint fmt-check wasm-check verify-func

# Detection quality regression gate (exit 1 on failure)
verify-func:
    just sim regression

# Check that all uncovered lines have a COVERAGE: comment
verify-coverage:
    ./scripts/check-coverage-comments.sh "apriltag/src/"

# Coverage summary (excludes CLI crate)
coverage:
    cargo llvm-cov --ignore-filename-regex '(apriltag-gen-cli/|apriltag-detect-cli/|apriltag-wasm/|apriltag-bench-wasm/|apriltag-bench/src/(main\.rs|bin/|report\.rs))'

# Coverage with per-line detail
coverage-text:
    cargo llvm-cov --text --ignore-filename-regex '(apriltag-gen-cli/|apriltag-detect-cli/|apriltag-wasm/|apriltag-bench-wasm/|apriltag-bench/src/(main\.rs|bin/|report\.rs))'

# Coverage as HTML report (opens in browser)
coverage-html:
    cargo llvm-cov --html --ignore-filename-regex '(apriltag-gen-cli/|apriltag-detect-cli/|apriltag-wasm/|apriltag-bench-wasm/|apriltag-bench/src/(main\.rs|bin/|report\.rs))' && open target/llvm-cov/html/index.html

# Run Criterion microbenchmarks
bench *ARGS:
    cargo bench -p apriltag {{ARGS}}

# Save a named Criterion baseline (default: "baseline")
bench-baseline NAME='baseline':
    cargo bench -p apriltag -- --save-baseline {{NAME}}

# Compare against a named Criterion baseline (default: "baseline")
bench-compare NAME='baseline':
    cargo bench -p apriltag -- --baseline {{NAME}}

# Run the simulation harness CLI (forwards all arguments)
sim *ARGS:
    cargo run --release -p apriltag-bench --bin apriltag-bench -- {{ARGS}}

# Run simulation harness with reference feature enabled (forwards all arguments)
sim-ref *ARGS:
    cargo run --release -p apriltag-bench --bin apriltag-bench --features reference -- {{ARGS}}

# Verify WASM compatibility (core crates only)
wasm-check:
    cargo build --target wasm32-unknown-unknown -p apriltag -p apriltag-gen

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
    cargo run --release -p apriltag-bench --bin apriltag-bench -- serve

# Download reference papers and clone reference implementations
fetch-references:
    ./scripts/fetch-references.sh
