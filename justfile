# Install dev tools (cargo-llvm-cov, wasm-pack, cargo-asm, wasm32 target)
setup:
    cargo install cargo-llvm-cov wasm-pack cargo-asm
    rustup target add wasm32-unknown-unknown

# Common cargo args: all crates, all CI-safe features (excludes apriltag-bench/reference which needs a C checkout)
_ws := "--workspace --features parallel"

# Run all tests
test:
    cargo test {{ _ws }}

# Run clippy lints
lint:
    cargo clippy {{ _ws }} -- -D warnings

# Check formatting
fmt-check:
    cargo fmt --all -- --check

# Cargo check (fast compilation check)
check:
    cargo check {{ _ws }}

# Full local CI suite
ci: check test lint fmt-check wasm-check verify-func verify-coverage

# Detection quality regression gate (exit 1 on failure)
verify-func:
    just sim regression

# Check that all uncovered lines have a COVERAGE: comment
verify-coverage:
    ./scripts/check-coverage-comments.sh "apriltag/src/"

# Crates excluded from coverage (CLIs, WASM bindings, bench harness entry points)
_cov-exclude := "--ignore-filename-regex '(apriltag-gen-cli/|apriltag-detect-cli/|apriltag-wasm/|apriltag-bench-wasm/|apriltag-bench/src/(main\\.rs|report\\.rs))'"

# Coverage summary
coverage:
    cargo llvm-cov {{ _cov-exclude }}

# Coverage with per-line detail
coverage-text:
    cargo llvm-cov --text {{ _cov-exclude }}

# Coverage as HTML report (opens in browser)
coverage-html:
    cargo llvm-cov --html {{ _cov-exclude }} && open target/llvm-cov/html/index.html

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
    cargo run --release -p apriltag-bench -- {{ARGS}}

# Run simulation harness with reference feature enabled (forwards all arguments)
sim-ref *ARGS:
    cargo run --release -p apriltag-bench --features reference -- {{ARGS}}

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
    cargo run --release -p apriltag-bench -- serve

# Compare Rust vs C reference across thread counts (default: 1,2,4,8)
bench-par THREADS='1,2,4,8':
    #!/usr/bin/env bash
    set -euo pipefail
    for t in $(echo "{{THREADS}}" | tr ',' ' '); do
        echo "=== $t thread(s) ==="
        just sim-ref benchmark --threads "$t"
        echo
    done

# Download reference papers and clone reference implementations
fetch-references:
    ./scripts/fetch-references.sh
