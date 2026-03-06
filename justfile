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
