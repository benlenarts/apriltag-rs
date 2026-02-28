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
    cargo run -p apriltag-bench -- serve
