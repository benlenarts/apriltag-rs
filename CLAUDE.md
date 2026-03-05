# apriltag-rs

WASM-compatible, pure Rust implementation of the AprilTag detection pipeline, including tag family and bitmap generation.

## Project Tenets

1. **100% test coverage** — measured with `cargo-llvm-cov`; uncovered lines get tests before moving on
2. **Red-green-refactor** — strict TDD discipline; write a failing test, make it pass, then clean up
3. **Many atomic commits on short-lived branches** — small, focused changes; merge early and often; Claude does this autonomously
4. **Detection robustness** — match or exceed the reference C implementation's detection quality
5. **Performance parity** — benchmark against the reference implementation; no unnecessary allocations
6. **Modular design** — clean separation between tag families, image processing, quad detection, decoding, and pose estimation
7. **WASM compatibility** — no_std-friendly where possible; no platform-specific dependencies in core logic

## Architecture

The detection pipeline follows the reference AprilTag implementation:

1. **Image preprocessing** — grayscale conversion, decimation, gaussian blur
2. **Gradient computation** — compute gradient magnitude and direction
3. **Segmentation / clustering** — union-find on gradient edges
4. **Quad detection** — fit quadrilaterals from clustered edge segments
5. **Homography & decoding** — sample tag bits via homography, match against tag families
6. **Pose estimation** — compute camera-relative pose from known tag geometry

### Crate Structure

- **`apriltag/`** — core types, families, and rendering (42 tests). Contains `TagFamily`, `Layout`, `BitLocation`, `hamming`, `render`, and built-in family data (`.toml` + `.bin` in `apriltag/families/`).
- **`apriltag-gen/`** — generation-only code (5 tests). Re-exports `apriltag::*` plus `codegen` and `upgrade` modules.
- **`apriltag-gen-cli/`** — CLI for tag generation and rendering. Depends on `apriltag-gen`.
- **`apriltag-detect-cli/`** — CLI for detecting AprilTags in PNG/JPEG images. Wraps the core `apriltag` detector with options for tag family selection, image preprocessing (decimation, blur, sharpening), and optional 6-DOF pose estimation. Outputs JSON to stdout.
- **`apriltag-bench/`** — detection test harness and benchmark suite. Library core (scene generation, transforms, distortions, metrics) is WASM-compatible. CLI binary provides batch testing, regression checks, and C reference comparison (behind `reference` feature). Web UI in `ui/` for interactive exploration.
- **`apriltag-bench-wasm/`** — thin WASM wrapper for `apriltag-bench` scene generation, used by the web UI.
- **`apriltag-wasm/`** — WASM bindings for AprilTag detection. Used by the web UI alongside `apriltag-bench-wasm`.

### Detection Test Harness (`apriltag-bench/`)

- **Web UI** (`apriltag-bench/ui/`) — interactive WASM exploration with sliders for distortion, perspective, noise.
- **CLI** — batch test runs, regression checks, C reference comparison via FFI. Machine-readable output (JSON, HTML).

Scene generation is shared between CLI and web UI (identical Rust, WASM-compatible). C reference comparison is native-only (behind `reference` feature flag).

**Tag-space convention:** transforms map tag-space [-1, 1] to the border region (`[border_start, grid_size - border_start]` in grid coordinates), matching the detector's homography. The white border extends beyond [-1, 1]. Ground-truth corners at tag-space ±1 align with detected quad corners.

### Tag Families

**Two eras of code generation** — classic families (tag16h5, tag25h9, tag36h11) use `upgrade.rs` to convert old row-major codes from the Java source; they cannot be regenerated from scratch. Era 2 families (Standard, Circle, Custom) use `codegen.rs` with LCG seed `nbits*10000 + minhamming*100 + min_complexity`. Note: the Java source on GitHub has `+7` but the reference families were generated with per-family `+min_complexity`.

## Development Workflow

- **TDD**: Every feature starts with a failing test. No production code without a test driving it.
- **Branches**: Work on short-lived feature/fix branches off `main`. PR and merge promptly.
- **Commits**: **You MUST commit early and often — do not wait for the user to ask.** After every small, meaningful unit of progress (a passing test, a new function, a refactor, a bug fix), immediately run `cargo test` and, if tests pass, create a commit. A single user request should typically produce multiple commits, not one large one. Err on the side of committing too often rather than too rarely. Never batch up unrelated changes into a single commit.
- **Testing**: `cargo test` must pass before every commit. Use `cargo test -- --nocapture` for debug output.

## Reference Materials (`docs/`)

- `docs/detection-spec.md` — language-agnostic spec for the full detection pipeline
- `docs/generation-spec.md` — language-agnostic spec for tag family generation
- `docs/papers/` — academic papers (Olson 2011, Wang 2016, Krogius 2019, etc.)
- `docs/reference-detection/` — reference C implementation (apriltag3) for detection
- `docs/reference-generation/` — reference Java implementation for tag family generation

Run `scripts/fetch-references.sh` to download papers and clone reference repos. This is required before building with the `reference` feature flag (`cargo check -p apriltag-bench --features reference`).

## Coverage Policy

**Target: 100% test coverage.** TDD is the primary strategy — code written to pass a test is covered by definition. When a coverage gap appears, ask whether the uncovered code is reachable. If it isn't, delete it; unreachable code is a design problem, not a testing problem. If it is reachable, test it through the public API with a realistic scenario (bad input, boundary condition, corrupt data). Never write a test that exists only to hit a line — every test must assert meaningful behavior.

```bash
# Quick summary (excludes CLI crate)
cargo llvm-cov --ignore-filename-regex 'apriltag-gen-cli/'

# Per-line detail (show uncovered lines)
cargo llvm-cov --text --ignore-filename-regex 'apriltag-gen-cli/'

# HTML report
cargo llvm-cov --html --ignore-filename-regex 'apriltag-gen-cli/' && open target/llvm-cov/html/index.html
```

After completing any feature or fix, run `cargo llvm-cov --text` and inspect for uncovered lines. If coverage is below 100%, add targeted tests before moving on. Each new test is its own atomic commit.

## Benchmarking Policy

**Every change must be benchmarked before merging.** Performance parity with the reference C implementation is a project tenet — regressions are bugs.

### When to benchmark

- **Before and after every change** — any measurable regression must be investigated and resolved before committing.
- **Detection-affecting changes** — image processing, gradient computation, segmentation, quad detection, homography, or decoding must run the full suite.
- **Refactors and "safe" changes** — even seemingly neutral refactors can affect performance. Benchmark them.

Run `cargo run -p apriltag-bench -- regression` before and after your change. If any scenario regresses, fix it before committing. For large changes, generate a full HTML report (`--format html`) and review manually.

### Regression criteria

- **Detection quality** — detection rate, false positive rate, and decode accuracy must not decrease.
- **Performance (time)** — no measurable latency regression without justification and approval.
- **Performance (space)** — no unexpected memory growth. WASM targets are memory-constrained.
- **CI gate** — `cargo run -p apriltag-bench -- regression` must pass before any PR is merged.

### Assembly inspection

When optimizing hot paths, inspect generated assembly — benchmarks tell you *whether* something is fast, assembly tells you *why*. Look for: auto-vectorization vs scalar fallback, unnecessary bounds checks, redundant loads/stores, missed inlining.

**Workflow:** identify hot function via benchmarking → inspect assembly with `cargo asm` → optimize → re-inspect to verify effect → benchmark to confirm improvement.

## Commands

See `docs/commands.md` for the full reference (testing, coverage, WASM, bench harness, assembly inspection, linting, reference setup). Key commands:

```bash
cargo test                                           # run all tests
cargo llvm-cov --text --ignore-filename-regex 'apriltag-gen-cli/'  # coverage
cargo run -p apriltag-bench -- regression            # bench regression gate
cargo clippy -- -D warnings                          # lint
cargo build --target wasm32-unknown-unknown -p apriltag -p apriltag-gen  # WASM check
```

## Code Style

- Idiomatic Rust with a preference for pattern matching and 'functional' iteration
- Follow standard `rustfmt` formatting
- Use `clippy` with default lints: `cargo clippy -- -D warnings`
- Prefer `&[T]` over `&Vec<T>`, iterators over index loops where natural
- Use `thiserror` or similar for error types; avoid `.unwrap()` in library code
- Document public APIs with doc comments
- Keep modules focused — one concept per module
