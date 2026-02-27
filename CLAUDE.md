# apriltag2

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

### Tag Families

**Two eras of code generation** — classic families (tag16h5, tag25h9, tag36h11) use `upgrade.rs` to convert old row-major codes from the Java source; they cannot be regenerated from scratch. Era 2 families (Standard, Circle, Custom) use `codegen.rs` with LCG seed `nbits*10000 + minhamming*100 + min_complexity`. Note: the Java source on GitHub has `+7` but the reference families were generated with per-family `+min_complexity`.

## Development Workflow

- **TDD**: Every feature starts with a failing test. No production code without a test driving it.
- **Branches**: Work on short-lived feature/fix branches off `main`. PR and merge promptly.
- **Commits**: Small, atomic, well-described. Each commit should compile and pass tests.
- **Testing**: `cargo test` must pass before every commit. Use `cargo test -- --nocapture` for debug output.

## Commit Policy

**You MUST commit early and often — do not wait for the user to ask.** After every small, meaningful unit of progress (a passing test, a new function, a refactor, a bug fix), immediately run `cargo test` and, if tests pass, create a commit. A single user request should typically produce multiple commits, not one large one. Err on the side of committing too often rather than too rarely. Never batch up unrelated changes into a single commit.

## Reference Materials (`docs/`)

- `docs/detection-spec.md` — language-agnostic spec for the full detection pipeline
- `docs/generation-spec.md` — language-agnostic spec for tag family generation
- `docs/papers/` — academic papers (Olson 2011, Wang 2016, Krogius 2019, etc.)
- `docs/reference-detection/` — reference C implementation (apriltag3) for detection
- `docs/reference-generation/` — reference Java implementation for tag family generation

## Coverage Policy

**Target: 100% test coverage.** Use `cargo-llvm-cov` to measure coverage. Whenever you find uncovered lines, write focused tests to cover them — this is part of the TDD loop.

```bash
# Quick summary (excludes CLI crate)
cargo llvm-cov --ignore-filename-regex 'apriltag-gen-cli/'

# Per-line detail (show uncovered lines)
cargo llvm-cov --text --ignore-filename-regex 'apriltag-gen-cli/'

# HTML report
cargo llvm-cov --html --ignore-filename-regex 'apriltag-gen-cli/' && open target/llvm-cov/html/index.html
```

After completing any feature or fix, run `cargo llvm-cov --text` and inspect for uncovered lines. If coverage is below 100%, add targeted tests before moving on. Each new test is its own atomic commit.

## Commands

Verify WASM compatibility

```bash
cargo build --target wasm32-unknown-unknown -p apriltag -p apriltag-gen
```

## Code Style

- Idiomatic Rust with a preference for pattern matching and 'functional' iteration 
- Follow standard `rustfmt` formatting
- Use `clippy` with default lints: `cargo clippy -- -D warnings`
- Prefer `&[T]` over `&Vec<T>`, iterators over index loops where natural
- Use `thiserror` or similar for error types; avoid `.unwrap()` in library code
- Document public APIs with doc comments
- Keep modules focused — one concept per module
