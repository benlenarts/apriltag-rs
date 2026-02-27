# apriltag2

WASM-compatible, pure Rust implementation of the AprilTag detection pipeline, including tag family and bitmap generation.

## Project Tenets

1. **Deep test coverage** — correctness and performance validated through an extensive test suite
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

### Tag Families

Tag family generation (layouts, bit patterns, Hamming distance properties) is a separate module from detection.

## Development Workflow

- **TDD**: Every feature starts with a failing test. No production code without a test driving it.
- **Branches**: Work on short-lived feature/fix branches off `main`. PR and merge promptly.
- **Commits**: Small, atomic, well-described. Each commit should compile and pass tests.
- **Testing**: `cargo test` must pass before every commit. Use `cargo test -- --nocapture` for debug output.

## Commit Policy

**You MUST commit early and often — do not wait for the user to ask.** After every small, meaningful unit of progress (a passing test, a new function, a refactor, a bug fix), immediately run `cargo test` and, if tests pass, create a commit. A single user request should typically produce multiple commits, not one large one. Err on the side of committing too often rather than too rarely. Never batch up unrelated changes into a single commit.

## Commands

Verify WASM compatibility

```bash
cargo build --target wasm32-unknown-unknown 
```

## Code Style

- Idiomatic Rust with a preference for pattern matching and 'functional' iteration 
- Follow standard `rustfmt` formatting
- Use `clippy` with default lints: `cargo clippy -- -D warnings`
- Prefer `&[T]` over `&Vec<T>`, iterators over index loops where natural
- Use `thiserror` or similar for error types; avoid `.unwrap()` in library code
- Document public APIs with doc comments
- Keep modules focused — one concept per module
