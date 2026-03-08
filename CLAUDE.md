# apriltag-rs

WASM-compatible, pure Rust implementation of the AprilTag detection pipeline, including tag family and bitmap generation.

## Tenets

1. **100% test coverage** — `cargo-llvm-cov`; uncovered lines get tests before moving on
2. **Red-green-refactor** — strict TDD; failing test → make it pass → clean up
3. **Atomic commits** — small, focused changes; Claude commits autonomously after each passing test
4. **Detection robustness** — match or exceed the reference C implementation
5. **Performance parity** — benchmark against reference; no unnecessary allocations
6. **Modular design** — clean separation between pipeline stages
7. **WASM compatibility** — no_std-friendly; no platform-specific deps in core

## Architecture

**Pipeline:** grayscale/decimation/blur → gradients → union-find clustering → quad detection → homography & decoding → pose estimation

### Crates

- **`apriltag/`** — core types, families (`.toml`+`.bin` in `families/`), rendering, detection
- **`apriltag-gen/`** — code generation (`codegen.rs`, `upgrade.rs`). Re-exports `apriltag::*`
- **`apriltag-gen-cli/`** — CLI for tag generation
- **`apriltag-detect-cli/`** — CLI for detection (JSON output, optional pose estimation)
- **`apriltag-bench/`** — test harness + benchmarks. WASM-compatible library; CLI for batch tests + C reference comparison (`reference` feature). Web UI in `ui/`
- **`apriltag-bench-wasm/`**, **`apriltag-wasm/`** — WASM bindings for bench and detection

**Tag-space convention:** transforms map tag-space [-1, 1] to the border region (`[border_start, grid_size - border_start]`). White border extends beyond [-1, 1]. Ground-truth corners at ±1 align with detected quad corners.

**Tag families:** Classic (tag16h5, tag25h9, tag36h11) use `upgrade.rs` from Java source — cannot be regenerated. Era 2 (Standard, Circle, Custom) use `codegen.rs` with LCG seed `nbits*10000 + minhamming*100 + min_complexity`.

## Workflow

- **TDD**: Every feature starts with a failing test. No production code without a test.
- **Commits**: **You MUST commit early and often — do not wait for the user to ask.** After each small unit of progress, run `cargo test` and commit if passing. Multiple commits per request. Never batch unrelated changes.
- **Changelog**: Update `CHANGELOG.md` `[Unreleased]` just before creating/updating a PR (Added/Changed/Fixed/etc.).

## Reference Materials

- `docs/detection-spec.md`, `docs/generation-spec.md` — pipeline and generation specs
- `docs/papers/` — academic papers; `docs/reference-detection/` — C ref; `docs/reference-generation/` — Java ref
- Run `just fetch-references` to download (required for `--features reference`)

## Coverage

**Target: 100%.** Unreachable code → delete it. Reachable gaps → test through public API with realistic scenarios. Every test asserts meaningful behavior.

```bash
just coverage        # summary
just coverage-text   # per-line (show uncovered lines)
just coverage-html   # HTML report
```

Run `just coverage-text` after every feature/fix. Each new test is its own commit.

## Benchmarking

**Benchmark before and after every change.** Run `just verify-func` — any regression must be fixed before committing. Regressions in detection quality, latency, or memory are bugs.

For hot-path optimization: benchmark → `cargo asm` → optimize → re-inspect → re-benchmark.

## Commands

```bash
just test            # run all tests
just coverage-text   # per-line coverage
just lint            # clippy lints
just wasm-check      # WASM compatibility
just verify-func     # detection quality gate
just bench           # Criterion microbenchmarks
just ci              # full CI (test + lint + fmt + wasm + verify-func)
```

## Code Style

- **No `unsafe` code** — completely disallowed. No `unsafe` blocks, fn, impl, or trait.
- Idiomatic Rust: pattern matching, functional iteration, `&[T]` over `&Vec<T>`
- `rustfmt` + `clippy` (`just lint`)
- `thiserror` for errors; no `.unwrap()` in library code
- Document public APIs; one concept per module
