# 055 — Feature-gated rayon integration

## Goal
Parallelize the detection pipeline's hot paths when the `parallel` feature is enabled, with identical results to single-threaded mode.

## Preconditions
- 042 complete: single-threaded detector working

## Postconditions
- `cargo test -p apriltag-detect` passes (no parallel)
- `cargo test -p apriltag-detect --features parallel` passes
- Same detection results with and without `parallel` feature (deterministic)
- Speedup on multi-core systems (2-4× typical)
- WASM build (no parallel feature) still compiles

## Description
Add conditional compilation to `detector.rs` and `cluster.rs`:

```rust
#[cfg(feature = "parallel")]
use rayon::prelude::*;

// In cluster extraction:
#[cfg(feature = "parallel")]
let edges: Vec<_> = (0..height).into_par_iter()
    .flat_map(|y| extract_row_edges(y, ...))
    .collect();

#[cfg(not(feature = "parallel"))]
let edges: Vec<_> = (0..height)
    .flat_map(|y| extract_row_edges(y, ...))
    .collect();

// In quad fitting:
#[cfg(feature = "parallel")]
let quads: Vec<_> = clusters.par_iter()
    .filter_map(|cluster| fit_quad(cluster, ...))
    .collect();

// In decoding:
#[cfg(feature = "parallel")]
let detections: Vec<_> = quads.par_iter()
    .filter_map(|quad| decode_quad(quad, ...))
    .collect();
```

**Key parallelism points:**
1. **Edge extraction**: rows are independent → `par_iter()` over row indices
2. **Quad fitting**: clusters are independent → `par_iter()` over clusters
3. **Decoding**: quads × families are independent → `par_iter()` over quads

**Determinism**: rayon's `par_iter()` preserves order with `collect()`. Results are identical.

## References
- Architecture decision — rayon behind `parallel` feature flag
