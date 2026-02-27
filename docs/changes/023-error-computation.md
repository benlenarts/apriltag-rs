# 023 — Per-point line-fit error computation

## Goal
Compute line-fit error at every point index in a sorted cluster, identifying corners as high-error regions.

## Preconditions
- 021 complete: prefix sum moments
- 022 complete: `fit_line()` returns MSE

## Postconditions
- Straight segments → low error
- Corner points → high error (local maximum)
- Neighborhood size `ksz = min(20, sz/12)`, minimum 1
- Circular indexing: point i uses neighbors `[i-ksz .. i+ksz]` wrapping around
- Error array length = number of points in cluster

## Description
Add to `quad.rs`:

```rust
pub fn compute_errors(
    lfps: &[LineFitPoint],
    sz: usize,
    errors: &mut Vec<f64>,
)
```

For each point i in 0..sz:
1. `ksz = max(1, min(20, sz / 12))`
2. Compute range: `i0 = (i + sz - ksz) % sz`, `i1 = (i + ksz) % sz`
3. Call `fit_line(lfps, i0, i1, sz)` → get MSE
4. `errors[i] = MSE`

The circular indexing means the error at point i reflects how well a line fits the 2·ksz neighbors centered on i. At corners (where the edge changes direction), the line fit is poor → high MSE.

## References
- `docs/detection-spec.md` §7.6 — "for each index i, fit line to 2·ksz neighbors, store MSE"
