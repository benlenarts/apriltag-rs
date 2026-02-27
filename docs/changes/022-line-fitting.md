# 022 — Line fitting via eigenanalysis

## Goal
Fit a line to a range of points using covariance eigenanalysis, returning the line parameters and MSE.

## Preconditions
- 021 complete: prefix sum moments available
- 004 complete: `eigenvalues_sym2x2` available

## Postconditions
- Points along y=2x → line direction matches (within angular tolerance)
- MSE (smallest eigenvalue / W) near 0 for collinear points
- Returns line as `[centroid_x, centroid_y, normal_x, normal_y]`
- Normal direction: perpendicular to the line, consistent orientation
- Handles degenerate cases (all points coincident) → MSE = 0

## Description
Add to `quad.rs`:

```rust
pub fn fit_line(lfps: &[LineFitPoint], i0: usize, i1: usize, sz: usize) -> LineFit
```

Where `LineFit = { line: [f64; 4], mse: f64 }`.

Algorithm (for range [i0, i1] with wraparound):
1. Extract moments by prefix-sum subtraction: `M = lfps[i1] - lfps[i0]` (handling wrap with `+= lfps[sz]`)
2. Compute covariance: `Cxx = Mxx/W - (Mx/W)², Cxy = Mxy/W - (Mx*My)/W², Cyy = Myy/W - (My/W)²`
3. Find eigenvalues via `eigenvalues_sym2x2(Cxx, Cxy, Cyy)` → `(λ_max, λ_min)`
4. Line direction = eigenvector of λ_max
5. Normal = eigenvector of λ_min (perpendicular to line)
6. MSE = λ_min / W
7. Centroid = (Mx/W, My/W)

Return `[centroid_x, centroid_y, normal_x, normal_y]` and MSE.

## References
- `docs/detection-spec.md` §7.5 — "covariance from moments, eigenanalysis, MSE = λ_min"
