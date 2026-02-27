# 005 — 8×9 Gaussian elimination with partial pivoting

## Goal
Solve the 8-equation DLT system for homography computation using Gaussian elimination with partial pivoting.

## Preconditions
- 003 complete: `math.rs` exists

## Postconditions
- Known 4-point correspondence (unit square → shifted square) recovers exact homography
- Mapping points through recovered H returns original correspondences within 1e-10
- Singular/near-singular system (pivot < 1e-10) returns `None`
- 8th row = h[8] = 1.0 (homogeneous normalization)

## Description
Add to `math.rs`:

```rust
pub fn gauss_eliminate_8x9(a: &mut [[f64; 9]; 8]) -> Option<[f64; 9]>
```

Algorithm:
1. For each column k (0..8):
   - Find pivot: row with largest `|a[row][k]|` in rows k..8
   - Swap pivot row to position k
   - If `|a[k][k]| < 1e-10`: return None (singular)
   - Eliminate: for each row i > k: `a[i] -= (a[i][k] / a[k][k]) * a[k]`
2. Back-substitution: `h[8] = 1.0`, solve from row 7 down
3. Return `Some(h)`

This is called once per quad during homography computation.

## References
- `docs/detection-spec.md` §7 — DLT homography: build 8×9 system, solve for h[0..8]
- `docs/reference-detection/homography.c` — `homography_compute2()` reference
