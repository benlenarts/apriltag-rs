# 003 — 3×3 matrix type and basic operations

## Goal
Implement a `[f64; 9]` row-major 3×3 matrix representation with multiply, determinant, transpose, inverse, and identity.

## Preconditions
- 001 complete: `apriltag-detect` crate compiles

## Postconditions
- `mat33_inverse(mat33_identity())` returns identity (within f64 epsilon)
- Inverse of known matrix verified against hand-computed result
- Determinant of singular matrix returns 0.0
- `mat33_mul(A, mat33_inverse(A))` ≈ identity for invertible A
- `mat33_transpose(mat33_transpose(A))` == A

## Description
Create `apriltag-detect/src/math.rs` with functions operating on `[f64; 9]`:

```rust
pub fn mat33_identity() -> [f64; 9]
pub fn mat33_mul(a: &[f64; 9], b: &[f64; 9]) -> [f64; 9]
pub fn mat33_det(m: &[f64; 9]) -> f64
pub fn mat33_transpose(m: &[f64; 9]) -> [f64; 9]
pub fn mat33_inverse(m: &[f64; 9]) -> Option<[f64; 9]>  // None if singular
```

Row-major layout: `m[row * 3 + col]`. Inverse via adjugate matrix divided by determinant. All functions `#[inline]`.

## References
- `docs/detection-spec.md` §7 — homography is a 3×3 matrix; projection uses `H * [x, y, 1]`
- `docs/reference-detection/matd.h` — reference matrix operations
