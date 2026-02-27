# 032 — DLT homography from 4 correspondences

## Goal
Compute the 3×3 homography mapping tag coordinates [-1,1]² to pixel coordinates from 4 corner correspondences.

## Preconditions
- 005 complete: Gaussian elimination solver
- 003 complete: `mat33_inverse` for computing Hinv

## Postconditions
- Known unit square → pixel square: H projects correctly within 1e-10
- `project(H, -1, -1)` maps to quad corner 0 (bottom-left)
- `project(H, 1, -1)` maps to corner 1 (bottom-right)
- `project(H, 1, 1)` maps to corner 2 (top-right)
- `project(H, -1, 1)` maps to corner 3 (top-left)
- `Hinv * H ≈ I` (inverse consistency)
- Returns None if system is singular (degenerate quad)

## Description
Create `apriltag-detect/src/homography.rs`:

```rust
pub struct HomographyResult {
    pub H: [f64; 9],
    pub Hinv: [f64; 9],
}

pub fn compute_homography(corners: &[[f64; 2]; 4]) -> Option<HomographyResult>
pub fn project(h: &[f64; 9], x: f64, y: f64) -> (f64, f64)
pub fn unproject(h: &[f64; 9], px: f64, py: f64) -> (f64, f64)
```

**DLT setup**: For each correspondence `(tag_x, tag_y) → (px, py)`:
```
| -tag_x  -tag_y  -1   0       0       0    px*tag_x  px*tag_y  px |
|  0       0       0  -tag_x  -tag_y  -1    py*tag_x  py*tag_y  py |
```

This gives an 8×9 matrix. Solve via `gauss_eliminate_8x9`.

**Tag coordinate convention**: corners map to (-1,-1), (1,-1), (1,1), (-1,1) in CCW order.

**Projection**: `[wx, wy, w] = H * [x, y, 1]`, result = `(wx/w, wy/w)`.

## References
- `docs/detection-spec.md` §9 — "DLT: build 8×9 system from 4 correspondences, solve, H[8]=1"
