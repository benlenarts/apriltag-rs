# 044 — Initial pose from homography decomposition

## Goal
Extract an initial rotation and translation estimate from the detection homography for the orthogonal iteration starting point.

## Preconditions
- 003 complete: `mat33_mul`, `mat33_inverse`
- 006 complete: `svd_3x3`

## Postconditions
- Frontal tag at distance d → `R ≈ I`, `t = [0, 0, d]`
- Translation scaled by `tagsize / 2`
- Rotation is approximately orthogonal (before refinement)
- Sign conventions match camera frame: Z forward, X right, Y down

## Description
Create `apriltag-detect/src/pose.rs`:

```rust
pub fn estimate_pose(
    det: &Detection,
    tagsize: f64,
    fx: f64, fy: f64,
    cx: f64, cy: f64,
) -> Pose
```

Algorithm:
1. Build camera matrix `K = [[fx,0,cx],[0,fy,cy],[0,0,1]]`
2. Compute `K_inv * H` → columns `[r0, r1, t]`
3. Normalize: `scale = (|r0| + |r1|) / 2`; `r0 /= scale`, `r1 /= scale`, `t /= scale`
4. `r2 = r0 × r1` (cross product)
5. Assemble `R = [r0, r1, r2]` (columns)
6. Project R onto SO(3) via SVD: `R = U * V^T` (with sign correction)
7. Scale translation: `t *= tagsize / 2`

This gives a reasonable initial pose for the orthogonal iteration refinement.

## References
- `docs/detection-spec.md` §12 — "decompose H: K_inv * H → [r0, r1, t], project onto SO(3)"
