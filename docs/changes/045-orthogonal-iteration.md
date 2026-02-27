# 045 — Orthogonal iteration

## Goal
Refine the initial pose estimate via alternating translation and rotation updates (Lu et al. 2000).

## Preconditions
- 044 complete: initial R, t from homography decomposition
- 006 complete: `svd_3x3` for SO(3) projection

## Postconditions
- Reprojection error decreases monotonically over iterations
- Converges within 50 iterations (default)
- Final R is exactly orthogonal: `det(R) = 1`, `R^T R = I` within 1e-10
- Works for oblique viewing angles (up to ~80° from frontal)

## Description
Add to `pose.rs`:

```rust
fn orthogonal_iteration(
    corners: &[[f64; 2]; 4],
    tag_points: &[[f64; 3]; 4],  // 3D tag corners in tag frame
    K: &[f64; 9],
    R_init: &[f64; 9],
    t_init: &[f64; 3],
    n_iters: u32,
) -> (Pose, f64)  // (pose, final_error)
```

Algorithm (per Lu et al. 2000):
1. Precompute projection operators `F[i]` for each corner point
2. Initialize R = R_init, t = t_init
3. For each iteration:
   a. **Update t**: `t = (Σ F[i])^-1 * (Σ F[i] * R * tag_points[i])`
   b. **Update R**: Form matrix `M = Σ (F[i] * w[i]) * tag_points[i]^T`, compute SVD `M = U*S*V^T`, set `R = U * V^T`
   c. Fix sign: if `det(R) < 0`, negate third column of U
   d. Compute error: `Σ |F[i] * (R * tag_points[i] + t) - q[i]|²`
4. Return final (R, t) and error

Default n_iters = 50. Error should converge to machine precision for good data.

## References
- `docs/detection-spec.md` §12.3 — "Lu et al. 2000: alternating t/R updates, 50 iterations"
