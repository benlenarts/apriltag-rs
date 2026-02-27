# 017 — Edge point extraction

## Goal
Extract boundary points between adjacent black/white connected components, with gradient direction and fixed-point coordinates.

## Preconditions
- 016 complete: connected components built in union-find
- 013 complete: ternary threshold image available
- 002 complete: `EdgePoint` type defined

## Postconditions
- Black square (10×10) centered in white (20×20) image → edge points along boundary
- Gradient directions point from black toward white (gx/gy magnitude 255)
- Fixed-point coordinates: `edge.x = 2*actual_x + 1`, `edge.y = 2*actual_y + 1` (midpoint between pixels)
- Components with < 25 pixels produce no edge points
- Cluster ID = `(min(rep_a, rep_b) << 32) | max(rep_a, rep_b)` — canonical ordered pair
- Each edge point has correct gradient: v0=black, v1=white → gradient points from v0 to v1

## Description
Add to `cluster.rs`:

```rust
pub fn extract_edges(
    thresholded: &ImageU8,
    uf: &UnionFind,
    min_component_size: u32,
) -> Vec<(u64, EdgePoint)>  // (cluster_id, point)
```

For each pixel (x, y) that is not 127:
1. Skip if component size < 25 (hardcoded minimum from reference)
2. Check 4 neighbor offsets: (1,0), (0,1), (-1,1), (1,1)
3. For each neighbor (nx, ny): if opposite polarity (`v0 + v1 == 255`):
   - `rep0 = uf.find(y*w + x)`, `rep1 = uf.find(ny*w + nx)`
   - Skip if either component < `min_cluster_pixels`
   - `cluster_id = (min(rep0,rep1) << 32) | max(rep0,rep1)`
   - Edge point at midpoint: `x_fp = 2*x + dx`, `y_fp = 2*y + dy` (fixed-point)
   - Gradient: `gx = (v1 - v0) * dx_sign`, `gy = (v1 - v0) * dy_sign`

## References
- `docs/detection-spec.md` §6.1 — edge point extraction: "4 neighbor offsets, opposite polarity, fixed-point midpoint"
- `docs/detection-spec.md` §1.7 — EdgePoint: "x: u16 (2× actual x), gx: i16 (gradient direction)"
