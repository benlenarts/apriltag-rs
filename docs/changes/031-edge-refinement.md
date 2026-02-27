# 031 — Edge refinement on original image

## Goal
Snap quad edges to strong gradients in the original (undecimated) image for sub-pixel corner accuracy.

## Preconditions
- 007 complete: `ImageU8` (original, undecimated)
- 008 complete: bilinear interpolation
- 027 complete: quad corners from fitting stage

## Postconditions
- Corners shift toward true edges (more accurate than decimated-space corners)
- Sample count per edge = `max(16, edge_length / 8)`
- Search range along normal = `quad_decimate + 1` pixels
- If refinement fails (e.g., no strong gradient found): keep original corners
- Only runs when `refine_edges = true` (default)

## Description
Create `apriltag-detect/src/refine.rs`:

```rust
pub fn refine_edges(
    quad: &mut Quad,
    original: &ImageU8,
    quad_decimate: f32,
)
```

For each of 4 edges (corners[i] → corners[(i+1)%4]):
1. Scale corners from decimated to original coords: `p *= quad_decimate`
2. Compute edge direction and normal
3. Sample N points along edge (N = max(16, edge_length/8))
4. For each sample point: search along normal ±(decimate+1) pixels
   - At each offset: sample two pixels separated by 1 in the normal direction
   - Weight = `(g2 - g1)²` (squared gradient)
   - Accumulate weighted position into line-fit moments
5. Fit line from accumulated moments (same eigenanalysis as change 022)
6. Recompute corners by intersecting refined lines

If any refined line has insufficient samples or failed intersection, keep original corners for that edge.

## References
- `docs/detection-spec.md` §8 — "refine edges on original image: sample along edge, search along normal, weight by gradient²"
