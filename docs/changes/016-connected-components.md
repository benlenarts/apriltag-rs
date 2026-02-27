# 016 — Connected components on threshold image

## Goal
Build connected components from the ternary threshold image using union-find with directional connectivity rules.

## Preconditions
- 013 complete: ternary threshold image (0/127/255) available
- 015 complete: `UnionFind` with find/union/get_set_size

## Postconditions
- Uniform white rectangle → single connected component
- Uniform black rectangle → single connected component
- Adjacent white and black regions → separate components
- Unknown (127) pixels → never initialized in union-find, not part of any component
- Diagonal white connectivity: upper-left (x-1,y-1) and upper-right (x+1,y-1) connected
- Black pixels: 4-connected only (no diagonals)
- Optimization: skip diagonal if left+up+upper-left already same component

## Description
Create initial version of `apriltag-detect/src/cluster.rs`:

```rust
pub fn build_connected_components(
    thresholded: &ImageU8,
    uf: &mut UnionFind,
)
```

Scan image left-to-right, top-to-bottom. For each pixel (x, y) that is not 127:
1. Initialize pixel in union-find: `uf.init(y * width + x)`
2. **Left neighbor** (x-1, y): if same polarity, `uf.union(current, left)`
3. **Up neighbor** (x, y-1): if same polarity, `uf.union(current, up)`
4. **Upper-left** (x-1, y-1): only for white (255) pixels, and only if `find(left) != find(up) || find(left) != find(upper_left)` — skip redundant diagonal if 3 neighbors already connected
5. **Upper-right** (x+1, y-1): only for white (255) pixels, same skip optimization

The asymmetric diagonal rule matches the reference: white gets 8-connectivity (with optimization), black gets 4-connectivity.

## References
- `docs/detection-spec.md` §5.1 — connectivity: "black: 4-connected; white: 8-connected (asymmetric diagonals)"
- `docs/detection-spec.md` §5.2 — skip optimization: "if left == up == upper-left, skip diagonal"
