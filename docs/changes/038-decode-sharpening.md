# 038 — Decode sharpening (Laplacian)

## Goal
Apply Laplacian sharpening to the sampled bit values to improve decision margin at bit boundaries.

## Preconditions
- 037 complete: bit values grid available

## Postconditions
- Sharpening factor 0.25 (default) increases contrast between adjacent bits
- Sharpening factor 0.0 → no change
- Laplacian kernel: `[0,-1,0; -1,4,-1; 0,-1,0]`
- Boundary pixels: exclude neighbors outside the data grid

## Description
Add to `decode.rs`:

```rust
pub fn apply_sharpening(
    values: &mut [f64],
    data_width: u32,    // sqrt of nbits for square layouts, or grid width
    data_height: u32,
    sharpening: f64,
)
```

For each bit at grid position (bx, by):
1. Compute Laplacian: `L = 4*v[y][x] - v[y-1][x] - v[y+1][x] - v[y][x-1] - v[y][x+1]`
2. If neighbor is outside grid: skip that term (don't include in sum, adjust coefficient)
3. `values[y * w + x] += sharpening * L`

Default sharpening = 0.25. This amplifies the difference between a bit and its neighbors, making the threshold decision more robust.

**Note**: For non-square layouts (circle families), the grid positions are derived from `BitLocation` coordinates. The Laplacian uses the actual grid neighborhood, not the physical positions.

## References
- `docs/detection-spec.md` §10.4 — "Laplacian sharpening: kernel [0,-1,0;-1,4,-1;0,-1,0], factor = decode_sharpening"
