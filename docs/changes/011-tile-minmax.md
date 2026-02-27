# 011 — Tile-based min/max computation

## Goal
Compute per-tile minimum and maximum pixel values for adaptive thresholding.

## Preconditions
- 007 complete: `ImageU8` available

## Postconditions
- 8×8 image with tilesz=4 → 2×2 tile grid
- Tile containing all-zero pixels → min=0, max=0
- Tile with pixels [0, 128, 255] → min=0, max=255
- Image not divisible by tilesz → partial tiles handled correctly (use actual pixels present)
- Output: two `Vec<u8>` arrays (tile_min, tile_max) of size `tw × th`

## Description
Create `apriltag-detect/src/threshold.rs`:

```rust
pub fn compute_tile_minmax(
    img: &ImageU8,
    tilesz: u32,
) -> (Vec<u8>, Vec<u8>, u32, u32)  // (tile_min, tile_max, tw, th)
```

For each tile at grid position (tx, ty):
- Pixel range: x in `[tx*tilesz .. min((tx+1)*tilesz, width))`, same for y
- Scan all pixels in tile, track min and max
- Store in `tile_min[ty * tw + tx]` and `tile_max[ty * tw + tx]`

Default `tilesz = 4`. Tile grid dimensions: `tw = (width + tilesz - 1) / tilesz`, `th = (height + tilesz - 1) / tilesz`.

## References
- `docs/detection-spec.md` §4.1 — tile min/max: "divide image into tilesz×tilesz tiles, compute min and max for each"
