# 013 — Ternary binarization

## Goal
Classify each pixel as black (0), white (255), or unknown (127) using interpolated tile min/max values.

## Preconditions
- 012 complete: dilated max tiles and eroded min tiles available

## Postconditions
- Pixel in low-contrast region (tile_max - tile_min < min_white_black_diff) → output 127 (unknown)
- Pixel above threshold in high-contrast region → output 255 (white)
- Pixel below threshold in high-contrast region → output 0 (black)
- Threshold per pixel = (interpolated_min + interpolated_max) / 2
- Output image has same dimensions as input

## Description
Add to `threshold.rs`:

```rust
pub fn threshold(
    img: &ImageU8,
    tile_min: &[u8],
    tile_max: &[u8],
    tw: u32, th: u32,
    tilesz: u32,
    min_white_black_diff: u32,
    output: &mut ImageU8,
)
```

For each pixel (x, y):
1. Compute tile position: `tx = x / tilesz`, `ty = y / tilesz` (clamped)
2. Bilinear interpolate min and max from 4 surrounding tiles
3. If `max - min < min_white_black_diff`: output = 127
4. Else: `threshold = (min + max) / 2`; output = if pixel > threshold { 255 } else { 0 }

The interpolation avoids block artifacts at tile boundaries.

## References
- `docs/detection-spec.md` §4.3 — ternary binarization: "black=0, white=255, unknown=127"
