# 012 — Tile min/max dilation and erosion

## Goal
Apply 3×3 neighborhood dilation to max tiles and erosion to min tiles to prevent threshold discontinuities at tile boundaries.

## Preconditions
- 011 complete: tile min/max arrays computed

## Postconditions
- Single bright tile (max=255) surrounded by dark (max=0) → after dilation, 3×3 neighborhood has max=255
- Single dark tile (min=0) surrounded by bright (min=200) → after erosion, 3×3 neighborhood has min=0
- Boundary tiles: clamp neighbor lookups to valid range (no out-of-bounds)
- Dilation and erosion operate on separate copies (not in-place)

## Description
Add to `threshold.rs`:

```rust
pub fn dilate_tile_max(tiles: &[u8], tw: u32, th: u32) -> Vec<u8>
pub fn erode_tile_min(tiles: &[u8], tw: u32, th: u32) -> Vec<u8>
```

**Dilation** (max tiles): for each tile (tx, ty), output = max of 3×3 neighborhood `[tx-1..=tx+1, ty-1..=ty+1]`, clamping to grid bounds.

**Erosion** (min tiles): for each tile (tx, ty), output = min of 3×3 neighborhood.

This prevents sharp threshold jumps at tile boundaries in the binarization step.

## References
- `docs/detection-spec.md` §4.2 — "dilate tile maxima, erode tile minima, using 3×3 neighborhood"
