# 037 — Bit value sampling via homography

## Goal
Sample each data bit's value by projecting bit coordinates through the homography and reading the image intensity.

## Preconditions
- 032 complete: `homography::project()` available
- 008 complete: `ImageU8::interpolate()` available
- 035 complete: gray models for threshold computation
- `BitLocation { x, y }` from `apriltag/src/bits.rs`

## Postconditions
- Rendered tag → correct bit values extracted (all within threshold margin)
- Threshold at each bit position = `(white_model + black_model) / 2` at that position
- Output: grid of `f64` values where positive = white, negative = black
- Grid dimensions match tag family data area

## Description
Add to `decode.rs`:

```rust
pub fn sample_bits(
    image: &ImageU8,
    H: &[f64; 9],
    white_model: &GrayModel,
    black_model: &GrayModel,
    bit_locations: &[BitLocation],
    total_width: u32,
    values: &mut Vec<f64>,  // output: one value per bit
)
```

For each bit i:
1. Convert `bit_locations[i]` to tag coordinates:
   - `tag_x = (2.0 * bit_x + 1.0) / total_width - 1.0` (center of data cell)
   - `tag_y = (2.0 * bit_y + 1.0) / total_width - 1.0`
2. Project to pixel: `(px, py) = project(H, tag_x, tag_y)`
3. Sample intensity: `gray = image.interpolate(px, py)`
4. Compute threshold: `thresh = (white_model.interpolate(tag_x, tag_y) + black_model.interpolate(tag_x, tag_y)) / 2.0`
5. `values[i] = gray - thresh` (positive = white, negative = black)

## References
- `docs/detection-spec.md` §10.4 — "project bit_location to tag coords, then to pixel coords, sample, threshold"
- `apriltag/src/bits.rs` — `BitLocation { x: i32, y: i32 }` coordinates
