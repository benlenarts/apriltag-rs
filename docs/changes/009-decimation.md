# 009 — Image decimation (block averaging)

## Goal
Downsample an image by a given factor, averaging blocks of pixels.

## Preconditions
- 007 complete: `ImageU8::new()`, `get()`, `set()` available

## Postconditions
- 100×100 image decimated by 2 → 50×50 output
- Output pixel values equal the mean of the corresponding f×f input block
- Factor 1.0 → output identical to input (copy)
- Factor 1.5 → floor dimensions, integer block size = 1 (each pixel maps to one output)
- Factor 3.0 on 10×10 → 3×3 output
- Output has correct stride alignment (inherited from ImageU8::new)

## Description
Add to `image.rs`:

```rust
pub fn decimate(src: &ImageU8, factor: f32) -> ImageU8
```

For integer factors (most common: 2, 3):
- Output dimensions: `width / f`, `height / f`
- Each output pixel = sum of f×f input block / (f*f)
- Use integer accumulation to avoid floating-point overhead

For non-integer factors:
- Round factor to nearest integer ≥ 1
- Same block-averaging logic

The caller is responsible for scaling coordinates back by the decimation factor when mapping from decimated space to original space (affects edge refinement).

## References
- `docs/detection-spec.md` §3.1 — decimation: `dfactor ← round(quad_decimate), output = floor(dim/f)`
