# 008 — Bilinear interpolation

## Goal
Add sub-pixel sampling to ImageU8 for edge refinement and bit sampling stages.

## Preconditions
- 007 complete: `ImageU8` with `get(x, y)` available

## Postconditions
- Integer coordinates return the exact pixel value (as f64)
- Midpoint (0.5, 0.5) between 4 pixels returns their average
- Coordinates at pixel edges blend correctly (linear weights)
- Near-boundary coordinates are clamped to valid range
- `#[inline]` attribute for hot-path performance

## Description
Add to `image.rs`:

```rust
impl ImageU8 {
    #[inline]
    pub fn interpolate(&self, fx: f64, fy: f64) -> f64 {
        let ix = fx.floor() as i32;
        let iy = fy.floor() as i32;
        let dx = fx - ix as f64;
        let dy = fy - iy as f64;
        // clamp to valid pixel range
        let ix = ix.clamp(0, self.width as i32 - 2) as usize;
        let iy = iy.clamp(0, self.height as i32 - 2) as usize;
        let v00 = self.buf[iy * self.stride as usize + ix] as f64;
        let v01 = self.buf[iy * self.stride as usize + ix + 1] as f64;
        let v10 = self.buf[(iy + 1) * self.stride as usize + ix] as f64;
        let v11 = self.buf[(iy + 1) * self.stride as usize + ix + 1] as f64;
        v00 * (1.0 - dx) * (1.0 - dy) + v01 * dx * (1.0 - dy)
            + v10 * (1.0 - dx) * dy + v11 * dx * dy
    }
}
```

4 pixel reads, 3 multiplies, 3 adds. This is called in tight loops during edge refinement (change 031) and bit sampling (change 037).

## References
- `docs/detection-spec.md` §6 — edge refinement uses bilinear interpolation to sample gradient
- `docs/detection-spec.md` §8 — bit sampling uses bilinear interpolation on original image
