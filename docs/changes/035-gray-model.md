# 035 — Gray model construction

## Goal
Build linear intensity models for white and black border regions of a detected tag, accounting for spatially-varying illumination.

## Preconditions
- 032 complete: homography `project()` available
- 008 complete: `ImageU8::interpolate()` available

## Postconditions
- Uniform illumination → flat model: `C[0]≈0, C[1]≈0, C[2]≈pixel_value`
- Linear gradient illumination → model captures gradient slope
- Separate white and black models built from border samples
- 8 sampling lines along tag borders (4 white side, 4 black side)
- Model solved via 3×3 linear system: `A · C = B`

## Description
Add to `decode.rs`:

```rust
pub struct GrayModel {
    A: [[f64; 3]; 3],  // J^T J (symmetric, upper triangular stored)
    B: [f64; 3],       // J^T gray
    C: [f64; 3],       // solved coefficients [Cx, Cy, C0]
}

impl GrayModel {
    pub fn new() -> Self
    pub fn add(&mut self, x: f64, y: f64, gray: f64)
    pub fn solve(&mut self)
    pub fn interpolate(&self, x: f64, y: f64) -> f64
}
```

**Border sampling**: Sample 8 lines along the tag border in tag coordinates:
- 4 lines on the white side (outside the black border)
- 4 lines on the black side (inside the black border)
- For each sample: project tag coord → pixel coord via homography, read intensity via bilinear interpolation
- Add each sample to the appropriate (white or black) GrayModel

**Coordinate system**: Tag coords `[-1, 1]`. Border is at `±(width_at_border / total_width)`. Sample at border ± half pixel spacing.

**Solve**: 3×3 system `A · C = B` for coefficients `[Cx, Cy, C0]`. `interpolate(x, y) = Cx*x + Cy*y + C0`.

## References
- `docs/detection-spec.md` §10.1 — "GrayModel: A (3×3), B (3×1), solve for C = [Cx, Cy, C0]"
- `docs/detection-spec.md` §10.2 — "8 border sampling lines, 4 white + 4 black"
