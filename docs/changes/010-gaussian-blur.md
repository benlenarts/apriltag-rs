# 010 — Separable Gaussian blur

## Goal
Implement 1D-separable Gaussian convolution with configurable sigma, supporting both blur and unsharp masking.

## Preconditions
- 007 complete: `ImageU8` with row/pixel access

## Postconditions
- Impulse response (single bright pixel in dark image) matches Gaussian profile within rounding
- Uniform image → unchanged after blur
- sigma=0.0 → no operation (identity)
- sigma > 0 → standard blur
- sigma < 0 → unsharp mask: `clamp(2*original - blurred, 0, 255)`
- Kernel size = `4 * |sigma|` rounded to nearest odd integer, minimum 3
- Separable: horizontal pass → scratch buffer → vertical pass → output

## Description
Add to `image.rs`:

```rust
pub fn gaussian_blur(img: &mut ImageU8, sigma: f32, scratch: &mut ImageU8)
```

Algorithm:
1. If sigma == 0.0: return immediately
2. Compute kernel: `k[i] = exp(-0.5 * (i/|sigma|)²)`, normalize so Σk = 1.0
3. Horizontal pass: for each row, convolve with kernel, write to scratch
4. Vertical pass: for each column of scratch, convolve with kernel, write back to img
5. If sigma < 0 (unsharp mask): `img[i] = clamp(2 * original[i] - blurred[i], 0, 255)`

The scratch buffer is a workspace argument to enable reuse across frames.

**Performance note**: The kernel is typically small (sigma=0.8 → kernel size 3). The separable approach reduces O(k²) per pixel to O(2k).

## References
- `docs/detection-spec.md` §3.2 — Gaussian filtering: kernel, separable convolution
- `docs/detection-spec.md` §3.3 — negative sigma = unsharp mask
