use super::image::ImageU8;
use wide::{i32x8, u32x8};

/// Decimate an image by factor `f`, subsampling every f-th pixel.
///
/// Picks the top-left pixel of each f×f block, matching the C reference
/// implementation (`image_u8_decimate`).
///
/// Pass a pre-allocated `buf` to reuse memory across calls. Use `Vec::new()`
/// for one-shot usage.
pub fn decimate(img: &ImageU8, f: u32, buf: Vec<u8>) -> ImageU8 {
    if f <= 1 {
        return img.clone();
    }

    let out_w = img.width / f;
    let out_h = img.height / f;
    let mut out = ImageU8::new_reuse(out_w, out_h, buf);

    for oy in 0..out_h {
        for ox in 0..out_w {
            out.set(ox, oy, img.get(ox * f, oy * f));
        }
    }
    out
}

/// Build a 1D Gaussian kernel with the given sigma and kernel size.
///
/// Returns fixed-point kernel values scaled so they sum to `1 << 15` (32768).
/// `ksz` must be odd.
///
/// Overflow safety: the blur accumulates `255 * 32768 * ksz` in a `u32`.
/// This fits for `ksz <= 514` (sigma <= 128). Practical sigma values (0-2)
/// yield `ksz` in the range 3-9.
fn gaussian_kernel(sigma: f32, ksz: usize) -> Vec<u16> {
    let half = ksz as i32 / 2;
    let mut raw = Vec::with_capacity(ksz);
    let mut sum = 0.0f32;
    for i in 0..ksz as i32 {
        let x = (i - half) as f32;
        let v = (-x * x / (2.0 * sigma * sigma)).exp();
        raw.push(v);
        sum += v;
    }
    raw.iter()
        .map(|&v| ((v / sum) * 32768.0 + 0.5) as u16)
        .collect()
}

/// Apply separable Gaussian blur with the given sigma and kernel size.
///
/// Uses fixed-point integer arithmetic (Q15) to avoid all float ops in the
/// inner loops. Accumulates in `u32` and rounds via `(sum + (1 << 14)) >> 15`.
/// Apply separable Gaussian blur with the given sigma and kernel size.
///
/// Uses fixed-point integer arithmetic (Q15) to avoid all float ops in the
/// inner loops. Accumulates in `u32` and rounds via `(sum + (1 << 14)) >> 15`.
///
/// Returns `(output_image, reclaimed_tmp_buf)`.
fn gaussian_blur(
    img: &ImageU8,
    sigma: f32,
    ksz: usize,
    tmp_buf: Vec<u8>,
    out_buf: Vec<u8>,
) -> (ImageU8, Vec<u8>) {
    let kernel = gaussian_kernel(sigma, ksz);
    let half = ksz as i32 / 2;
    let w = img.width as i32;
    let h = img.height as i32;
    let wu = img.width as usize;

    // Horizontal pass: SIMD for interior, scalar for edges
    let mut tmp = ImageU8::new_reuse(img.width, img.height, tmp_buf);
    let halfu = half as usize;
    for y in 0..h {
        let row = img.row(y as u32);
        let out_off = y as usize * wu;

        // Left edge (scalar with clamping)
        for x in 0..halfu.min(wu) {
            let mut acc = 0u32;
            for k in 0..ksz as i32 {
                let sx = (x as i32 + k - half).clamp(0, w - 1) as usize;
                acc += row[sx] as u32 * kernel[k as usize] as u32;
            }
            tmp.buf[out_off + x] = ((acc + (1 << 14)) >> 15) as u8;
        }

        // Interior (SIMD — no clamping needed)
        let interior_end = (wu - halfu).max(halfu);
        let mut x = halfu;
        while x + 8 <= interior_end {
            let mut acc = u32x8::ZERO;
            for (ki, &kv) in kernel.iter().enumerate() {
                let src_x = x + ki - halfu;
                let pixels = u32x8::new([
                    row[src_x] as u32,
                    row[src_x + 1] as u32,
                    row[src_x + 2] as u32,
                    row[src_x + 3] as u32,
                    row[src_x + 4] as u32,
                    row[src_x + 5] as u32,
                    row[src_x + 6] as u32,
                    row[src_x + 7] as u32,
                ]);
                acc += pixels * u32x8::splat(kv as u32);
            }
            let rounded: u32x8 = (acc + u32x8::splat(1 << 14)) >> 15;
            let vals = rounded.to_array();
            for i in 0..8 {
                tmp.buf[out_off + x + i] = vals[i] as u8;
            }
            x += 8;
        }
        // Interior remainder (scalar, no clamping)
        while x < interior_end {
            let mut acc = 0u32;
            for (ki, &kv) in kernel.iter().enumerate() {
                acc += row[x + ki - halfu] as u32 * kv as u32;
            }
            tmp.buf[out_off + x] = ((acc + (1 << 14)) >> 15) as u8;
            x += 1;
        }

        // Right edge (scalar with clamping)
        for x in interior_end..wu {
            let mut acc = 0u32;
            for k in 0..ksz as i32 {
                let sx = (x as i32 + k - half).clamp(0, w - 1) as usize;
                acc += row[sx] as u32 * kernel[k as usize] as u32;
            }
            tmp.buf[out_off + x] = ((acc + (1 << 14)) >> 15) as u8;
        }
    }

    // Vertical pass: SIMD for 8-wide chunks, scalar remainder
    let mut out = ImageU8::new_reuse(img.width, img.height, out_buf);
    for y in 0..h {
        let rows: Vec<&[u8]> = (0..ksz as i32)
            .map(|k| tmp.row((y + k - half).clamp(0, h - 1) as u32))
            .collect();
        let out_off = y as usize * wu;

        let mut x = 0usize;
        while x + 8 <= wu {
            let mut acc = u32x8::ZERO;
            for (k, &kv) in kernel.iter().enumerate() {
                let r = rows[k];
                let pixels = u32x8::new([
                    r[x] as u32,
                    r[x + 1] as u32,
                    r[x + 2] as u32,
                    r[x + 3] as u32,
                    r[x + 4] as u32,
                    r[x + 5] as u32,
                    r[x + 6] as u32,
                    r[x + 7] as u32,
                ]);
                acc += pixels * u32x8::splat(kv as u32);
            }
            let rounded: u32x8 = (acc + u32x8::splat(1 << 14)) >> 15;
            let vals = rounded.to_array();
            for i in 0..8 {
                out.buf[out_off + x + i] = vals[i] as u8;
            }
            x += 8;
        }
        // Scalar remainder
        while x < wu {
            let mut acc = 0u32;
            for (k, &kv) in kernel.iter().enumerate() {
                acc += rows[k][x] as u32 * kv as u32;
            }
            out.buf[out_off + x] = ((acc + (1 << 14)) >> 15) as u8;
            x += 1;
        }
    }
    (out, tmp.into_buf())
}

/// Apply Gaussian blur or sharpening based on `quad_sigma`.
///
/// - `quad_sigma > 0` → Gaussian blur
/// - `quad_sigma < 0` → Unsharp mask (sharpening)
/// - `quad_sigma == 0` → No filtering
///
/// Pass pre-allocated `out_buf` and `tmp_buf` to reuse memory across calls.
/// Use `Vec::new()` for one-shot usage.
///
/// Returns `(output_image, reclaimed_tmp_buf)`.
pub fn apply_sigma(
    img: &ImageU8,
    quad_sigma: f32,
    out_buf: Vec<u8>,
    tmp_buf: Vec<u8>,
) -> (ImageU8, Vec<u8>) {
    if quad_sigma == 0.0 {
        return (img.clone(), tmp_buf);
    }

    let sigma = quad_sigma.abs();
    let mut ksz = (4.0 * sigma) as usize;
    if ksz.is_multiple_of(2) {
        ksz += 1;
    }
    if ksz <= 1 {
        return (img.clone(), tmp_buf);
    }

    let (blurred, reclaimed_tmp) = gaussian_blur(img, sigma, ksz, tmp_buf, out_buf);

    if quad_sigma > 0.0 {
        (blurred, reclaimed_tmp)
    } else {
        // Unsharp mask: 2*original - blurred, clamped to [0, 255]
        let blur_buf = blurred.buf;
        let mut out = ImageU8::new_reuse(img.width, img.height, Vec::new());
        let wu = img.width as usize;
        for y in 0..img.height {
            let orig_row = img.row(y);
            let out_off = (y * img.width) as usize;

            // SIMD: process 8 pixels at a time
            let mut x = 0usize;
            while x + 8 <= wu {
                let orig = i32x8::new([
                    orig_row[x] as i32,
                    orig_row[x + 1] as i32,
                    orig_row[x + 2] as i32,
                    orig_row[x + 3] as i32,
                    orig_row[x + 4] as i32,
                    orig_row[x + 5] as i32,
                    orig_row[x + 6] as i32,
                    orig_row[x + 7] as i32,
                ]);
                let blur = i32x8::new([
                    blur_buf[out_off + x] as i32,
                    blur_buf[out_off + x + 1] as i32,
                    blur_buf[out_off + x + 2] as i32,
                    blur_buf[out_off + x + 3] as i32,
                    blur_buf[out_off + x + 4] as i32,
                    blur_buf[out_off + x + 5] as i32,
                    blur_buf[out_off + x + 6] as i32,
                    blur_buf[out_off + x + 7] as i32,
                ]);
                let v = orig * i32x8::new([2; 8]) - blur;
                let clamped = v.max(i32x8::new([0; 8])).min(i32x8::new([255; 8]));
                let vals = clamped.to_array();
                for i in 0..8 {
                    out.buf[out_off + x + i] = vals[i] as u8;
                }
                x += 8;
            }

            // Scalar remainder
            while x < wu {
                let v = 2 * orig_row[x] as i32 - blur_buf[out_off + x] as i32;
                out.buf[out_off + x] = v.clamp(0, 255) as u8;
                x += 1;
            }
        }
        (out, reclaimed_tmp)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decimate_factor_1_returns_clone() {
        let mut img = ImageU8::new(4, 4);
        img.set(0, 0, 100);
        let out = decimate(&img, 1, Vec::new());
        assert_eq!(out.width, 4);
        assert_eq!(out.height, 4);
        assert_eq!(out.get(0, 0), 100);
    }

    #[test]
    fn decimate_factor_2_subsamples_top_left() {
        let mut img = ImageU8::new(4, 4);
        // Set distinct values in each 2x2 block's top-left
        img.set(0, 0, 100);
        img.set(1, 0, 200); // not top-left, should be ignored
        img.set(0, 1, 50); // not top-left, should be ignored
        img.set(1, 1, 75); // not top-left, should be ignored
        img.set(2, 0, 180); // top-left of second block
        let out = decimate(&img, 2, Vec::new());
        assert_eq!(out.width, 2);
        assert_eq!(out.height, 2);
        // Subsampling picks top-left pixel of each block
        assert_eq!(out.get(0, 0), 100);
        assert_eq!(out.get(1, 0), 180);
    }

    #[test]
    fn decimate_truncates_partial_blocks() {
        let img = ImageU8::new(5, 5);
        let out = decimate(&img, 2, Vec::new());
        assert_eq!(out.width, 2); // 5/2 = 2
        assert_eq!(out.height, 2);
    }

    #[test]
    fn decimate_reuses_buffer() {
        let img = ImageU8::new(8, 8);
        let buf = Vec::with_capacity(1024);
        let out = decimate(&img, 2, buf);
        assert!(out.buf.capacity() >= 1024);
    }

    #[test]
    fn gaussian_kernel_sums_to_one() {
        let k = gaussian_kernel(1.0, 5);
        assert_eq!(k.len(), 5);
        let sum: u32 = k.iter().map(|&v| v as u32).sum();
        // Fixed-point sum should be close to 1 << 15 = 32768 (within ±1 rounding)
        // fixed-point sum should be close to 1 << 15 = 32768
        assert!((sum as i32 - 32768).unsigned_abs() <= 1);
    }

    #[test]
    fn gaussian_kernel_is_symmetric() {
        let k = gaussian_kernel(1.0, 5);
        assert_eq!(k[0], k[4]);
        assert_eq!(k[1], k[3]);
    }

    #[test]
    fn apply_sigma_zero_returns_clone() {
        let mut img = ImageU8::new(4, 4);
        img.set(2, 2, 128);
        let (out, _) = apply_sigma(&img, 0.0, Vec::new(), Vec::new());
        assert_eq!(out.get(2, 2), 128);
    }

    #[test]
    fn apply_sigma_positive_blurs() {
        let mut img = ImageU8::new(10, 10);
        img.set(5, 5, 255);
        let (out, _) = apply_sigma(&img, 1.0, Vec::new(), Vec::new());
        // After blur, the peak should be reduced
        assert!(out.get(5, 5) < 255);
        // Neighbors should get some value
        assert!(out.get(4, 5) > 0);
    }

    #[test]
    fn apply_sigma_negative_sharpens() {
        // Fill with uniform value, add a dip - sharpening should enhance it
        let mut img = ImageU8::new(10, 10);
        for y in 0..10 {
            for x in 0..10 {
                img.set(x, y, 128);
            }
        }
        img.set(5, 5, 100); // a dip
        let (out, _) = apply_sigma(&img, -1.0, Vec::new(), Vec::new());
        // The dip should be enhanced (lower than original)
        assert!(out.get(5, 5) < 100);
    }

    #[test]
    fn apply_sigma_small_is_noop() {
        // sigma so small that ksz <= 1
        let mut img = ImageU8::new(4, 4);
        img.set(0, 0, 42);
        let (out, _) = apply_sigma(&img, 0.1, Vec::new(), Vec::new());
        assert_eq!(out.get(0, 0), 42);
    }

    /// Reference f32 Gaussian blur for regression testing against fixed-point.
    fn gaussian_blur_f32(img: &ImageU8, sigma: f32, ksz: usize) -> ImageU8 {
        let half = ksz as i32 / 2;
        let mut kernel = Vec::with_capacity(ksz);
        let mut sum = 0.0f32;
        for i in 0..ksz as i32 {
            let x = (i - half) as f32;
            let v = (-x * x / (2.0 * sigma * sigma)).exp();
            kernel.push(v);
            sum += v;
        }
        for v in &mut kernel {
            *v /= sum;
        }

        let w = img.width as i32;
        let h = img.height as i32;

        // Horizontal pass
        let mut tmp = ImageU8::new(img.width, img.height);
        for y in 0..h {
            let row = img.row(y as u32);
            for x in 0..w {
                let mut acc = 0.0f32;
                for k in 0..ksz as i32 {
                    let sx = (x + k - half).clamp(0, w - 1) as usize;
                    acc += row[sx] as f32 * kernel[k as usize];
                }
                tmp.set(x as u32, y as u32, acc.round() as u8);
            }
        }

        // Vertical pass
        let mut out = ImageU8::new(img.width, img.height);
        for y in 0..h {
            let rows: Vec<&[u8]> = (0..ksz as i32)
                .map(|k| tmp.row((y + k - half).clamp(0, h - 1) as u32))
                .collect();
            for x in 0..w as usize {
                let mut acc = 0.0f32;
                for (k, &kv) in kernel.iter().enumerate() {
                    acc += rows[k][x] as f32 * kv;
                }
                out.set(x as u32, y as u32, acc.round() as u8);
            }
        }
        out
    }

    /// Test that blur produces identical results across various image widths,
    /// including SIMD-aligned (32, 64), non-aligned (33, 65), narrow (3), and standard (640).
    #[test]
    fn blur_consistent_across_widths() {
        let sigma = 1.0f32;
        let ksz = 5;

        for width in [3, 32, 33, 64, 65, 640] {
            let height = 20u32;
            let mut img = ImageU8::new(width, height);
            // Fill with a pattern that exercises all pixels
            for y in 0..height {
                for x in 0..width {
                    img.set(x, y, ((x * 7 + y * 13) % 256) as u8);
                }
            }

            let float_result = gaussian_blur_f32(&img, sigma, ksz);
            let (fixed_result, _) = gaussian_blur(&img, sigma, ksz, Vec::new(), Vec::new());

            let mut max_diff = 0i32;
            for y in 0..height {
                for x in 0..width {
                    let diff =
                        (float_result.get(x, y) as i32 - fixed_result.get(x, y) as i32).abs();
                    max_diff = max_diff.max(diff);
                }
            }
            // max pixel diff between float and fixed-point should be <= 1
            assert!(max_diff <= 1);
        }
    }

    /// Scalar reference implementation of unsharp mask for comparison.
    fn unsharp_scalar(orig: &ImageU8, blur_buf: &[u8]) -> ImageU8 {
        let mut out = ImageU8::new(orig.width, orig.height);
        for y in 0..orig.height {
            let orig_row = orig.row(y);
            let out_off = (y * orig.width) as usize;
            for x in 0..orig.width as usize {
                let v = 2 * orig_row[x] as i32 - blur_buf[out_off + x] as i32;
                out.set(x as u32, y, v.clamp(0, 255) as u8);
            }
        }
        out
    }

    /// Test that unsharp mask produces identical results across various widths,
    /// including SIMD-aligned (8, 16, 32, 64), non-aligned (3, 7, 9, 33, 65), and standard (640).
    #[test]
    fn unsharp_consistent_across_widths() {
        for width in [3u32, 7, 8, 9, 15, 16, 17, 32, 33, 64, 65, 640] {
            let height = 20u32;
            let mut img = ImageU8::new(width, height);
            for y in 0..height {
                for x in 0..width {
                    img.set(x, y, ((x * 7 + y * 13) % 256) as u8);
                }
            }

            // Get the blurred image via gaussian_blur
            let (blurred, _) = gaussian_blur(&img, 1.0, 5, Vec::new(), Vec::new());

            // Compute expected via scalar reference
            let reference = unsharp_scalar(&img, &blurred.buf);

            // Compute actual via apply_sigma (which will use SIMD when available)
            let (result, _) = apply_sigma(&img, -1.0, Vec::new(), Vec::new());

            for y in 0..height {
                for x in 0..width {
                    // SIMD and scalar unsharp mask should produce identical results
                    assert_eq!(result.get(x, y), reference.get(x, y));
                }
            }
        }
    }

    #[test]
    fn fixed_point_matches_float_blur() {
        // Build a non-trivial test image with gradients and a bright spot
        let (w, h) = (64, 48);
        let mut img = ImageU8::new(w, h);
        for y in 0..h {
            for x in 0..w {
                // Diagonal gradient + a bright region
                let base = ((x + y) as f32 / (w + h) as f32 * 200.0) as u8;
                img.set(x, y, base);
            }
        }
        // Add a bright spot
        for y in 20..28 {
            for x in 28..36 {
                img.set(x, y, 255);
            }
        }

        // Test several sigma values covering practical range
        for sigma in [0.8f32, 1.0, 1.5, 2.0] {
            let mut ksz = (4.0 * sigma) as usize;
            if ksz % 2 == 0 {
                ksz += 1;
            }
            if ksz <= 1 {
                continue;
            }

            let float_result = gaussian_blur_f32(&img, sigma, ksz);
            let (fixed_result, _) = gaussian_blur(&img, sigma, ksz, Vec::new(), Vec::new());

            let mut max_diff = 0i32;
            for y in 0..h {
                for x in 0..w {
                    let diff =
                        (float_result.get(x, y) as i32 - fixed_result.get(x, y) as i32).abs();
                    max_diff = max_diff.max(diff);
                }
            }
            // max pixel diff between float and fixed-point blur should be <= 1
            assert!(max_diff <= 1);
        }
    }
}
