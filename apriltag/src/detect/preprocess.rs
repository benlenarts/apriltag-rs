use super::image::ImageU8;

/// Decimate an image by factor `f`, subsampling every f-th pixel.
///
/// Picks the top-left pixel of each f×f block, matching the C reference
/// implementation (`image_u8_decimate`).
pub fn decimate(img: &ImageU8, f: u32) -> ImageU8 {
    if f <= 1 {
        return img.clone();
    }
    decimate_into(img, f, Vec::new())
}

/// Like [`decimate`], but reuses `buf` for the output image to avoid allocation.
pub fn decimate_into(img: &ImageU8, f: u32, buf: Vec<u8>) -> ImageU8 {
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
#[cfg(test)]
fn gaussian_blur(img: &ImageU8, sigma: f32, ksz: usize) -> ImageU8 {
    gaussian_blur_into(img, sigma, ksz, Vec::new(), Vec::new())
}

/// Like [`gaussian_blur`], but reuses `out_buf` and `tmp_buf` to avoid allocation.
fn gaussian_blur_into(
    img: &ImageU8,
    sigma: f32,
    ksz: usize,
    tmp_buf: Vec<u8>,
    out_buf: Vec<u8>,
) -> ImageU8 {
    let kernel = gaussian_kernel(sigma, ksz);
    let half = ksz as i32 / 2;
    let w = img.width as i32;
    let h = img.height as i32;

    // Horizontal pass
    let mut tmp = ImageU8::new_reuse(img.width, img.height, tmp_buf);
    for y in 0..h {
        let row = img.row(y as u32);
        for x in 0..w {
            let mut acc = 0u32;
            for k in 0..ksz as i32 {
                let sx = (x + k - half).clamp(0, w - 1) as usize;
                acc += row[sx] as u32 * kernel[k as usize] as u32;
            }
            tmp.set(x as u32, y as u32, ((acc + (1 << 14)) >> 15) as u8);
        }
    }

    // Vertical pass
    let mut out = ImageU8::new_reuse(img.width, img.height, out_buf);
    for y in 0..h {
        // Pre-fetch row slices for all kernel taps
        let rows: Vec<&[u8]> = (0..ksz as i32)
            .map(|k| tmp.row((y + k - half).clamp(0, h - 1) as u32))
            .collect();
        for x in 0..w as usize {
            let mut acc = 0u32;
            for (k, &kv) in kernel.iter().enumerate() {
                acc += rows[k][x] as u32 * kv as u32;
            }
            out.set(x as u32, y as u32, ((acc + (1 << 14)) >> 15) as u8);
        }
    }
    out
}

/// Apply Gaussian blur or sharpening based on `quad_sigma`.
///
/// - `quad_sigma > 0` → Gaussian blur
/// - `quad_sigma < 0` → Unsharp mask (sharpening)
/// - `quad_sigma == 0` → No filtering
pub fn apply_sigma(img: &ImageU8, quad_sigma: f32) -> ImageU8 {
    apply_sigma_into(img, quad_sigma, Vec::new(), Vec::new())
}

/// Like [`apply_sigma`], but reuses `out_buf` and `tmp_buf` to avoid allocation.
///
/// `out_buf` is used for the final output image. `tmp_buf` is used for the
/// intermediate horizontal-pass buffer inside the Gaussian blur.
pub fn apply_sigma_into(
    img: &ImageU8,
    quad_sigma: f32,
    out_buf: Vec<u8>,
    tmp_buf: Vec<u8>,
) -> ImageU8 {
    if quad_sigma == 0.0 {
        return img.clone();
    }

    let sigma = quad_sigma.abs();
    let mut ksz = (4.0 * sigma) as usize;
    if ksz.is_multiple_of(2) {
        ksz += 1;
    }
    if ksz <= 1 {
        return img.clone();
    }

    let blurred = gaussian_blur_into(img, sigma, ksz, tmp_buf, out_buf);

    if quad_sigma > 0.0 {
        blurred
    } else {
        // Unsharp mask: 2*original - blurred
        // Reuse the blurred image's buffer for the output
        let blur_buf = blurred.buf;
        let mut out = ImageU8::new_reuse(img.width, img.height, Vec::new());
        for y in 0..img.height {
            let orig_row = img.row(y);
            let out_off = (y * img.width) as usize;
            for x in 0..img.width as usize {
                let v = 2 * orig_row[x] as i32 - blur_buf[out_off + x] as i32;
                out.set(x as u32, y, v.clamp(0, 255) as u8);
            }
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decimate_factor_1_returns_clone() {
        let mut img = ImageU8::new(4, 4);
        img.set(0, 0, 100);
        let out = decimate(&img, 1);
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
        let out = decimate(&img, 2);
        assert_eq!(out.width, 2);
        assert_eq!(out.height, 2);
        // Subsampling picks top-left pixel of each block
        assert_eq!(out.get(0, 0), 100);
        assert_eq!(out.get(1, 0), 180);
    }

    #[test]
    fn decimate_truncates_partial_blocks() {
        let img = ImageU8::new(5, 5);
        let out = decimate(&img, 2);
        assert_eq!(out.width, 2); // 5/2 = 2
        assert_eq!(out.height, 2);
    }

    #[test]
    fn decimate_into_matches_decimate() {
        let mut img = ImageU8::new(8, 8);
        for y in 0..8 {
            for x in 0..8 {
                img.set(x, y, (x * 31 + y * 17) as u8);
            }
        }
        let expected = decimate(&img, 2);
        let actual = decimate_into(&img, 2, Vec::new());
        assert_eq!(expected.buf, actual.buf);
        assert_eq!(expected.width, actual.width);
        assert_eq!(expected.height, actual.height);
    }

    #[test]
    fn decimate_into_reuses_buffer() {
        let img = ImageU8::new(8, 8);
        let buf = Vec::with_capacity(1024);
        let out = decimate_into(&img, 2, buf);
        assert!(out.buf.capacity() >= 1024);
    }

    #[test]
    fn decimate_into_factor_1_clones() {
        let mut img = ImageU8::new(4, 4);
        img.set(0, 0, 42);
        let out = decimate_into(&img, 1, Vec::new());
        assert_eq!(out.get(0, 0), 42);
        assert_eq!(out.width, 4);
    }

    #[test]
    fn gaussian_kernel_sums_to_one() {
        let k = gaussian_kernel(1.0, 5);
        assert_eq!(k.len(), 5);
        let sum: u32 = k.iter().map(|&v| v as u32).sum();
        // Fixed-point sum should be close to 1 << 15 = 32768 (within ±1 rounding)
        assert!(
            (sum as i32 - 32768).unsigned_abs() <= 1,
            "kernel sum {sum} not close to 32768"
        );
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
        let out = apply_sigma(&img, 0.0);
        assert_eq!(out.get(2, 2), 128);
    }

    #[test]
    fn apply_sigma_positive_blurs() {
        let mut img = ImageU8::new(10, 10);
        img.set(5, 5, 255);
        let out = apply_sigma(&img, 1.0);
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
        let out = apply_sigma(&img, -1.0);
        // The dip should be enhanced (lower than original)
        assert!(out.get(5, 5) < 100);
    }

    #[test]
    fn apply_sigma_small_is_noop() {
        // sigma so small that ksz <= 1
        let mut img = ImageU8::new(4, 4);
        img.set(0, 0, 42);
        let out = apply_sigma(&img, 0.1);
        assert_eq!(out.get(0, 0), 42);
    }

    #[test]
    fn apply_sigma_into_matches_apply_sigma_blur() {
        let mut img = ImageU8::new(10, 10);
        img.set(5, 5, 255);
        let expected = apply_sigma(&img, 1.0);
        let actual = apply_sigma_into(&img, 1.0, Vec::new(), Vec::new());
        assert_eq!(expected.buf, actual.buf);
    }

    #[test]
    fn apply_sigma_into_matches_apply_sigma_sharpen() {
        let mut img = ImageU8::new(10, 10);
        for y in 0..10 {
            for x in 0..10 {
                img.set(x, y, 128);
            }
        }
        img.set(5, 5, 100);
        let expected = apply_sigma(&img, -1.0);
        let actual = apply_sigma_into(&img, -1.0, Vec::new(), Vec::new());
        assert_eq!(expected.buf, actual.buf);
    }

    #[test]
    fn apply_sigma_into_zero_clones() {
        let mut img = ImageU8::new(4, 4);
        img.set(0, 0, 42);
        let out = apply_sigma_into(&img, 0.0, Vec::new(), Vec::new());
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
            let fixed_result = gaussian_blur(&img, sigma, ksz);

            let mut max_diff = 0i32;
            for y in 0..h {
                for x in 0..w {
                    let diff =
                        (float_result.get(x, y) as i32 - fixed_result.get(x, y) as i32).abs();
                    max_diff = max_diff.max(diff);
                }
            }
            assert!(
                max_diff <= 1,
                "sigma={sigma}: max pixel diff {max_diff} exceeds tolerance 1"
            );
        }
    }
}
