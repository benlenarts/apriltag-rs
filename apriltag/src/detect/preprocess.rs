use super::image::ImageU8;

/// Decimate an image by factor `f`, averaging each f×f block.
pub fn decimate(img: &ImageU8, f: u32) -> ImageU8 {
    if f <= 1 {
        return img.clone();
    }

    let out_w = img.width / f;
    let out_h = img.height / f;
    let mut out = ImageU8::new(out_w, out_h);
    let area = (f * f) as u32;

    for oy in 0..out_h {
        for ox in 0..out_w {
            let mut sum = 0u32;
            for dy in 0..f {
                for dx in 0..f {
                    sum += img.get(ox * f + dx, oy * f + dy) as u32;
                }
            }
            out.set(ox, oy, (sum / area) as u8);
        }
    }
    out
}

/// Build a 1D Gaussian kernel with the given sigma and kernel size.
///
/// Returns normalized kernel values. `ksz` must be odd.
fn gaussian_kernel(sigma: f32, ksz: usize) -> Vec<f32> {
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
    kernel
}

/// Apply separable Gaussian blur with the given sigma and kernel size.
fn gaussian_blur(img: &ImageU8, sigma: f32, ksz: usize) -> ImageU8 {
    let kernel = gaussian_kernel(sigma, ksz);
    let half = ksz as i32 / 2;
    let w = img.width as i32;
    let h = img.height as i32;

    // Horizontal pass
    let mut tmp = ImageU8::new(img.width, img.height);
    for y in 0..h {
        for x in 0..w {
            let mut sum = 0.0f32;
            for k in 0..ksz as i32 {
                let sx = (x + k - half).clamp(0, w - 1);
                sum += img.get(sx as u32, y as u32) as f32 * kernel[k as usize];
            }
            tmp.set(x as u32, y as u32, sum.round() as u8);
        }
    }

    // Vertical pass
    let mut out = ImageU8::new(img.width, img.height);
    for y in 0..h {
        for x in 0..w {
            let mut sum = 0.0f32;
            for k in 0..ksz as i32 {
                let sy = (y + k - half).clamp(0, h - 1);
                sum += tmp.get(x as u32, sy as u32) as f32 * kernel[k as usize];
            }
            out.set(x as u32, y as u32, sum.round() as u8);
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
    if quad_sigma == 0.0 {
        return img.clone();
    }

    let sigma = quad_sigma.abs();
    let mut ksz = (4.0 * sigma) as usize;
    if ksz % 2 == 0 {
        ksz += 1;
    }
    if ksz <= 1 {
        return img.clone();
    }

    let blurred = gaussian_blur(img, sigma, ksz);

    if quad_sigma > 0.0 {
        blurred
    } else {
        // Unsharp mask: 2*original - blurred
        let mut out = ImageU8::new(img.width, img.height);
        for y in 0..img.height {
            for x in 0..img.width {
                let v = 2 * img.get(x, y) as i32 - blurred.get(x, y) as i32;
                out.set(x, y, v.clamp(0, 255) as u8);
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
    fn decimate_factor_2_averages_blocks() {
        let mut img = ImageU8::new(4, 4);
        // Set 2x2 block at (0,0) to known values
        img.set(0, 0, 100);
        img.set(1, 0, 200);
        img.set(0, 1, 0);
        img.set(1, 1, 100);
        let out = decimate(&img, 2);
        assert_eq!(out.width, 2);
        assert_eq!(out.height, 2);
        // Average of 100, 200, 0, 100 = 400/4 = 100
        assert_eq!(out.get(0, 0), 100);
    }

    #[test]
    fn decimate_truncates_partial_blocks() {
        let img = ImageU8::new(5, 5);
        let out = decimate(&img, 2);
        assert_eq!(out.width, 2); // 5/2 = 2
        assert_eq!(out.height, 2);
    }

    #[test]
    fn gaussian_kernel_sums_to_one() {
        let k = gaussian_kernel(1.0, 5);
        assert_eq!(k.len(), 5);
        let sum: f32 = k.iter().sum();
        assert!((sum - 1.0).abs() < 1e-5);
    }

    #[test]
    fn gaussian_kernel_is_symmetric() {
        let k = gaussian_kernel(1.0, 5);
        assert!((k[0] - k[4]).abs() < 1e-6);
        assert!((k[1] - k[3]).abs() < 1e-6);
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
}
