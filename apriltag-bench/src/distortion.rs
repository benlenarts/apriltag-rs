/// Image distortions for testing detector robustness.
use apriltag::detect::image::ImageU8;
use serde::{Deserialize, Serialize};

/// An image distortion to apply after scene composition.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum Distortion {
    /// Additive Gaussian noise with the given standard deviation.
    GaussianNoise { sigma: f64, seed: u64 },
    /// Salt-and-pepper noise: randomly set pixels to 0 or 255.
    SaltPepper { density: f64, seed: u64 },
    /// Gaussian blur with the given sigma (in pixels).
    GaussianBlur { sigma: f64 },
    /// Scale contrast around the mean: pixel = mean + factor * (pixel - mean).
    ContrastScale { factor: f64 },
    /// Shift brightness by a fixed offset (clamped to 0–255).
    BrightnessShift { offset: i16 },
    /// Linear gradient lighting: brightness varies across the image.
    GradientLighting {
        /// Direction angle in radians (0 = left-to-right, π/2 = top-to-bottom).
        direction: f64,
        /// Minimum brightness factor (darkest side).
        min_factor: f64,
        /// Maximum brightness factor (brightest side).
        max_factor: f64,
    },
    /// Vignette: darken corners, bright center.
    Vignette { strength: f64 },
    /// Black rectangle occlusion.
    Occlude { rect: [u32; 4] },
}

/// Apply a sequence of distortions to an image in-place.
pub fn apply(img: &mut ImageU8, distortions: &[Distortion]) {
    for d in distortions {
        apply_one(img, d);
    }
}

fn apply_one(img: &mut ImageU8, d: &Distortion) {
    match d {
        Distortion::GaussianNoise { sigma, seed } => apply_gaussian_noise(img, *sigma, *seed),
        Distortion::SaltPepper { density, seed } => apply_salt_pepper(img, *density, *seed),
        Distortion::GaussianBlur { sigma } => apply_gaussian_blur(img, *sigma),
        Distortion::ContrastScale { factor } => apply_contrast_scale(img, *factor),
        Distortion::BrightnessShift { offset } => apply_brightness_shift(img, *offset),
        Distortion::GradientLighting {
            direction,
            min_factor,
            max_factor,
        } => apply_gradient_lighting(img, *direction, *min_factor, *max_factor),
        Distortion::Vignette { strength } => apply_vignette(img, *strength),
        Distortion::Occlude { rect } => apply_occlude(img, rect),
    }
}

/// Simple LCG pseudo-random number generator (deterministic, no_std compatible).
struct Rng {
    state: u64,
}

impl Rng {
    fn new(seed: u64) -> Self {
        Self {
            state: seed.wrapping_add(1),
        }
    }

    fn next_u64(&mut self) -> u64 {
        // LCG with Knuth's constants
        self.state = self
            .state
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        self.state
    }

    /// Generate a uniform f64 in [0, 1).
    fn next_f64(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }

    /// Generate an approximately Gaussian random number using Box-Muller.
    fn next_gaussian(&mut self) -> f64 {
        let u1 = self.next_f64().max(1e-15); // avoid log(0)
        let u2 = self.next_f64();
        (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos()
    }
}

fn apply_gaussian_noise(img: &mut ImageU8, sigma: f64, seed: u64) {
    let mut rng = Rng::new(seed);
    for y in 0..img.height {
        for x in 0..img.width {
            let val = img.get(x, y) as f64 + rng.next_gaussian() * sigma;
            img.set(x, y, val.round().clamp(0.0, 255.0) as u8);
        }
    }
}

fn apply_salt_pepper(img: &mut ImageU8, density: f64, seed: u64) {
    let mut rng = Rng::new(seed);
    for y in 0..img.height {
        for x in 0..img.width {
            let r = rng.next_f64();
            if r < density / 2.0 {
                img.set(x, y, 0); // pepper
            } else if r < density {
                img.set(x, y, 255); // salt
            }
        }
    }
}

fn apply_gaussian_blur(img: &mut ImageU8, sigma: f64) {
    if sigma <= 0.0 {
        return;
    }
    // Separable Gaussian blur
    let radius = (sigma * 3.0).ceil() as usize;
    let kernel: Vec<f64> = (0..=radius)
        .map(|i| (-0.5 * (i as f64 / sigma).powi(2)).exp())
        .collect();
    let sum: f64 = kernel[0] + 2.0 * kernel[1..].iter().sum::<f64>();
    let kernel: Vec<f64> = kernel.iter().map(|v| v / sum).collect();

    // Horizontal pass
    let mut tmp = ImageU8::new(img.width, img.height);
    for y in 0..img.height {
        for x in 0..img.width {
            let mut acc = 0.0;
            for (i, &k) in kernel.iter().enumerate() {
                let xi = (x as i64 + i as i64).min(img.width as i64 - 1) as u32;
                acc += k * img.get(xi, y) as f64;
                if i > 0 {
                    let xi = (x as i64 - i as i64).max(0) as u32;
                    acc += k * img.get(xi, y) as f64;
                }
            }
            tmp.set(x, y, acc.round().clamp(0.0, 255.0) as u8);
        }
    }

    // Vertical pass
    for y in 0..img.height {
        for x in 0..img.width {
            let mut acc = 0.0;
            for (i, &k) in kernel.iter().enumerate() {
                let yi = (y as i64 + i as i64).min(img.height as i64 - 1) as u32;
                acc += k * tmp.get(x, yi) as f64;
                if i > 0 {
                    let yi = (y as i64 - i as i64).max(0) as u32;
                    acc += k * tmp.get(x, yi) as f64;
                }
            }
            img.set(x, y, acc.round().clamp(0.0, 255.0) as u8);
        }
    }
}

fn apply_contrast_scale(img: &mut ImageU8, factor: f64) {
    // Compute mean
    let mut sum = 0u64;
    let count = img.width as u64 * img.height as u64;
    for y in 0..img.height {
        for x in 0..img.width {
            sum += img.get(x, y) as u64;
        }
    }
    let mean = sum as f64 / count as f64;

    for y in 0..img.height {
        for x in 0..img.width {
            let val = mean + factor * (img.get(x, y) as f64 - mean);
            img.set(x, y, val.round().clamp(0.0, 255.0) as u8);
        }
    }
}

fn apply_brightness_shift(img: &mut ImageU8, offset: i16) {
    for y in 0..img.height {
        for x in 0..img.width {
            let val = img.get(x, y) as i16 + offset;
            img.set(x, y, val.clamp(0, 255) as u8);
        }
    }
}

fn apply_gradient_lighting(img: &mut ImageU8, direction: f64, min_factor: f64, max_factor: f64) {
    let cos_d = direction.cos();
    let sin_d = direction.sin();
    let cx = img.width as f64 / 2.0;
    let cy = img.height as f64 / 2.0;
    let max_proj = (cx * cos_d.abs() + cy * sin_d.abs()).max(1.0);

    for y in 0..img.height {
        for x in 0..img.width {
            let dx = x as f64 + 0.5 - cx;
            let dy = y as f64 + 0.5 - cy;
            let proj = dx * cos_d + dy * sin_d;
            // Map [-max_proj, max_proj] → [min_factor, max_factor]
            let t = (proj / max_proj + 1.0) * 0.5; // 0..1
            let factor = min_factor + t * (max_factor - min_factor);
            let val = img.get(x, y) as f64 * factor;
            img.set(x, y, val.round().clamp(0.0, 255.0) as u8);
        }
    }
}

fn apply_vignette(img: &mut ImageU8, strength: f64) {
    let cx = img.width as f64 / 2.0;
    let cy = img.height as f64 / 2.0;
    let max_r = (cx * cx + cy * cy).sqrt();

    for y in 0..img.height {
        for x in 0..img.width {
            let dx = x as f64 + 0.5 - cx;
            let dy = y as f64 + 0.5 - cy;
            let r = (dx * dx + dy * dy).sqrt() / max_r;
            let factor = 1.0 - strength * r * r;
            let val = img.get(x, y) as f64 * factor;
            img.set(x, y, val.round().clamp(0.0, 255.0) as u8);
        }
    }
}

fn apply_occlude(img: &mut ImageU8, rect: &[u32; 4]) {
    let [x0, y0, x1, y1] = *rect;
    let x0 = x0.min(img.width);
    let y0 = y0.min(img.height);
    let x1 = x1.min(img.width);
    let y1 = y1.min(img.height);
    for y in y0..y1 {
        for x in x0..x1 {
            img.set(x, y, 0);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn uniform_image(width: u32, height: u32, val: u8) -> ImageU8 {
        let buf = vec![val; (width * height) as usize];
        ImageU8::from_buf(width, height, width, buf)
    }

    #[test]
    fn gaussian_noise_changes_pixels() {
        let mut img = uniform_image(50, 50, 128);
        apply_gaussian_noise(&mut img, 20.0, 42);

        // Not all pixels should still be 128
        let changed = (0..50)
            .flat_map(|y| (0..50).map(move |x| (x, y)))
            .filter(|&(x, y)| img.get(x, y) != 128)
            .count();
        assert!(changed > 100, "expected noise to change pixels, changed={changed}");
    }

    #[test]
    fn gaussian_noise_deterministic() {
        let mut img1 = uniform_image(20, 20, 128);
        let mut img2 = uniform_image(20, 20, 128);
        apply_gaussian_noise(&mut img1, 10.0, 99);
        apply_gaussian_noise(&mut img2, 10.0, 99);

        for y in 0..20 {
            for x in 0..20 {
                assert_eq!(img1.get(x, y), img2.get(x, y));
            }
        }
    }

    #[test]
    fn salt_pepper_creates_extremes() {
        let mut img = uniform_image(100, 100, 128);
        apply_salt_pepper(&mut img, 0.5, 42);

        let zeros = (0..100)
            .flat_map(|y| (0..100).map(move |x| (x, y)))
            .filter(|&(x, y)| img.get(x, y) == 0)
            .count();
        let ones = (0..100)
            .flat_map(|y| (0..100).map(move |x| (x, y)))
            .filter(|&(x, y)| img.get(x, y) == 255)
            .count();

        assert!(zeros > 100, "expected salt pixels, got {zeros}");
        assert!(ones > 100, "expected pepper pixels, got {ones}");
    }

    #[test]
    fn gaussian_blur_smooths() {
        // Create a sharp edge: left half = 0, right half = 255
        let mut img = ImageU8::new(100, 10);
        for y in 0..10 {
            for x in 50..100 {
                img.set(x, y, 255);
            }
        }

        apply_gaussian_blur(&mut img, 5.0);

        // After blur, the edge should be gradual
        let at_edge = img.get(50, 5);
        assert!(
            at_edge > 10 && at_edge < 245,
            "blur should smooth edge, got {at_edge}"
        );
    }

    #[test]
    fn contrast_scale_expands() {
        let mut img = uniform_image(10, 10, 128);
        img.set(5, 5, 200);
        apply_contrast_scale(&mut img, 2.0);

        // Mean ≈ 128. Pixel 200: 128 + 2*(200-128) = 128+144 = 272 → clamped to 255
        assert_eq!(img.get(5, 5), 255);
    }

    #[test]
    fn contrast_scale_reduces() {
        let mut img = ImageU8::new(10, 10);
        for y in 0..10 {
            for x in 0..10 {
                img.set(x, y, if x < 5 { 0 } else { 200 });
            }
        }
        apply_contrast_scale(&mut img, 0.5);

        // After reducing contrast, range should be halved around mean
        let v0 = img.get(0, 0);
        let v1 = img.get(9, 0);
        assert!(
            (v1 as i16 - v0 as i16).abs() < 120,
            "contrast should be reduced: {} vs {}",
            v0,
            v1
        );
    }

    #[test]
    fn brightness_shift_up() {
        let mut img = uniform_image(10, 10, 100);
        apply_brightness_shift(&mut img, 50);
        assert_eq!(img.get(5, 5), 150);
    }

    #[test]
    fn brightness_shift_clamps() {
        let mut img = uniform_image(10, 10, 200);
        apply_brightness_shift(&mut img, 100);
        assert_eq!(img.get(5, 5), 255); // clamped
    }

    #[test]
    fn gradient_lighting_left_right() {
        let mut img = uniform_image(100, 10, 200);
        apply_gradient_lighting(&mut img, 0.0, 0.5, 1.5);

        let left = img.get(0, 5);
        let right = img.get(99, 5);
        assert!(
            left < right,
            "gradient L→R: left={left} should be darker than right={right}"
        );
    }

    #[test]
    fn vignette_darkens_corners() {
        let mut img = uniform_image(100, 100, 200);
        apply_vignette(&mut img, 1.0);

        let center = img.get(50, 50);
        let corner = img.get(0, 0);
        assert!(
            corner < center,
            "vignette: corner={corner} should be darker than center={center}"
        );
    }

    #[test]
    fn occlude_fills_black() {
        let mut img = uniform_image(100, 100, 200);
        apply_occlude(&mut img, &[10, 10, 20, 20]);

        assert_eq!(img.get(15, 15), 0); // inside rect
        assert_eq!(img.get(5, 5), 200); // outside rect
    }

    #[test]
    fn apply_chain() {
        let mut img = uniform_image(50, 50, 128);
        apply(
            &mut img,
            &[
                Distortion::BrightnessShift { offset: 20 },
                Distortion::GaussianNoise {
                    sigma: 5.0,
                    seed: 1,
                },
            ],
        );

        // After brightness shift of +20 and noise, mean should be roughly 148
        let mean: f64 = (0..50)
            .flat_map(|y| (0..50).map(move |x| (x, y)))
            .map(|(x, y)| img.get(x, y) as f64)
            .sum::<f64>()
            / 2500.0;
        assert!(
            (mean - 148.0).abs() < 10.0,
            "mean after chain: {mean}"
        );
    }
}
