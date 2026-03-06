use crate::family::{FamilyId, TagFamily};
use crate::hamming;

use super::homography::Homography;
use super::image::ImageU8;

/// Result of decoding a tag from a quad.
#[derive(Debug, Clone)]
pub struct DecodeResult {
    pub family_id: FamilyId,
    pub id: i32,
    pub hamming: i32,
    pub decision_margin: f32,
    pub rotation: i32,
}

/// A spatially-varying intensity model: intensity(x,y) = C[0]*x + C[1]*y + C[2].
#[derive(Debug, Clone, Default)]
struct GrayModel {
    a: [[f64; 3]; 3], // J'J (upper triangular)
    b: [f64; 3],      // J'gray
    c: [f64; 3],      // solved coefficients
}

impl GrayModel {
    fn add(&mut self, x: f64, y: f64, gray: f64) {
        self.a[0][0] += x * x;
        self.a[0][1] += x * y;
        self.a[0][2] += x;
        self.a[1][1] += y * y;
        self.a[1][2] += y;
        self.a[2][2] += 1.0;
        self.b[0] += x * gray;
        self.b[1] += y * gray;
        self.b[2] += gray;
    }

    fn solve(&mut self) {
        // Fill symmetric part
        self.a[1][0] = self.a[0][1];
        self.a[2][0] = self.a[0][2];
        self.a[2][1] = self.a[1][2];

        // Solve Ac = b using Gaussian elimination
        let mut aug = [[0.0f64; 4]; 3];
        for i in 0..3 {
            for j in 0..3 {
                aug[i][j] = self.a[i][j];
            }
            aug[i][3] = self.b[i];
        }

        for col in 0..3 {
            let mut max_val = aug[col][col].abs();
            let mut max_row = col;
            for row in (col + 1)..3 {
                if aug[row][col].abs() > max_val {
                    max_val = aug[row][col].abs();
                    max_row = row;
                }
            }
            if max_val < 1e-20 {
                return;
            }
            if max_row != col {
                aug.swap(col, max_row);
            }
            let pivot = aug[col][col];
            for row in (col + 1)..3 {
                let factor = aug[row][col] / pivot;
                for c in col..4 {
                    aug[row][c] -= factor * aug[col][c];
                }
            }
        }

        // Back-substitute
        for row in (0..3).rev() {
            let mut sum = aug[row][3];
            for c in (row + 1)..3 {
                sum -= aug[row][c] * self.c[c];
            }
            if aug[row][row].abs() > 1e-20 {
                self.c[row] = sum / aug[row][row];
            }
        }
    }

    fn interpolate(&self, x: f64, y: f64) -> f64 {
        self.c[0] * x + self.c[1] * y + self.c[2]
    }
}

/// Quick decode lookup table for fast code matching.
#[derive(Debug, Clone)]
pub struct QuickDecode {
    nbits: u32,
    chunk_mask: u32,
    shifts: [u32; 4],
    chunk_offsets: [Vec<u16>; 4],
    chunk_ids: [Vec<u16>; 4],
    maxhamming: u32,
}

impl QuickDecode {
    /// Build a quick decode table from a tag family.
    pub fn new(family: &TagFamily, maxhamming: u32) -> Self {
        let nbits = family.layout.nbits as u32;
        let chunk_size = nbits.div_ceil(4);
        let capacity = 1u32 << chunk_size;
        let chunk_mask = capacity - 1;
        let shifts = [0, chunk_size, 2 * chunk_size, 3 * chunk_size];
        let ncodes = family.codes.len();

        let mut chunk_offsets = [
            vec![0u16; capacity as usize + 1],
            vec![0u16; capacity as usize + 1],
            vec![0u16; capacity as usize + 1],
            vec![0u16; capacity as usize + 1],
        ];
        let mut chunk_ids = [
            vec![0u16; ncodes],
            vec![0u16; ncodes],
            vec![0u16; ncodes],
            vec![0u16; ncodes],
        ];

        for j in 0..4 {
            // Count frequencies
            let mut counts = vec![0u16; capacity as usize];
            for &code in family.codes.iter() {
                let val = ((code >> shifts[j]) & chunk_mask as u64) as usize;
                counts[val] += 1;
            }

            // Prefix sum
            chunk_offsets[j][0] = 0;
            for v in 0..capacity as usize {
                chunk_offsets[j][v + 1] = chunk_offsets[j][v] + counts[v];
            }

            // Fill ids
            let mut pos = chunk_offsets[j].clone();
            for (idx, &code) in family.codes.iter().enumerate() {
                let val = ((code >> shifts[j]) & chunk_mask as u64) as usize;
                chunk_ids[j][pos[val] as usize] = idx as u16;
                pos[val] += 1;
            }
        }

        Self {
            nbits,
            chunk_mask,
            shifts,
            chunk_offsets,
            chunk_ids,
            maxhamming,
        }
    }

    /// Look up a code in the quick decode table.
    ///
    /// Returns (id, hamming, rotation) or None if no match within maxhamming.
    pub fn decode(&self, family: &TagFamily, rcode: u64) -> Option<(i32, i32, i32)> {
        let mut rcode = rcode;
        let nbits = self.nbits;

        for rotation in 0..4 {
            for j in 0..4 {
                let val = ((rcode >> self.shifts[j]) & self.chunk_mask as u64) as usize;
                let start = self.chunk_offsets[j][val] as usize;
                let end = self.chunk_offsets[j][val + 1] as usize;

                for k in start..end {
                    let id = self.chunk_ids[j][k] as usize;
                    let h = (family.codes[id] ^ rcode).count_ones();
                    if h <= self.maxhamming {
                        return Some((id as i32, h as i32, rotation));
                    }
                }
            }

            rcode = hamming::rotate90(rcode, nbits);
        }

        None
    }
}

/// Attempt to decode a tag from a quad using the given tag family.
pub fn decode_quad(
    img: &ImageU8,
    family: &TagFamily,
    qd: &QuickDecode,
    h: &Homography,
    reversed_border: bool,
    decode_sharpening: f64,
) -> Option<DecodeResult> {
    let w = family.layout.border_width as f64;
    let total_width = family.layout.grid_size;

    // Build gray models for white and black borders
    let mut white_model = GrayModel::default();
    let mut black_model = GrayModel::default();

    // Border sampling patterns: (start_x, start_y, dx, dy, is_white)
    let patterns: [(f64, f64, f64, f64, bool); 8] = [
        (-0.5, 0.5, 0.0, 1.0, true),     // left white column
        (0.5, 0.5, 0.0, 1.0, false),     // left black column
        (w + 0.5, 0.5, 0.0, 1.0, true),  // right white column
        (w - 0.5, 0.5, 0.0, 1.0, false), // right black column
        (0.5, -0.5, 1.0, 0.0, true),     // top white row
        (0.5, 0.5, 1.0, 0.0, false),     // top black row
        (0.5, w + 0.5, 1.0, 0.0, true),  // bottom white row
        (0.5, w - 0.5, 1.0, 0.0, false), // bottom black row
    ];

    for &(sx, sy, dx, dy, is_white) in &patterns {
        let n = w as usize;
        for step in 0..n {
            let bx = sx + dx * step as f64;
            let by = sy + dy * step as f64;

            let tagx = 2.0 * (bx / w - 0.5);
            let tagy = 2.0 * (by / w - 0.5);

            let (px, py) = h.project(tagx, tagy);

            if px < 0.0 || py < 0.0 || px >= img.width as f64 - 1.0 || py >= img.height as f64 - 1.0
            {
                continue;
            }

            let gray = img.interpolate(px, py);

            if is_white {
                white_model.add(tagx, tagy, gray);
            } else {
                black_model.add(tagx, tagy, gray);
            }
        }
    }

    white_model.solve();
    black_model.solve();

    // Polarity check
    let white_at_center = white_model.interpolate(0.0, 0.0);
    let black_at_center = black_model.interpolate(0.0, 0.0);

    if !reversed_border && white_at_center <= black_at_center {
        return None;
    }
    if reversed_border && white_at_center >= black_at_center {
        return None;
    }

    // Sample data bits
    let nbits = family.layout.nbits;
    let bit_locs = &family.bit_locations;

    // Create values grid for sharpening
    let mut values = vec![vec![0.0f64; total_width]; total_width];

    for i in 0..nbits {
        let bx = bit_locs[i].x as f64 + 0.5;
        let by = bit_locs[i].y as f64 + 0.5;

        let tagx = 2.0 * (bx / w - 0.5);
        let tagy = 2.0 * (by / w - 0.5);

        let (px, py) = h.project(tagx, tagy);
        let pixel_val = img.interpolate(px, py);
        let thresh =
            (black_model.interpolate(tagx, tagy) + white_model.interpolate(tagx, tagy)) / 2.0;

        // Grid position for sharpening
        let gx = (bit_locs[i].x + family.layout.border_start as i32) as usize;
        let gy = (bit_locs[i].y + family.layout.border_start as i32) as usize;
        if gx < total_width && gy < total_width {
            values[gy][gx] = pixel_val - thresh;
        }
    }

    // Apply decode sharpening
    if decode_sharpening > 0.0 && total_width >= 3 {
        let orig = values.clone();
        for i in 0..nbits {
            let gx = (bit_locs[i].x + family.layout.border_start as i32) as usize;
            let gy = (bit_locs[i].y + family.layout.border_start as i32) as usize;
            if gx >= 1 && gx + 1 < total_width && gy >= 1 && gy + 1 < total_width {
                let laplacian = 4.0 * orig[gy][gx]
                    - orig[gy - 1][gx]
                    - orig[gy + 1][gx]
                    - orig[gy][gx - 1]
                    - orig[gy][gx + 1];
                values[gy][gx] += decode_sharpening * laplacian;
            }
        }
    }

    // Extract code and compute decision margin
    let mut rcode = 0u64;
    let mut white_score = 0.0f64;
    let mut black_score = 0.0f64;
    let mut white_count = 1.0f64; // Laplace smoothing
    let mut black_count = 1.0f64;

    for i in 0..nbits {
        rcode <<= 1;
        let gx = (bit_locs[i].x + family.layout.border_start as i32) as usize;
        let gy = (bit_locs[i].y + family.layout.border_start as i32) as usize;
        let v = if gx < total_width && gy < total_width {
            values[gy][gx]
        } else {
            0.0
        };

        if v > 0.0 {
            rcode |= 1;
            white_score += v;
            white_count += 1.0;
        } else {
            black_score -= v;
            black_count += 1.0;
        }
    }

    let decision_margin = (white_score / white_count).min(black_score / black_count) as f32;
    // white_score, black_score >= 0 and counts >= 1, so margin is always >= 0
    debug_assert!(decision_margin >= 0.0);

    // Quick decode
    let (id, hamming_dist, rotation) = qd.decode(family, rcode)?;

    Some(DecodeResult {
        family_id: family.config.name.clone(),
        id,
        hamming: hamming_dist,
        decision_margin,
        rotation,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn gray_model_solve_singular() {
        // With only a single observation, the system is rank-deficient;
        // solve() should bail out and leave coefficients at zero.
        let mut gm = GrayModel::default();
        gm.add(1.0, 1.0, 42.0);
        gm.solve();
        assert_eq!(gm.c, [0.0, 0.0, 0.0]);
    }

    #[test]
    #[cfg(feature = "family-tag16h5")]
    fn decode_quad_reversed_border_wrong_polarity() {
        // Render a normal-polarity tag, then call decode_quad with reversed_border=true.
        // The polarity check should reject it.
        let family = crate::family::tag16h5();
        let rendered = crate::render::render(&family.layout, family.codes[0]);
        let qd = QuickDecode::new(&family, 2);

        let w = family.layout.border_width as f64;
        let scale = 10.0;
        let ox = 60.0;
        let oy = 60.0;
        let gs = rendered.grid_size as f64;

        // Build synthetic image with the tag
        let mut img = ImageU8::new(200, 200);
        for y in 0..200u32 {
            for x in 0..200u32 {
                img.set(x, y, 255);
            }
        }
        for ty in 0..rendered.grid_size {
            for tx in 0..rendered.grid_size {
                let pixel = rendered.pixel(tx, ty);
                let val = match pixel {
                    crate::types::Pixel::Black => 0u8,
                    _ => 255u8,
                };
                for dy in 0..10u32 {
                    for dx in 0..10u32 {
                        img.set(
                            ox as u32 + tx as u32 * 10 + dx,
                            oy as u32 + ty as u32 * 10 + dy,
                            val,
                        );
                    }
                }
            }
        }

        // Compute quad corners that map tag-space [-1,1] to the rendered tag location
        let tag_start_x = ox + (family.layout.border_start as f64) * scale;
        let tag_start_y = oy + (family.layout.border_start as f64) * scale;
        let tag_end_x = ox + (gs - family.layout.border_start as f64) * scale;
        let tag_end_y = oy + (gs - family.layout.border_start as f64) * scale;
        let corners = [
            [tag_start_x, tag_start_y],
            [tag_end_x, tag_start_y],
            [tag_end_x, tag_end_y],
            [tag_start_x, tag_end_y],
        ];
        let h = Homography::from_quad_corners(&corners).unwrap();

        // reversed_border=true on a normal-polarity tag should fail polarity check
        let result = decode_quad(&img, &family, &qd, &h, true, 0.0);
        assert!(result.is_none());
    }

    #[test]
    #[cfg(feature = "family-tag16h5")]
    fn decode_quad_with_sharpening() {
        // Call decode_quad with decode_sharpening > 0 to exercise the sharpening path
        let family = crate::family::tag16h5();
        let rendered = crate::render::render(&family.layout, family.codes[0]);
        let qd = QuickDecode::new(&family, 2);

        let scale = 10.0;
        let ox = 60.0;
        let oy = 60.0;
        let gs = rendered.grid_size as f64;

        let mut img = ImageU8::new(200, 200);
        for y in 0..200u32 {
            for x in 0..200u32 {
                img.set(x, y, 255);
            }
        }
        for ty in 0..rendered.grid_size {
            for tx in 0..rendered.grid_size {
                let pixel = rendered.pixel(tx, ty);
                let val = match pixel {
                    crate::types::Pixel::Black => 0u8,
                    _ => 255u8,
                };
                for dy in 0..10u32 {
                    for dx in 0..10u32 {
                        img.set(
                            ox as u32 + tx as u32 * 10 + dx,
                            oy as u32 + ty as u32 * 10 + dy,
                            val,
                        );
                    }
                }
            }
        }

        let tag_start_x = ox + (family.layout.border_start as f64) * scale;
        let tag_start_y = oy + (family.layout.border_start as f64) * scale;
        let tag_end_x = ox + (gs - family.layout.border_start as f64) * scale;
        let tag_end_y = oy + (gs - family.layout.border_start as f64) * scale;
        let corners = [
            [tag_start_x, tag_start_y],
            [tag_end_x, tag_start_y],
            [tag_end_x, tag_end_y],
            [tag_start_x, tag_end_y],
        ];
        let h = Homography::from_quad_corners(&corners).unwrap();

        // decode_sharpening = 1.0 exercises the sharpening branch
        let result = decode_quad(&img, &family, &qd, &h, false, 1.0);
        assert!(result.is_some());
        let r = result.unwrap();
        assert_eq!(r.id, 0);
    }

    #[test]
    #[cfg(feature = "family-tag16h5")]
    fn decode_quad_bit_outside_grid() {
        // Clone a family and modify a bit_location to be out-of-bounds,
        // exercising the gx >= total_width fallback (line 316).
        let mut family = crate::family::tag16h5();
        let qd = QuickDecode::new(&family, 2);
        let rendered = crate::render::render(&family.layout, family.codes[0]);

        let scale = 10.0;
        let ox = 60.0;
        let oy = 60.0;
        let gs = rendered.grid_size as f64;

        let mut img = ImageU8::new(200, 200);
        for y in 0..200u32 {
            for x in 0..200u32 {
                img.set(x, y, 255);
            }
        }
        for ty in 0..rendered.grid_size {
            for tx in 0..rendered.grid_size {
                let pixel = rendered.pixel(tx, ty);
                let val = match pixel {
                    crate::types::Pixel::Black => 0u8,
                    _ => 255u8,
                };
                for dy in 0..10u32 {
                    for dx in 0..10u32 {
                        img.set(
                            ox as u32 + tx as u32 * 10 + dx,
                            oy as u32 + ty as u32 * 10 + dy,
                            val,
                        );
                    }
                }
            }
        }

        let tag_start_x = ox + (family.layout.border_start as f64) * scale;
        let tag_start_y = oy + (family.layout.border_start as f64) * scale;
        let tag_end_x = ox + (gs - family.layout.border_start as f64) * scale;
        let tag_end_y = oy + (gs - family.layout.border_start as f64) * scale;
        let corners = [
            [tag_start_x, tag_start_y],
            [tag_end_x, tag_start_y],
            [tag_end_x, tag_end_y],
            [tag_start_x, tag_end_y],
        ];
        let h = Homography::from_quad_corners(&corners).unwrap();

        // Push one bit_location way out of bounds
        family.bit_locations[0].x = 9999;

        // Should still return a result (the out-of-bounds bit gets value 0.0)
        // but the code will differ, so decode may or may not find a match
        let _result = decode_quad(&img, &family, &qd, &h, false, 0.0);
        // We just care that it doesn't panic
    }

    #[test]
    #[cfg(feature = "family-tag16h5")]
    fn decode_quad_border_sample_out_of_bounds() {
        // Place quad corners at the very edge of a small image so that border
        // sample projections (which extend beyond the tag corners into the
        // white surround) fall outside the image, exercising the OOB `continue`.
        let family = crate::family::tag16h5();
        let qd = QuickDecode::new(&family, 2);

        // Tiny 20x20 image — corners right at the image edges so the white
        // border samples project to negative or beyond-image coordinates.
        let mut img = ImageU8::new(20, 20);
        for y in 0..20u32 {
            for x in 0..20u32 {
                img.set(x, y, 128);
            }
        }

        // Quad corners at the very image boundary
        let corners = [[0.0, 0.0], [19.0, 0.0], [19.0, 19.0], [0.0, 19.0]];
        let h = Homography::from_quad_corners(&corners).unwrap();

        // Border samples extend beyond [-1,1] tag-space. With corners at
        // pixel edges, these project outside the 20x20 image → OOB continue.
        let _result = decode_quad(&img, &family, &qd, &h, false, 0.0);
    }

    #[test]
    #[cfg(feature = "family-tag16h5")]
    fn decode_quad_normal_border_wrong_polarity() {
        // Mirror of decode_quad_reversed_border_wrong_polarity: render a
        // reversed-polarity image (swap black/white) and decode with
        // reversed_border=false. The polarity check at line 251 should reject.
        let family = crate::family::tag16h5();
        let rendered = crate::render::render(&family.layout, family.codes[0]);
        let qd = QuickDecode::new(&family, 2);

        let scale = 10.0;
        let ox = 60.0;
        let oy = 60.0;
        let gs = rendered.grid_size as f64;

        // Build image with *inverted* polarity (white→black, black→white)
        let mut img = ImageU8::new(200, 200);
        for y in 0..200u32 {
            for x in 0..200u32 {
                img.set(x, y, 0); // black background instead of white
            }
        }
        for ty in 0..rendered.grid_size {
            for tx in 0..rendered.grid_size {
                let pixel = rendered.pixel(tx, ty);
                let val = match pixel {
                    crate::types::Pixel::Black => 255u8, // inverted
                    _ => 0u8,                            // inverted
                };
                for dy in 0..10u32 {
                    for dx in 0..10u32 {
                        img.set(
                            ox as u32 + tx as u32 * 10 + dx,
                            oy as u32 + ty as u32 * 10 + dy,
                            val,
                        );
                    }
                }
            }
        }

        let tag_start_x = ox + (family.layout.border_start as f64) * scale;
        let tag_start_y = oy + (family.layout.border_start as f64) * scale;
        let tag_end_x = ox + (gs - family.layout.border_start as f64) * scale;
        let tag_end_y = oy + (gs - family.layout.border_start as f64) * scale;
        let corners = [
            [tag_start_x, tag_start_y],
            [tag_end_x, tag_start_y],
            [tag_end_x, tag_end_y],
            [tag_start_x, tag_end_y],
        ];
        let h = Homography::from_quad_corners(&corners).unwrap();

        // reversed_border=false on an inverted-polarity tag → white <= black → None
        let result = decode_quad(&img, &family, &qd, &h, false, 0.0);
        assert!(result.is_none());
    }

    #[test]
    fn gray_model_constant_field() {
        let mut gm = GrayModel::default();
        for i in 0..10 {
            for j in 0..10 {
                gm.add(i as f64, j as f64, 100.0);
            }
        }
        gm.solve();
        let v = gm.interpolate(5.0, 5.0);
        assert!((v - 100.0).abs() < 1e-6, "v={v}");
    }

    #[test]
    fn gray_model_linear_gradient() {
        let mut gm = GrayModel::default();
        for i in 0..10 {
            for j in 0..10 {
                let x = i as f64 / 10.0;
                let y = j as f64 / 10.0;
                gm.add(x, y, 50.0 * x + 30.0 * y + 10.0);
            }
        }
        gm.solve();
        let v = gm.interpolate(0.5, 0.5);
        let expected = 50.0 * 0.5 + 30.0 * 0.5 + 10.0;
        assert!((v - expected).abs() < 1e-6, "v={v}, expected={expected}");
    }

    #[test]
    #[cfg(feature = "family-tag16h5")]
    fn quick_decode_finds_exact_match() {
        let family = crate::family::tag16h5();
        let qd = QuickDecode::new(&family, 2);

        // Code 0 should match
        let result = qd.decode(&family, family.codes[0]);
        assert!(result.is_some());
        let (id, h, r) = result.unwrap();
        assert_eq!(id, 0);
        assert_eq!(h, 0);
        assert_eq!(r, 0);
    }

    #[test]
    #[cfg(feature = "family-tag16h5")]
    fn quick_decode_with_one_bit_error() {
        let family = crate::family::tag16h5();
        let qd = QuickDecode::new(&family, 2);

        // Flip one bit
        let corrupted = family.codes[0] ^ 1;
        let result = qd.decode(&family, corrupted);
        assert!(result.is_some());
        let (id, h, _) = result.unwrap();
        assert_eq!(id, 0);
        assert_eq!(h, 1);
    }

    #[test]
    #[cfg(feature = "family-tag36h11")]
    fn quick_decode_too_many_errors_returns_none() {
        let family = crate::family::tag36h11();
        let qd = QuickDecode::new(&family, 1);

        // Use a code very far from all valid codes
        let corrupted = 0xAAAAAAAAA_u64; // arbitrary pattern
        let result = qd.decode(&family, corrupted);
        // With maxhamming 1, an arbitrary code shouldn't match
        assert!(result.is_none());
    }

    #[test]
    #[cfg(feature = "family-tag16h5")]
    fn quick_decode_rotated_code() {
        let family = crate::family::tag16h5();
        let qd = QuickDecode::new(&family, 2);

        // Rotate code 0 once
        let rotated = hamming::rotate90(family.codes[0], 16);
        let result = qd.decode(&family, rotated);
        assert!(result.is_some());
        let (id, h, _r) = result.unwrap();
        assert_eq!(id, 0);
        assert_eq!(h, 0);
    }

    #[test]
    #[cfg(feature = "family-tag36h11")]
    fn quick_decode_tag36h11() {
        let family = crate::family::tag36h11();
        let qd = QuickDecode::new(&family, 2);

        // Check first and last codes
        let result = qd.decode(&family, family.codes[0]).unwrap();
        assert_eq!(result.0, 0);
        assert_eq!(result.1, 0);

        let last = family.codes.len() - 1;
        let result = qd.decode(&family, family.codes[last]).unwrap();
        assert_eq!(result.0, last as i32);
        assert_eq!(result.1, 0);
    }
}
