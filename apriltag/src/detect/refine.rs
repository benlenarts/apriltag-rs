use super::image::ImageU8;
use super::quad::Quad;

/// Refine quad edges by snapping to strong gradients in the original image.
///
/// For each quad edge, samples along the edge and searches perpendicular to it
/// to find the strongest gradient, then re-fits the edge line and recomputes
/// corner intersections.
pub fn refine_edges(quad: &mut Quad, img: &ImageU8, quad_decimate: f32) {
    let range = quad_decimate as f64 + 1.0;

    let mut lines = [[0.0f64; 4]; 4]; // [px, py, nx, ny]

    let steps = (2.0 * range * 4.0) as usize;
    // Precomputed interpolation values: offsets from (n_min - 1) to (n_max + 1) in 0.25 steps.
    // n ranges [-range, +range], so unique offsets span [-range-1, range+1].
    // Count: steps + 1 (for g2 values at offsets n-1) + 8 (g1 at n+1 is 8 quarter-steps ahead).
    let n_vals = steps + 9;
    let mut vals: Vec<f64> = Vec::with_capacity(n_vals);

    for edge in 0..4 {
        let a = quad.corners[edge];
        let b = quad.corners[(edge + 1) % 4];

        let dx = b[0] - a[0];
        let dy = b[1] - a[1];
        let edge_len = (dx * dx + dy * dy).sqrt();

        // Edge normal (perpendicular, pointing outward)
        let mut nx = dy / edge_len;
        let mut ny = -dx / edge_len;

        // Flip normal if reversed border
        if quad.reversed_border {
            nx = -nx;
            ny = -ny;
        }

        let nsamples = 16.max((edge_len / 8.0) as usize);

        // Check if the entire edge search region is safely inside the image,
        // so we can skip per-pixel clamping in the inner loop.
        let margin = range + 1.0;
        let use_fast = img.interpolation_safe(a[0] + margin * nx, a[1] + margin * ny)
            && img.interpolation_safe(a[0] - margin * nx, a[1] - margin * ny)
            && img.interpolation_safe(b[0] + margin * nx, b[1] + margin * ny)
            && img.interpolation_safe(b[0] - margin * nx, b[1] - margin * ny);

        let mut mx = 0.0f64;
        let mut my = 0.0f64;
        let mut mxx = 0.0f64;
        let mut mxy = 0.0f64;
        let mut myy = 0.0f64;
        let mut n_total = 0.0f64;

        for s in 0..nsamples {
            let alpha = (1.0 + s as f64) / (nsamples as f64 + 1.0);
            let x0 = alpha * b[0] + (1.0 - alpha) * a[0];
            let y0 = alpha * b[1] + (1.0 - alpha) * a[1];

            // Precompute all interpolated values along the normal.
            // The base offset starts at -range - 1.0 (the smallest offset needed for g2).
            // vals[i] = interpolation at offset (base_offset + i * 0.25) from (x0, y0).
            // Then: g2(step) = vals[step] (offset n - 1)
            //        g1(step) = vals[step + 8] (offset n + 1, which is 8 quarter-steps ahead)
            let base_offset = -range - 1.0;
            vals.clear();
            if use_fast {
                for i in 0..n_vals {
                    let offset = base_offset + i as f64 * 0.25;
                    let px = x0 + offset * nx;
                    let py = y0 + offset * ny;
                    vals.push(img.interpolate_unclamped(px, py));
                }
            } else {
                for i in 0..n_vals {
                    let offset = base_offset + i as f64 * 0.25;
                    let px = x0 + offset * nx;
                    let py = y0 + offset * ny;
                    vals.push(img.interpolate(px, py));
                }
            }

            let mut mn = 0.0f64;
            let mut mcount = 0.0f64;

            for step in 0..=steps {
                let g2 = vals[step];
                let g1 = vals[step + 8];

                if g1 < g2 {
                    continue; // backwards gradient
                }

                let weight = (g2 - g1) * (g2 - g1);
                let n = -range + step as f64 * 0.25;
                mn += weight * n;
                mcount += weight;
            }

            if mcount < 1e-10 {
                continue;
            }

            let n0 = mn / mcount;
            let bestx = x0 + n0 * nx;
            let besty = y0 + n0 * ny;

            mx += bestx;
            my += besty;
            mxx += bestx * bestx;
            mxy += bestx * besty;
            myy += besty * besty;
            n_total += 1.0;
        }

        if n_total < 2.0 {
            // Not enough samples, keep original line
            let cx = (a[0] + b[0]) / 2.0;
            let cy = (a[1] + b[1]) / 2.0;
            lines[edge] = [cx, cy, nx, ny];
            continue;
        }

        let ex = mx / n_total;
        let ey = my / n_total;
        let cxx = mxx / n_total - ex * ex;
        let cxy = mxy / n_total - ex * ey;
        let cyy = myy / n_total - ey * ey;

        let theta = 0.5 * (-2.0 * cxy).atan2(cyy - cxx);
        lines[edge] = [ex, ey, theta.cos(), theta.sin()];
    }

    // Recompute corners from refined lines
    for i in 0..4 {
        let j = (i + 1) % 4;
        if let Some((cx, cy)) = intersect_lines_raw(&lines[i], &lines[j]) {
            quad.corners[j] = [cx, cy];
        }
    }
}

/// Intersect two lines given as [px, py, nx, ny].
fn intersect_lines_raw(l0: &[f64; 4], l1: &[f64; 4]) -> Option<(f64, f64)> {
    // Direction = perpendicular to normal
    let a00 = l0[3]; // ny0 (direction x)
    let a01 = -l1[3]; // -ny1
    let a10 = -l0[2]; // -nx0 (direction y)
    let a11 = l1[2]; // nx1

    let b0 = l1[0] - l0[0];
    let b1 = l1[1] - l0[1];

    let det = a00 * a11 - a10 * a01;
    if det.abs() < 0.001 {
        return None;
    }

    let lambda = (a11 * b0 - a01 * b1) / det;
    let cx = l0[0] + lambda * a00;
    let cy = l0[1] + lambda * a10;

    Some((cx, cy))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn intersect_lines_raw_perpendicular() {
        let l0 = [5.0, 0.0, 0.0, 1.0]; // horizontal through (5,0)
        let l1 = [0.0, 3.0, 1.0, 0.0]; // vertical through (0,3)
        let (cx, cy) = intersect_lines_raw(&l0, &l1).unwrap();
        assert!((cx - 0.0).abs() < 1e-10);
        assert!((cy - 0.0).abs() < 1e-10);
    }

    #[test]
    fn intersect_lines_raw_parallel_returns_none() {
        let l0 = [0.0, 0.0, 0.0, 1.0];
        let l1 = [0.0, 5.0, 0.0, 1.0];
        assert!(intersect_lines_raw(&l0, &l1).is_none());
    }

    #[test]
    fn refine_edges_no_crash_on_uniform_image() {
        let img = ImageU8::new(100, 100);
        let mut quad = Quad {
            corners: [[20.0, 20.0], [80.0, 20.0], [80.0, 80.0], [20.0, 80.0]],
            reversed_border: false,
        };
        refine_edges(&mut quad, &img, 2.0);
        // Should not crash; corners may change slightly
        for c in &quad.corners {
            assert!(c[0].is_finite());
            assert!(c[1].is_finite());
        }
    }

    #[test]
    fn refine_edges_with_strong_edge() {
        // Create an image with a strong vertical edge at x=50
        let mut img = ImageU8::new(100, 100);
        for y in 0..100 {
            for x in 0..100 {
                img.set(x, y, if x < 50 { 0 } else { 255 });
            }
        }

        let mut quad = Quad {
            corners: [[45.0, 20.0], [55.0, 20.0], [55.0, 80.0], [45.0, 80.0]],
            reversed_border: false,
        };
        refine_edges(&mut quad, &img, 2.0);

        // Corners should still be finite
        for c in &quad.corners {
            assert!(c[0].is_finite());
            assert!(c[1].is_finite());
        }
    }

    #[test]
    fn refine_edges_corner_assignment_not_cyclically_rotated() {
        // Non-square rectangle so all 4 corners are geometrically distinguishable.
        // Black rectangle (40×60) on white background — each corner has a unique position.
        let (w, h) = (120, 120);
        let mut img = ImageU8::new(w, h);
        // Fill white
        for y in 0..h {
            for x in 0..w {
                img.set(x as u32, y as u32, 255);
            }
        }
        // Draw black rectangle from (30,20) to (90,80) — width=60, height=60
        // Make it non-square: (30,20) to (90,70) — width=60, height=50
        let (rx0, ry0, rx1, ry1) = (30, 20, 90, 70);
        for y in ry0..ry1 {
            for x in rx0..rx1 {
                img.set(x as u32, y as u32, 0);
            }
        }

        // Expected corners (CCW): bottom-left, bottom-right, top-right, top-left
        let expected: [[f64; 2]; 4] = [
            [rx0 as f64, ry1 as f64], // corner 0: bottom-left
            [rx1 as f64, ry1 as f64], // corner 1: bottom-right
            [rx1 as f64, ry0 as f64], // corner 2: top-right
            [rx0 as f64, ry0 as f64], // corner 3: top-left
        ];

        // Start with corners close to expected (as the initial quad detection would produce)
        let mut quad = Quad {
            corners: [
                [rx0 as f64 + 1.0, ry1 as f64 - 1.0],
                [rx1 as f64 - 1.0, ry1 as f64 - 1.0],
                [rx1 as f64 - 1.0, ry0 as f64 + 1.0],
                [rx0 as f64 + 1.0, ry0 as f64 + 1.0],
            ],
            reversed_border: false,
        };

        refine_edges(&mut quad, &img, 1.0);

        // Each refined corner should be closer to its own expected position
        // than to the cyclically-shifted expected position.
        for i in 0..4 {
            let correct_dist = ((quad.corners[i][0] - expected[i][0]).powi(2)
                + (quad.corners[i][1] - expected[i][1]).powi(2))
            .sqrt();
            let shifted = (i + 1) % 4; // the position it would get with the bug
            let wrong_dist = ((quad.corners[i][0] - expected[shifted][0]).powi(2)
                + (quad.corners[i][1] - expected[shifted][1]).powi(2))
            .sqrt();
            // refined corner should be closer to expected than to shifted position
            assert!(correct_dist < wrong_dist);
        }
    }

    #[test]
    fn refine_edges_near_boundary_uses_clamped_path() {
        // Quad right at the image edge forces the clamped fallback path
        let mut img = ImageU8::new(50, 50);
        for y in 0..50 {
            for x in 0..50 {
                img.set(x, y, if x < 25 { 0 } else { 255 });
            }
        }
        let mut quad = Quad {
            corners: [[0.0, 1.0], [48.0, 1.0], [48.0, 48.0], [0.0, 48.0]],
            reversed_border: false,
        };
        refine_edges(&mut quad, &img, 2.0);
        for c in &quad.corners {
            assert!(c[0].is_finite());
            assert!(c[1].is_finite());
        }
    }

    /// Golden output test: refine_edges must produce bit-identical results
    /// regardless of SIMD vs scalar code path.
    #[test]
    fn refine_edges_golden_output() {
        // Create a test image with a strong edge pattern
        let mut img = ImageU8::new(200, 200);
        for y in 0..200 {
            for x in 0..200 {
                // Diagonal gradient with a sharp edge at x=100
                let v = if x < 100 {
                    (x as f64 * 0.5 + y as f64 * 0.3) as u8
                } else {
                    200u8.saturating_add((x as u8).wrapping_mul(3))
                };
                img.set(x, y, v);
            }
        }

        // Interior quad (will use fast/SIMD path)
        let mut quad_fast = Quad {
            corners: [[90.0, 50.0], [110.0, 50.0], [110.0, 150.0], [90.0, 150.0]],
            reversed_border: false,
        };
        refine_edges(&mut quad_fast, &img, 2.0);

        // Capture golden values — these were computed by the scalar implementation.
        // If SIMD changes the result, this test fails.
        let corners = quad_fast.corners;
        for c in &corners {
            assert!(c[0].is_finite(), "corner x not finite: {c:?}");
            assert!(c[1].is_finite(), "corner y not finite: {c:?}");
        }
        // Snapshot the refined corners for regression (rounded to avoid float noise)
        let snap: Vec<[i64; 2]> = corners
            .iter()
            .map(|c| {
                [
                    (c[0] * 1000.0).round() as i64,
                    (c[1] * 1000.0).round() as i64,
                ]
            })
            .collect();
        // Golden values from scalar implementation
        assert_eq!(
            snap,
            vec![
                [90000, 50000],
                [110000, 50000],
                [110000, 150178],
                [90000, 149789]
            ],
        );
    }

    #[test]
    fn refine_edges_reversed_border() {
        let img = ImageU8::new(100, 100);
        let mut quad = Quad {
            corners: [[20.0, 20.0], [80.0, 20.0], [80.0, 80.0], [20.0, 80.0]],
            reversed_border: true,
        };
        refine_edges(&mut quad, &img, 1.0);
        for c in &quad.corners {
            assert!(c[0].is_finite());
            assert!(c[1].is_finite());
        }
    }
}
