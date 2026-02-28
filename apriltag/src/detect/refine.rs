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

            let mut mn = 0.0f64;
            let mut mcount = 0.0f64;

            let steps = (2.0 * range * 4.0) as i32;
            for step in 0..=steps {
                let n = -range + step as f64 * 0.25;

                let gx = x0 + n * nx;
                let gy = y0 + n * ny;

                // Sample gradient along the normal
                let g1 = img.interpolate(gx + nx, gy + ny);
                let g2 = img.interpolate(gx - nx, gy - ny);

                if g1 < g2 {
                    continue; // backwards gradient
                }

                let weight = (g2 - g1) * (g2 - g1);
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
            quad.corners[i] = [cx, cy];
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
