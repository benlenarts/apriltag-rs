mod corners;
mod geometry;
mod line_fitting;

use corners::find_corners;
use geometry::{compute_quad_corners, validate_quad};
use line_fitting::build_line_fit_pts;

use super::cluster::{Cluster, Pt};
use super::geometry::Vec2;
use super::par::Par;

/// A detected quadrilateral with four corners in pixel coordinates.
#[derive(Debug, Clone)]
pub struct Quad {
    /// Four corner positions in pixel coords (counter-clockwise winding).
    pub corners: [Vec2; 4],
    /// Whether the black border is inside the white border (reversed).
    pub reversed_border: bool,
}

/// Quad detection parameters.
#[derive(Debug, Clone)]
pub struct QuadThreshParams {
    pub min_cluster_pixels: i32,
    pub max_nmaxima: i32,
    pub cos_critical_rad: f32,
    pub max_line_fit_mse: f32,
    pub min_white_black_diff: i32,
    pub deglitch: bool,
}

impl Default for QuadThreshParams {
    fn default() -> Self {
        Self {
            min_cluster_pixels: 5,
            max_nmaxima: 10,
            cos_critical_rad: (10.0f32.to_radians()).cos(),
            max_line_fit_mse: 10.0,
            min_white_black_diff: 5,
            deglitch: false,
        }
    }
}

/// Reusable scratch buffers for quad fitting, avoiding per-cluster allocation.
#[derive(Default)]
pub struct QuadFitBufs {
    lfps: Vec<line_fitting::LineFitPt>,
    errors: Vec<f64>,
    maxima: Vec<(usize, f64)>,
}

impl QuadFitBufs {
    pub fn new() -> Self {
        Self::default()
    }
}

/// Fit quads from a list of clusters.
pub fn fit_quads(
    clusters: &mut [Cluster],
    image_width: u32,
    image_height: u32,
    params: &QuadThreshParams,
    normal_border: bool,
    reversed_border: bool,
    out: &mut Vec<Quad>,
) {
    // C reference: 2*(2*w + 2*h) = 4*(w+h). Each edge point is typically added
    // twice (two unique neighbors), so the limit is 2× the geometric perimeter.
    // See apriltag_quad_thresh.c:1090.
    let max_perimeter = 4 * (image_width + image_height) as usize;

    *out = Par::get().map_init_collect(clusters, QuadFitBufs::new, |bufs, cluster| {
        fit_quad(
            cluster,
            params,
            max_perimeter,
            normal_border,
            reversed_border,
            bufs,
        )
    });
}

/// Try to fit a single quad from a cluster of edge points.
fn fit_quad(
    cluster: &mut Cluster,
    params: &QuadThreshParams,
    max_perimeter: usize,
    normal_border: bool,
    reversed_border: bool,
    bufs: &mut QuadFitBufs,
) -> Option<Quad> {
    let sz = cluster.points.len();

    // Size filtering
    if (sz as i32) < params.min_cluster_pixels || sz < 24 {
        return None;
    }
    if sz > max_perimeter {
        return None;
    }

    // Border direction check
    let (is_reversed, dot) = check_border_direction(&cluster.points);
    if dot.abs() < f64::EPSILON {
        return None;
    }
    if is_reversed && !reversed_border {
        return None;
    }
    if !is_reversed && !normal_border {
        return None;
    }

    // Angular sorting
    sort_by_angle(&mut cluster.points);

    // Build cumulative moments
    build_line_fit_pts(&cluster.points, &mut bufs.lfps);

    // Corner detection
    let corners_idx = find_corners(&bufs.lfps, &mut bufs.errors, &mut bufs.maxima, params)?;

    // Fit lines through each segment and compute corners
    let quad_corners = compute_quad_corners(&bufs.lfps, &corners_idx, sz)?;

    // Validate quad
    validate_quad(&quad_corners, params)?;

    Some(Quad {
        corners: quad_corners,
        reversed_border: is_reversed,
    })
}

/// Compute the dot product of each point's position (relative to centroid) with
/// its gradient direction to determine border orientation.
fn check_border_direction(points: &[Pt]) -> (bool, f64) {
    let n = points.len() as f64;
    let cx: f64 = points.iter().map(|p| p.x as f64).sum::<f64>() / n;
    let cy: f64 = points.iter().map(|p| p.y as f64).sum::<f64>() / n;

    let dot: f64 = points
        .iter()
        .map(|p| {
            let dx = p.x as f64 - cx;
            let dy = p.y as f64 - cy;
            dx * p.gx as f64 + dy * p.gy as f64
        })
        .sum();

    (dot < 0.0, dot)
}

/// Sort points by angle around the cluster centroid using a fast slope proxy.
fn sort_by_angle(points: &mut [Pt]) {
    let (Some(xmin), Some(xmax), Some(ymin), Some(ymax)) = (
        points.iter().map(|p| p.x).min(),
        points.iter().map(|p| p.x).max(),
        points.iter().map(|p| p.y).min(),
        points.iter().map(|p| p.y).max(),
    ) else {
        // COVERAGE: empty points slice — callers always filter empty clusters
        return;
    };
    let (xmin, xmax, ymin, ymax) = (xmin as f64, xmax as f64, ymin as f64, ymax as f64);

    let cx = (xmin + xmax) / 2.0 + 0.05118;
    let cy = (ymin + ymax) / 2.0 - 0.028581;

    for p in points.iter_mut() {
        let dx = p.x as f64 - cx;
        let dy = p.y as f64 - cy;
        p.slope = slope_key(dx, dy);
    }

    points.sort_unstable_by_key(|p| p.slope);
}

/// Fast slope key that maps an angle to a monotonically increasing `u32`.
///
/// Returns the bit pattern of a non-negative `f32` in `[0, 4)`. Since IEEE 754
/// bit patterns of non-negative floats preserve ordering, the resulting `u32`
/// can be compared directly as an integer sort key — avoiding the overhead of
/// `f32::total_cmp` during sorting.
fn slope_key(dx: f64, dy: f64) -> u32 {
    let adx = dx.abs();
    let ady = dy.abs();

    let value = if dy > 0.0 {
        if dx > 0.0 {
            // Quadrant 0: [0, 1)
            (ady / (ady + adx)) as f32
        } else {
            // Quadrant 1: [1, 2)
            (1.0 + adx / (ady + adx)) as f32
        }
    } else if dx < 0.0 {
        // Quadrant 2: [2, 3)
        (2.0 + ady / (ady + adx)) as f32
    } else {
        // Quadrant 3: [3, 4)
        (3.0 + adx / (ady + adx)) as f32
    };
    value.to_bits()
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use super::*;

    #[test]
    fn slope_key_monotonic_around_circle() {
        let n = 100;
        let mut prev = 0u32;
        for i in 0..n {
            let angle = 2.0 * std::f64::consts::PI * i as f64 / n as f64 + 0.01;
            let dx = angle.cos();
            let dy = angle.sin();
            let s = slope_key(dx, dy);
            let f = f32::from_bits(s);
            assert!(f >= 0.0 && f < 4.0, "slope out of range: {f}");
            if i > 0 {
                assert!(s > prev, "not monotonic at i={i}: {prev} -> {s}");
            }
            prev = s;
        }
    }

    #[test]
    fn fit_quad_exceeds_max_perimeter() {
        let points: Vec<Pt> = (0..50)
            .map(|i| {
                let angle = 2.0 * std::f64::consts::PI * i as f64 / 50.0;
                Pt {
                    x: (100.0 + 50.0 * angle.cos()) as u16,
                    y: (100.0 + 50.0 * angle.sin()) as u16,
                    gx: (255.0 * angle.cos()) as i16,
                    gy: (255.0 * angle.sin()) as i16,
                    slope: 0,
                }
            })
            .collect();
        let cluster = Cluster { points };
        let params = QuadThreshParams::default();
        let mut quads = Vec::new();
        fit_quads(&mut [cluster], 5, 5, &params, true, true, &mut quads);
        assert!(quads.is_empty());
    }

    #[test]
    fn check_border_direction_normal() {
        let mut points = Vec::new();
        for i in 0..8 {
            let angle = 2.0 * std::f64::consts::PI * i as f64 / 8.0;
            let r = 100.0;
            let x = (r * angle.cos() + 200.0) as u16;
            let y = (r * angle.sin() + 200.0) as u16;
            let gx = (255.0 * angle.cos()) as i16;
            let gy = (255.0 * angle.sin()) as i16;
            points.push(Pt {
                x,
                y,
                gx,
                gy,
                slope: 0,
            });
        }
        let (reversed, _dot) = check_border_direction(&points);
        assert!(!reversed);
    }

    #[test]
    fn default_params_reasonable() {
        let p = QuadThreshParams::default();
        assert_eq!(p.min_cluster_pixels, 5);
        assert_eq!(p.max_nmaxima, 10);
        assert!(p.cos_critical_rad > 0.98);
        assert!((p.max_line_fit_mse - 10.0).abs() < 1e-6);
    }

    #[test]
    fn fit_quad_synthetic_rectangle() {
        let mut points = Vec::new();
        let (x0, y0, x1, y1) = (140, 140, 260, 260);

        for x in (x0..x1).step_by(2) {
            points.push(Pt {
                x: x as u16,
                y: y0 as u16,
                gx: 0,
                gy: -255,
                slope: 0,
            });
        }
        for y in (y0..y1).step_by(2) {
            points.push(Pt {
                x: x1 as u16,
                y: y as u16,
                gx: 255,
                gy: 0,
                slope: 0,
            });
        }
        for x in (x0..x1).step_by(2) {
            points.push(Pt {
                x: x as u16,
                y: y1 as u16,
                gx: 0,
                gy: 255,
                slope: 0,
            });
        }
        for y in (y0..y1).step_by(2) {
            points.push(Pt {
                x: x0 as u16,
                y: y as u16,
                gx: -255,
                gy: 0,
                slope: 0,
            });
        }

        let cluster = Cluster { points };
        let params = QuadThreshParams::default();

        let mut quads = Vec::new();
        fit_quads(&mut [cluster], 400, 400, &params, true, true, &mut quads);

        assert!(!quads.is_empty());
    }

    #[test]
    fn slope_key_zero_displacement_is_nan() {
        let s = slope_key(0.0, 0.0);
        let f = f32::from_bits(s);
        assert!(f.is_nan(), "slope_key(0,0) should be NaN bits, got {f}");
    }

    #[test]
    fn sort_by_angle_with_nan_slope_does_not_panic() {
        let mut points: Vec<Pt> = (0..4)
            .map(|i| {
                let angle = std::f64::consts::FRAC_PI_2 * i as f64;
                Pt {
                    x: (100.0 + 40.0 * angle.cos()) as u16,
                    y: (100.0 + 40.0 * angle.sin()) as u16,
                    gx: 0,
                    gy: 0,
                    slope: 0,
                }
            })
            .collect();

        points[0].slope = f32::NAN.to_bits();
        points[1].slope = 1.0f32.to_bits();
        points[2].slope = 2.0f32.to_bits();
        points[3].slope = 3.0f32.to_bits();

        points.sort_unstable_by_key(|p| p.slope);
    }

    #[test]
    fn fit_quad_too_few_points() {
        let points: Vec<Pt> = (0..10)
            .map(|i| Pt {
                x: i * 10,
                y: i * 10,
                gx: 255,
                gy: 0,
                slope: 0,
            })
            .collect();
        let cluster = Cluster { points };
        let params = QuadThreshParams::default();
        let mut quads = Vec::new();
        fit_quads(&mut [cluster], 400, 400, &params, true, true, &mut quads);
        assert!(quads.is_empty());
    }

    #[test]
    fn fit_quad_zero_gradient() {
        let points: Vec<Pt> = (0..30)
            .map(|i| {
                let angle = 2.0 * std::f64::consts::PI * i as f64 / 30.0;
                let r = 100.0;
                Pt {
                    x: (r * angle.cos() + 200.0) as u16,
                    y: (r * angle.sin() + 200.0) as u16,
                    gx: 0,
                    gy: 0,
                    slope: 0,
                }
            })
            .collect();
        let cluster = Cluster { points };
        let params = QuadThreshParams::default();
        let mut quads = Vec::new();
        fit_quads(&mut [cluster], 400, 400, &params, true, true, &mut quads);
        assert!(quads.is_empty());
    }

    #[test]
    fn fit_quad_normal_border_rejected_reversed_only() {
        let mut points = Vec::new();
        let (x0, y0, x1, y1) = (140, 140, 260, 260);
        for i in 0..30 {
            let t = i as f64 / 30.0;
            points.push(Pt {
                x: (x0 as f64 + (x1 - x0) as f64 * t) as u16,
                y: y0,
                gx: 0,
                gy: -255,
                slope: 0,
            });
        }
        for i in 0..30 {
            let t = i as f64 / 30.0;
            points.push(Pt {
                x: x1,
                y: (y0 as f64 + (y1 - y0) as f64 * t) as u16,
                gx: 255,
                gy: 0,
                slope: 0,
            });
        }
        for i in 0..30 {
            let t = i as f64 / 30.0;
            points.push(Pt {
                x: (x1 as f64 - (x1 - x0) as f64 * t) as u16,
                y: y1,
                gx: 0,
                gy: 255,
                slope: 0,
            });
        }
        for i in 0..30 {
            let t = i as f64 / 30.0;
            points.push(Pt {
                x: x0,
                y: (y1 as f64 - (y1 - y0) as f64 * t) as u16,
                gx: -255,
                gy: 0,
                slope: 0,
            });
        }
        let cluster = Cluster { points };
        let params = QuadThreshParams::default();
        let mut quads = Vec::new();
        fit_quads(&mut [cluster], 400, 400, &params, false, true, &mut quads);
        assert!(quads.is_empty());
    }
}
