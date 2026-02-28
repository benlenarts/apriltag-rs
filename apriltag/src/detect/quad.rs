use super::cluster::{Cluster, Pt};

#[cfg(feature = "parallel")]
use rayon::iter::{IntoParallelRefMutIterator, ParallelIterator};

/// A detected quadrilateral with four corners in pixel coordinates.
#[derive(Debug, Clone)]
pub struct Quad {
    /// Four corner positions in pixel coords (counter-clockwise winding).
    pub corners: [[f64; 2]; 4],
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

/// Cumulative weighted moments for efficient range line fitting.
#[derive(Debug, Clone, Copy, Default)]
struct LineFitPt {
    mx: f64,
    my: f64,
    mxx: f64,
    mxy: f64,
    myy: f64,
    w: f64,
}

/// A fitted line parameterized by a point (px, py) and unit normal (nx, ny).
#[derive(Debug, Clone, Copy)]
struct FittedLine {
    px: f64,
    py: f64,
    nx: f64,
    ny: f64,
}

/// Fit quads from a list of clusters.
pub fn fit_quads(
    clusters: &mut [Cluster],
    image_width: u32,
    image_height: u32,
    params: &QuadThreshParams,
    normal_border: bool,
    reversed_border: bool,
) -> Vec<Quad> {
    let max_perimeter = 2 * (image_width + image_height) as usize;

    #[cfg(feature = "parallel")]
    {
        clusters
            .par_iter_mut()
            .filter_map(|cluster| {
                fit_quad(
                    cluster,
                    params,
                    max_perimeter,
                    normal_border,
                    reversed_border,
                )
            })
            .collect()
    }

    #[cfg(not(feature = "parallel"))]
    {
        clusters
            .iter_mut()
            .filter_map(|cluster| {
                fit_quad(
                    cluster,
                    params,
                    max_perimeter,
                    normal_border,
                    reversed_border,
                )
            })
            .collect()
    }
}

/// Try to fit a single quad from a cluster of edge points.
fn fit_quad(
    cluster: &mut Cluster,
    params: &QuadThreshParams,
    max_perimeter: usize,
    normal_border: bool,
    reversed_border: bool,
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
    let lfps = build_line_fit_pts(&cluster.points);

    // Corner detection
    let corners_idx = find_corners(&cluster.points, &lfps, params)?;

    // Fit lines through each segment and compute corners
    let quad_corners = compute_quad_corners(&lfps, &corners_idx, sz)?;

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
    let xmin = points.iter().map(|p| p.x).min().unwrap() as f64;
    let xmax = points.iter().map(|p| p.x).max().unwrap() as f64;
    let ymin = points.iter().map(|p| p.y).min().unwrap() as f64;
    let ymax = points.iter().map(|p| p.y).max().unwrap() as f64;

    let cx = (xmin + xmax) / 2.0 + 0.05118;
    let cy = (ymin + ymax) / 2.0 - 0.028581;

    for p in points.iter_mut() {
        let dx = p.x as f64 - cx;
        let dy = p.y as f64 - cy;
        p.slope = slope_proxy(dx, dy);
    }

    points.sort_by(|a, b| a.slope.partial_cmp(&b.slope).unwrap());
}

/// Fast slope proxy that maps an angle to a monotonic value in [0, 4).
fn slope_proxy(dx: f64, dy: f64) -> f32 {
    let adx = dx.abs();
    let ady = dy.abs();

    if dy > 0.0 {
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
    }
}

/// Build cumulative weighted moments for line fitting.
fn build_line_fit_pts(points: &[Pt]) -> Vec<LineFitPt> {
    let mut lfps = Vec::with_capacity(points.len());
    let mut cum = LineFitPt::default();

    for p in points {
        let x = p.x as f64 / 2.0;
        let y = p.y as f64 / 2.0;
        let w = ((p.gx as f64).powi(2) + (p.gy as f64).powi(2)).sqrt() + 1.0;

        cum.mx += w * x;
        cum.my += w * y;
        cum.mxx += w * x * x;
        cum.mxy += w * x * y;
        cum.myy += w * y * y;
        cum.w += w;

        lfps.push(cum);
    }
    lfps
}

/// Compute line fit moments for a range [i0, i1] (inclusive, wrapping).
fn range_moments(lfps: &[LineFitPt], i0: usize, i1: usize) -> LineFitPt {
    let sz = lfps.len();
    let last = &lfps[sz - 1];

    if i0 <= i1 {
        let end = &lfps[i1];
        if i0 == 0 {
            *end
        } else {
            let start = &lfps[i0 - 1];
            LineFitPt {
                mx: end.mx - start.mx,
                my: end.my - start.my,
                mxx: end.mxx - start.mxx,
                mxy: end.mxy - start.mxy,
                myy: end.myy - start.myy,
                w: end.w - start.w,
            }
        }
    } else {
        // Wrapping: [i0, sz) + [0, i1]
        let tail = if i0 == 0 {
            *last
        } else {
            let start = &lfps[i0 - 1];
            LineFitPt {
                mx: last.mx - start.mx,
                my: last.my - start.my,
                mxx: last.mxx - start.mxx,
                mxy: last.mxy - start.mxy,
                myy: last.myy - start.myy,
                w: last.w - start.w,
            }
        };
        let head = &lfps[i1];
        LineFitPt {
            mx: tail.mx + head.mx,
            my: tail.my + head.my,
            mxx: tail.mxx + head.mxx,
            mxy: tail.mxy + head.mxy,
            myy: tail.myy + head.myy,
            w: tail.w + head.w,
        }
    }
}

/// Fit a line from cumulative moments and return (line, mse).
fn fit_line(moments: &LineFitPt) -> Option<(FittedLine, f64)> {
    if moments.w < 1e-10 {
        return None;
    }

    let ex = moments.mx / moments.w;
    let ey = moments.my / moments.w;
    let cxx = moments.mxx / moments.w - ex * ex;
    let cxy = moments.mxy / moments.w - ex * ey;
    let cyy = moments.myy / moments.w - ey * ey;

    let disc = ((cxx - cyy).powi(2) + 4.0 * cxy * cxy).sqrt();
    let eig_small = 0.5 * (cxx + cyy - disc);
    let eig_large = 0.5 * (cxx + cyy + disc);

    if eig_large < 1e-10 {
        return None;
    }

    // Normal direction = eigenvector of smaller eigenvalue
    let (nx, ny) = {
        let nx0 = cxy;
        let ny0 = eig_small - cxx;
        let len0 = (nx0 * nx0 + ny0 * ny0).sqrt();
        if len0 > 1e-10 {
            (nx0, ny0)
        } else {
            // Degenerate case (cxy ≈ 0): eigenvectors are axis-aligned
            if cxx > cyy {
                (0.0, 1.0) // line is horizontal, normal is vertical
            } else {
                (1.0, 0.0) // line is vertical, normal is horizontal
            }
        }
    };
    let len = (nx * nx + ny * ny).sqrt();

    Some((
        FittedLine {
            px: ex,
            py: ey,
            nx: nx / len,
            ny: ny / len,
        },
        eig_small.max(0.0),
    ))
}

/// Find 4 corner indices that partition the sorted points into quad segments.
fn find_corners(
    points: &[Pt],
    lfps: &[LineFitPt],
    params: &QuadThreshParams,
) -> Option<[usize; 4]> {
    let sz = points.len();
    let ksz = 20.min(sz / 12).max(1);

    // Compute line-fit error at each point
    let mut errors: Vec<f64> = Vec::with_capacity(sz);
    for i in 0..sz {
        let i0 = (i + sz - ksz) % sz;
        let i1 = (i + ksz) % sz;
        let moments = range_moments(lfps, i0, i1);
        let err = fit_line(&moments).map(|(_, mse)| mse).unwrap_or(0.0);
        errors.push(err);
    }

    // Smooth errors with Gaussian-like filter
    smooth_errors(&mut errors);

    // Find local maxima (use >= on left to handle plateaus from synthetic images)
    let mut maxima: Vec<(usize, f64)> = Vec::new();
    for i in 0..sz {
        let prev = errors[(i + sz - 1) % sz];
        let next = errors[(i + 1) % sz];
        if errors[i] >= prev && errors[i] > next {
            maxima.push((i, errors[i]));
        }
    }

    if maxima.len() < 4 {
        return None;
    }

    // Keep top max_nmaxima by error magnitude
    if maxima.len() > params.max_nmaxima as usize {
        maxima.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());
        maxima.truncate(params.max_nmaxima as usize);
        maxima.sort_by_key(|&(idx, _)| idx);
    }

    // Exhaustive search for best 4-combination
    let nm = maxima.len();
    let mut best_err = f64::MAX;
    let mut best_corners: Option<[usize; 4]> = None;

    for m0 in 0..nm {
        for m1 in (m0 + 1)..nm {
            for m2 in (m1 + 1)..nm {
                for m3 in (m2 + 1)..nm {
                    let indices = [
                        maxima[m0].0,
                        maxima[m1].0,
                        maxima[m2].0,
                        maxima[m3].0,
                    ];

                    if let Some(err) = evaluate_quad_combination(
                        lfps,
                        &indices,
                        sz,
                        params,
                    ) {
                        if err < best_err {
                            best_err = err;
                            best_corners = Some(indices);
                        }
                    }
                }
            }
        }
    }

    best_corners
}

/// Evaluate the total error for a 4-corner combination.
fn evaluate_quad_combination(
    lfps: &[LineFitPt],
    indices: &[usize; 4],
    _sz: usize,
    params: &QuadThreshParams,
) -> Option<f64> {
    let mut total_err = 0.0;
    let mut prev_line: Option<FittedLine> = None;

    for seg in 0..4 {
        let i0 = indices[seg];
        let i1 = indices[(seg + 1) % 4];
        let moments = range_moments(lfps, i0, i1);
        let (line, mse) = fit_line(&moments)?;

        if mse > params.max_line_fit_mse as f64 {
            return None;
        }

        // Check angle between adjacent lines
        if let Some(prev) = prev_line {
            let dot = (prev.nx * line.nx + prev.ny * line.ny).abs();
            if dot > params.cos_critical_rad as f64 {
                return None;
            }
        }

        total_err += mse;
        prev_line = Some(line);
    }

    // Check angle between last and first line
    let first_moments = range_moments(lfps, indices[0], indices[1]);
    let (first_line, _) = fit_line(&first_moments)?;
    let last_line = prev_line.unwrap();
    let dot = (last_line.nx * first_line.nx + last_line.ny * first_line.ny).abs();
    if dot > params.cos_critical_rad as f64 {
        return None;
    }

    Some(total_err)
}

/// Smooth the error array using a simple low-pass filter.
fn smooth_errors(errors: &mut [f64]) {
    let sz = errors.len();
    if sz < 3 {
        return;
    }

    // Gaussian-like smoothing (σ≈1)
    let kernel = [0.1665, 0.6670, 0.1665];
    let orig = errors.to_vec();

    for i in 0..sz {
        let prev = orig[(i + sz - 1) % sz];
        let curr = orig[i];
        let next = orig[(i + 1) % sz];
        errors[i] = kernel[0] * prev + kernel[1] * curr + kernel[2] * next;
    }
}

/// Compute quad corner positions from line intersections.
fn compute_quad_corners(
    lfps: &[LineFitPt],
    indices: &[usize; 4],
    _sz: usize,
) -> Option<[[f64; 2]; 4]> {
    let mut lines = Vec::with_capacity(4);
    for seg in 0..4 {
        let i0 = indices[seg];
        let i1 = indices[(seg + 1) % 4];
        let moments = range_moments(lfps, i0, i1);
        let (line, _) = fit_line(&moments)?;
        lines.push(line);
    }

    let mut corners = [[0.0f64; 2]; 4];
    for i in 0..4 {
        let j = (i + 1) % 4;
        let (cx, cy) = intersect_lines(&lines[i], &lines[j])?;
        corners[i] = [cx, cy];
    }

    Some(corners)
}

/// Compute intersection of two fitted lines.
fn intersect_lines(l0: &FittedLine, l1: &FittedLine) -> Option<(f64, f64)> {
    // Line direction = perpendicular to normal
    let a00 = l0.ny; // dy0
    let a01 = -l1.ny; // -dy1
    let a10 = -l0.nx; // dx0 (negated normal x gives direction y... let me use the spec)
    let a11 = l1.nx;

    let b0 = l1.px - l0.px;
    let b1 = l1.py - l0.py;

    let det = a00 * a11 - a10 * a01;
    if det.abs() < 0.001 {
        return None;
    }

    let lambda = (a11 * b0 - a01 * b1) / det;
    let cx = l0.px + lambda * a00;
    let cy = l0.py + lambda * a10;

    Some((cx, cy))
}

/// Validate that the quad has correct geometry.
fn validate_quad(corners: &[[f64; 2]; 4], _params: &QuadThreshParams) -> Option<()> {
    // Check area (> 0 for valid winding)
    let area = quad_area(corners);
    if area < 0.0 {
        return None;
    }

    // Check convexity via cross products
    for i in 0..4 {
        let p0 = corners[i];
        let p1 = corners[(i + 1) % 4];
        let p2 = corners[(i + 2) % 4];
        let cross = (p1[0] - p0[0]) * (p2[1] - p1[1])
            - (p1[1] - p0[1]) * (p2[0] - p1[0]);
        if cross < 0.0 {
            return None; // Non-convex
        }
    }

    Some(())
}

/// Compute area of a quad using the shoelace formula.
fn quad_area(corners: &[[f64; 2]; 4]) -> f64 {
    let mut area = 0.0;
    for i in 0..4 {
        let j = (i + 1) % 4;
        area += corners[i][0] * corners[j][1];
        area -= corners[j][0] * corners[i][1];
    }
    area / 2.0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn slope_proxy_monotonic_around_circle() {
        let n = 100;
        let mut prev = -1.0f32;
        for i in 0..n {
            let angle = 2.0 * std::f64::consts::PI * i as f64 / n as f64 + 0.01;
            let dx = angle.cos();
            let dy = angle.sin();
            let s = slope_proxy(dx, dy);
            assert!(s >= 0.0 && s < 4.0, "slope out of range: {s}");
            if i > 0 {
                assert!(s > prev, "not monotonic at i={i}: {prev} -> {s}");
            }
            prev = s;
        }
    }

    #[test]
    fn fit_line_collinear_points() {
        // Points along y=2x
        let mut cum = LineFitPt::default();
        for i in 0..10 {
            let x = i as f64;
            let y = 2.0 * x;
            let w = 1.0;
            cum.mx += w * x;
            cum.my += w * y;
            cum.mxx += w * x * x;
            cum.mxy += w * x * y;
            cum.myy += w * y * y;
            cum.w += w;
        }
        let (line, mse) = fit_line(&cum).unwrap();
        assert!(mse < 1e-10, "MSE should be ~0: {mse}");
        // Normal should be perpendicular to (1, 2), i.e. proportional to (-2, 1) or (2, -1)
        let dot = line.nx * 1.0 + line.ny * 2.0;
        assert!(dot.abs() < 1e-10, "normal not perpendicular: {dot}");
    }

    #[test]
    fn intersect_perpendicular_lines() {
        let l0 = FittedLine { px: 0.0, py: 0.0, nx: 0.0, ny: 1.0 }; // horizontal
        let l1 = FittedLine { px: 5.0, py: 0.0, nx: 1.0, ny: 0.0 }; // vertical at x=5
        let (cx, cy) = intersect_lines(&l0, &l1).unwrap();
        assert!((cx - 5.0).abs() < 1e-10);
        assert!(cy.abs() < 1e-10);
    }

    #[test]
    fn intersect_parallel_lines_returns_none() {
        let l0 = FittedLine { px: 0.0, py: 0.0, nx: 0.0, ny: 1.0 };
        let l1 = FittedLine { px: 0.0, py: 5.0, nx: 0.0, ny: 1.0 };
        assert!(intersect_lines(&l0, &l1).is_none());
    }

    #[test]
    fn quad_area_unit_square() {
        let corners = [
            [0.0, 0.0],
            [1.0, 0.0],
            [1.0, 1.0],
            [0.0, 1.0],
        ];
        let area = quad_area(&corners);
        assert!((area - 1.0).abs() < 1e-10);
    }

    #[test]
    fn quad_area_ccw_positive() {
        // CCW winding → positive area
        let corners = [
            [0.0, 0.0],
            [10.0, 0.0],
            [10.0, 10.0],
            [0.0, 10.0],
        ];
        assert!(quad_area(&corners) > 0.0);
    }

    #[test]
    fn validate_quad_convex_ccw_passes() {
        let corners = [
            [0.0, 0.0],
            [10.0, 0.0],
            [10.0, 10.0],
            [0.0, 10.0],
        ];
        let params = QuadThreshParams::default();
        assert!(validate_quad(&corners, &params).is_some());
    }

    #[test]
    fn validate_quad_clockwise_fails() {
        // CW winding → negative area → rejected
        let corners = [
            [0.0, 0.0],
            [0.0, 10.0],
            [10.0, 10.0],
            [10.0, 0.0],
        ];
        let params = QuadThreshParams::default();
        assert!(validate_quad(&corners, &params).is_none());
    }

    #[test]
    fn smooth_errors_reduces_spike() {
        let mut errors = vec![0.0, 0.0, 100.0, 0.0, 0.0];
        smooth_errors(&mut errors);
        assert!(errors[2] < 100.0);
        assert!(errors[1] > 0.0);
    }

    #[test]
    fn check_border_direction_normal() {
        // Points arranged in a circle with gradients pointing outward
        let mut points = Vec::new();
        for i in 0..8 {
            let angle = 2.0 * std::f64::consts::PI * i as f64 / 8.0;
            let r = 100.0;
            let x = (r * angle.cos() + 200.0) as u16;
            let y = (r * angle.sin() + 200.0) as u16;
            let gx = (255.0 * angle.cos()) as i16;
            let gy = (255.0 * angle.sin()) as i16;
            points.push(Pt { x, y, gx, gy, slope: 0.0 });
        }
        let (reversed, dot) = check_border_direction(&points);
        assert!(!reversed, "dot={dot}, should be positive (gradients point out)");
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
    fn range_moments_whole_range() {
        let points = [
            Pt { x: 0, y: 0, gx: 255, gy: 0, slope: 0.0 },
            Pt { x: 2, y: 0, gx: 255, gy: 0, slope: 0.0 },
        ];
        let lfps = build_line_fit_pts(&points);
        let m = range_moments(&lfps, 0, 1);
        assert!((m.w - lfps[1].w).abs() < 1e-10);
    }

    #[test]
    fn range_moments_wrapping() {
        let points = [
            Pt { x: 0, y: 0, gx: 255, gy: 0, slope: 0.0 },
            Pt { x: 2, y: 0, gx: 255, gy: 0, slope: 0.0 },
            Pt { x: 4, y: 0, gx: 255, gy: 0, slope: 0.0 },
        ];
        let lfps = build_line_fit_pts(&points);
        // Wrapping range [2, 0] = point 2 + point 0
        let m = range_moments(&lfps, 2, 0);
        let w2 = lfps[2].w - lfps[1].w; // weight of point 2
        let w0 = lfps[0].w; // weight of point 0
        assert!((m.w - (w2 + w0)).abs() < 1e-10);
    }

    #[test]
    fn fit_quad_synthetic_rectangle() {
        // Create a cluster of points tracing a rectangle
        let mut points = Vec::new();
        let (x0, y0, x1, y1) = (140, 140, 260, 260); // fixed-point coords (actual: 70-130)

        // Top edge: y=y0, x varies, gradient points up
        for x in (x0..x1).step_by(2) {
            points.push(Pt { x: x as u16, y: y0 as u16, gx: 0, gy: -255, slope: 0.0 });
        }
        // Right edge: x=x1, y varies, gradient points right
        for y in (y0..y1).step_by(2) {
            points.push(Pt { x: x1 as u16, y: y as u16, gx: 255, gy: 0, slope: 0.0 });
        }
        // Bottom edge: y=y1, x varies, gradient points down
        for x in (x0..x1).step_by(2) {
            points.push(Pt { x: x as u16, y: y1 as u16, gx: 0, gy: 255, slope: 0.0 });
        }
        // Left edge: x=x0, y varies, gradient points left
        for y in (y0..y1).step_by(2) {
            points.push(Pt { x: x0 as u16, y: y as u16, gx: -255, gy: 0, slope: 0.0 });
        }

        let cluster = Cluster { points };
        let params = QuadThreshParams::default();

        let quads = fit_quads(
            &mut [cluster],
            400,
            400,
            &params,
            true,
            true,
        );

        eprintln!("Synthetic rectangle: found {} quads", quads.len());
        assert!(!quads.is_empty(), "Should find a quad from a perfect rectangle");
    }
}
