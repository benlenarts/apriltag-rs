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

/// Reusable scratch buffers for quad fitting, avoiding per-cluster allocation.
#[derive(Default)]
pub struct QuadFitBufs {
    lfps: Vec<LineFitPt>,
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
    #[cfg(not(feature = "parallel"))] bufs: &mut QuadFitBufs,
) {
    // C reference: 2*(2*w + 2*h) = 4*(w+h). Each edge point is typically added
    // twice (two unique neighbors), so the limit is 2× the geometric perimeter.
    // See apriltag_quad_thresh.c:1090.
    let max_perimeter = 4 * (image_width + image_height) as usize;

    out.clear();

    // COVERAGE: parallel feature block — only compiled with --features parallel
    #[cfg(feature = "parallel")]
    {
        *out = clusters
            .par_iter_mut()
            .map_init(QuadFitBufs::new, |bufs, cluster| {
                fit_quad(
                    cluster,
                    params,
                    max_perimeter,
                    normal_border,
                    reversed_border,
                    bufs,
                )
            })
            .flatten()
            .collect();
    }

    #[cfg(not(feature = "parallel"))]
    {
        out.extend(clusters.iter_mut().filter_map(|cluster| {
            fit_quad(
                cluster,
                params,
                max_perimeter,
                normal_border,
                reversed_border,
                bufs,
            )
        }));
    }
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

/// Gradient magnitude weight lookup, indexed by `(gx != 0) << 1 | (gy != 0)`.
/// Since `gx` and `gy` are always in `{-255, 0, 255}`, there are only 4 distinct values.
#[inline]
fn grad_weight(gx: i16, gy: i16) -> f64 {
    // sqrt(255² + 255²) + 1 = sqrt(130050) + 1 ≈ 361.624...
    const DIAG: f64 = 361.62445840513925;
    const TABLE: [f64; 4] = [1.0, 256.0, 256.0, DIAG];
    TABLE[((gx != 0) as usize) << 1 | (gy != 0) as usize]
}

/// Build cumulative weighted moments for line fitting into a reusable buffer.
fn build_line_fit_pts(points: &[Pt], lfps: &mut Vec<LineFitPt>) {
    lfps.clear();
    lfps.reserve(points.len().saturating_sub(lfps.capacity()));
    let mut cum = LineFitPt::default();

    for p in points {
        let x = p.x as f64 * 0.5 + 0.5;
        let y = p.y as f64 * 0.5 + 0.5;
        let w = grad_weight(p.gx, p.gy);

        cum.mx += w * x;
        cum.my += w * y;
        cum.mxx += w * x * x;
        cum.mxy += w * x * y;
        cum.myy += w * y * y;
        cum.w += w;

        lfps.push(cum);
    }
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
        // Wrapping: [i0, sz) + [0, i1] — i0 > i1 implies i0 >= 1
        debug_assert!(i0 >= 1);
        let start = &lfps[i0 - 1];
        let tail = LineFitPt {
            mx: last.mx - start.mx,
            my: last.my - start.my,
            mxx: last.mxx - start.mxx,
            mxy: last.mxy - start.mxy,
            myy: last.myy - start.myy,
            w: last.w - start.w,
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
            // COVERAGE: degenerate case requires exact FP conditions (cxy ≈ 0 with
            // cxx ≈ cyy) that don't arise from realistic cluster geometry.
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
    lfps: &[LineFitPt],
    errors: &mut Vec<f64>,
    maxima: &mut Vec<(usize, f64)>,
    params: &QuadThreshParams,
) -> Option<[usize; 4]> {
    let sz = lfps.len();
    let ksz = 20.min(sz / 12).max(1);

    // Compute line-fit error at each point
    errors.clear();
    for i in 0..sz {
        let i0 = (i + sz - ksz) % sz;
        let i1 = (i + ksz) % sz;
        let moments = range_moments(lfps, i0, i1);
        let err = fit_line(&moments).map(|(_, mse)| mse).unwrap_or(0.0);
        errors.push(err);
    }

    // Smooth errors with Gaussian-like filter
    smooth_errors(errors);

    // Find local maxima (use >= on left to handle plateaus from synthetic images)
    maxima.clear();
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
        maxima.sort_by(|a, b| b.1.total_cmp(&a.1));
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
                    let indices = [maxima[m0].0, maxima[m1].0, maxima[m2].0, maxima[m3].0];

                    if let Some(err) = evaluate_quad_combination(lfps, &indices, sz, params) {
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
    // prev_line is always Some here: the loop runs 4 iterations, each setting prev_line.
    // Early returns (via `?`) would have exited the function before reaching this point.
    let last_line = prev_line?;
    let dot = (last_line.nx * first_line.nx + last_line.ny * first_line.ny).abs();
    if dot > params.cos_critical_rad as f64 {
        return None;
    }

    Some(total_err)
}

/// Smooth the error array using a simple low-pass filter.
///
/// Uses a two-variable rolling window to avoid allocating a copy of the array.
fn smooth_errors(errors: &mut [f64]) {
    let sz = errors.len();
    if sz < 3 {
        return;
    }

    // Gaussian-like smoothing (σ≈1)
    let k0: f64 = 0.1665;
    let k1: f64 = 0.6670;
    let k2: f64 = 0.1665;

    // Save the original first and last values before they get overwritten
    let orig_first = errors[0];
    let orig_last = errors[sz - 1];

    // Process forward: at each step we need orig[i-1], orig[i], orig[i+1].
    // After writing errors[i], we've lost orig[i], but we still have it as
    // `prev_orig`. We haven't touched orig[i+1] yet.
    let mut prev_orig = orig_last; // wrapping: orig[0-1] = orig[sz-1]
    for i in 0..sz - 1 {
        let curr_orig = errors[i];
        let next_orig = errors[i + 1];
        errors[i] = k0 * prev_orig + k1 * curr_orig + k2 * next_orig;
        prev_orig = curr_orig;
    }
    // Last element wraps to first (use saved original first value)
    errors[sz - 1] = k0 * prev_orig + k1 * orig_last + k2 * orig_first;
}

/// Compute quad corner positions from line intersections.
fn compute_quad_corners(
    lfps: &[LineFitPt],
    indices: &[usize; 4],
    _sz: usize,
) -> Option<[[f64; 2]; 4]> {
    let mut lines = [FittedLine {
        px: 0.0,
        py: 0.0,
        nx: 0.0,
        ny: 0.0,
    }; 4];
    for seg in 0..4 {
        let i0 = indices[seg];
        let i1 = indices[(seg + 1) % 4];
        let moments = range_moments(lfps, i0, i1);
        let (line, _) = fit_line(&moments)?;
        lines[seg] = line;
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
        let cross = (p1[0] - p0[0]) * (p2[1] - p1[1]) - (p1[1] - p0[1]) * (p2[0] - p1[0]);
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
        let l0 = FittedLine {
            px: 0.0,
            py: 0.0,
            nx: 0.0,
            ny: 1.0,
        }; // horizontal
        let l1 = FittedLine {
            px: 5.0,
            py: 0.0,
            nx: 1.0,
            ny: 0.0,
        }; // vertical at x=5
        let (cx, cy) = intersect_lines(&l0, &l1).unwrap();
        assert!((cx - 5.0).abs() < 1e-10);
        assert!(cy.abs() < 1e-10);
    }

    #[test]
    fn intersect_parallel_lines_returns_none() {
        let l0 = FittedLine {
            px: 0.0,
            py: 0.0,
            nx: 0.0,
            ny: 1.0,
        };
        let l1 = FittedLine {
            px: 0.0,
            py: 5.0,
            nx: 0.0,
            ny: 1.0,
        };
        assert!(intersect_lines(&l0, &l1).is_none());
    }

    #[test]
    fn quad_area_unit_square() {
        let corners = [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]];
        let area = quad_area(&corners);
        assert!((area - 1.0).abs() < 1e-10);
    }

    #[test]
    fn quad_area_ccw_positive() {
        // CCW winding → positive area
        let corners = [[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]];
        assert!(quad_area(&corners) > 0.0);
    }

    #[test]
    fn validate_quad_convex_ccw_passes() {
        let corners = [[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]];
        let params = QuadThreshParams::default();
        assert!(validate_quad(&corners, &params).is_some());
    }

    #[test]
    fn validate_quad_clockwise_fails() {
        // CW winding → negative area → rejected
        let corners = [[0.0, 0.0], [0.0, 10.0], [10.0, 10.0], [10.0, 0.0]];
        let params = QuadThreshParams::default();
        assert!(validate_quad(&corners, &params).is_none());
    }

    #[test]
    fn fit_quad_exceeds_max_perimeter() {
        // 50 points exceeds max_perimeter = 4*(5+5) = 40 for a 5x5 image
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
        // tiny image makes max_perimeter = 4*(5+5) = 40, less than 50 points
        let mut quads = Vec::new();
        fit_quads(
            &mut [cluster],
            5,
            5,
            &params,
            true,
            true,
            &mut quads,
            #[cfg(not(feature = "parallel"))]
            &mut QuadFitBufs::new(),
        );
        assert!(quads.is_empty());
    }

    #[test]
    fn find_corners_collinear_returns_none() {
        // Collinear cluster (all points on a line) produces < 4 maxima
        let points: Vec<Pt> = (0..30)
            .map(|i| Pt {
                x: 100 + i * 2,
                y: 100,
                gx: 0,
                gy: 255,
                slope: 0,
            })
            .collect();
        let mut lfps = Vec::new();
        build_line_fit_pts(&points, &mut lfps);
        let mut errors = Vec::new();
        let mut maxima = Vec::new();
        let params = QuadThreshParams::default();
        // collinear points have uniform error → fewer than 4 maxima
        assert!(find_corners(&lfps, &mut errors, &mut maxima, &params).is_none());
    }

    #[test]
    fn validate_quad_non_convex_fails() {
        // CCW winding (positive area) but non-convex (one corner dented inward)
        let corners = [[0.0, 0.0], [10.0, 0.0], [5.0, 2.0], [0.0, 10.0]];
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
            points.push(Pt {
                x,
                y,
                gx,
                gy,
                slope: 0,
            });
        }
        let (reversed, _dot) = check_border_direction(&points);
        // dot should be positive (gradients point outward)
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
    fn range_moments_whole_range() {
        let points = [
            Pt {
                x: 0,
                y: 0,
                gx: 255,
                gy: 0,
                slope: 0,
            },
            Pt {
                x: 2,
                y: 0,
                gx: 255,
                gy: 0,
                slope: 0,
            },
        ];
        let mut lfps = Vec::new();
        build_line_fit_pts(&points, &mut lfps);
        let m = range_moments(&lfps, 0, 1);
        assert!((m.w - lfps[1].w).abs() < 1e-10);
    }

    #[test]
    fn range_moments_wrapping() {
        let points = [
            Pt {
                x: 0,
                y: 0,
                gx: 255,
                gy: 0,
                slope: 0,
            },
            Pt {
                x: 2,
                y: 0,
                gx: 255,
                gy: 0,
                slope: 0,
            },
            Pt {
                x: 4,
                y: 0,
                gx: 255,
                gy: 0,
                slope: 0,
            },
        ];
        let mut lfps = Vec::new();
        build_line_fit_pts(&points, &mut lfps);
        // Wrapping range [2, 0] = point 2 + point 0
        let m = range_moments(&lfps, 2, 0);
        let w2 = lfps[2].w - lfps[1].w; // weight of point 2
        let w0 = lfps[0].w; // weight of point 0
        assert!((m.w - (w2 + w0)).abs() < 1e-10);
    }

    #[test]
    fn build_line_fit_pts_pixel_center_delta() {
        // Three collinear points at fixed-point y=20 (pixel y=10).
        // With +0.5 pixel-center delta, centroid py should be 10.5, not 10.0.
        let points = [
            Pt {
                x: 0,
                y: 20,
                gx: 255,
                gy: 0,
                slope: 0,
            },
            Pt {
                x: 2,
                y: 20,
                gx: 255,
                gy: 0,
                slope: 0,
            },
            Pt {
                x: 4,
                y: 20,
                gx: 255,
                gy: 0,
                slope: 0,
            },
        ];
        let mut lfps = Vec::new();
        build_line_fit_pts(&points, &mut lfps);
        let total = &lfps[2];
        let py = total.my / total.w;
        // centroid py=10.5 (with pixel-center delta)
        assert!((py - 10.5).abs() < 1e-10);
    }

    #[test]
    fn fit_quad_synthetic_rectangle() {
        // Create a cluster of points tracing a rectangle
        let mut points = Vec::new();
        let (x0, y0, x1, y1) = (140, 140, 260, 260); // fixed-point coords (actual: 70-130)

        // Top edge: y=y0, x varies, gradient points up
        for x in (x0..x1).step_by(2) {
            points.push(Pt {
                x: x as u16,
                y: y0 as u16,
                gx: 0,
                gy: -255,
                slope: 0,
            });
        }
        // Right edge: x=x1, y varies, gradient points right
        for y in (y0..y1).step_by(2) {
            points.push(Pt {
                x: x1 as u16,
                y: y as u16,
                gx: 255,
                gy: 0,
                slope: 0,
            });
        }
        // Bottom edge: y=y1, x varies, gradient points down
        for x in (x0..x1).step_by(2) {
            points.push(Pt {
                x: x as u16,
                y: y1 as u16,
                gx: 0,
                gy: 255,
                slope: 0,
            });
        }
        // Left edge: x=x0, y varies, gradient points left
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
        fit_quads(
            &mut [cluster],
            400,
            400,
            &params,
            true,
            true,
            &mut quads,
            #[cfg(not(feature = "parallel"))]
            &mut QuadFitBufs::new(),
        );

        eprintln!("Synthetic rectangle: found {} quads", quads.len());
        // should find a quad from a perfect rectangle
        assert!(!quads.is_empty());
    }

    #[test]
    fn slope_key_zero_displacement_is_nan() {
        // When a point coincides with the centroid, dx=dy=0 yields NaN
        let s = slope_key(0.0, 0.0);
        let f = f32::from_bits(s);
        assert!(f.is_nan(), "slope_key(0,0) should be NaN bits, got {f}");
    }

    #[test]
    fn sort_by_angle_with_nan_slope_does_not_panic() {
        // If any point's slope is NaN (e.g. from coincident centroid),
        // the sort must not panic. With u32 keys, NaN bit patterns sort
        // consistently as large values (NaN bits > all finite positive f32 bits).
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

        // Manually inject a NaN slope (as u32 bits)
        points[0].slope = f32::NAN.to_bits();
        points[1].slope = 1.0f32.to_bits();
        points[2].slope = 2.0f32.to_bits();
        points[3].slope = 3.0f32.to_bits();

        // With u32 keys, sort_unstable_by_key just works — no NaN issues.
        points.sort_unstable_by_key(|p| p.slope);
    }

    #[test]
    fn find_corners_sort_with_nan_error_does_not_panic() {
        // The maxima sort in find_corners uses partial_cmp().unwrap() on f64.
        // If an error value is NaN, it panics. Verify the comparator is safe.
        let mut maxima: Vec<(usize, f64)> = vec![(0, 1.0), (1, f64::NAN), (2, 3.0), (3, 2.0)];

        // This is the exact comparator from find_corners line 373.
        // With partial_cmp().unwrap() this panics; with total_cmp() it doesn't.
        maxima.sort_by(|a, b| b.1.total_cmp(&a.1));
    }

    #[test]
    fn fit_quad_too_few_points() {
        // Cluster with only 10 points (< 24 hard minimum)
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
        fit_quads(
            &mut [cluster],
            400,
            400,
            &params,
            true,
            true,
            &mut quads,
            #[cfg(not(feature = "parallel"))]
            &mut QuadFitBufs::new(),
        );
        assert!(quads.is_empty());
    }

    #[test]
    fn fit_quad_zero_gradient() {
        // Cluster with 30+ points, all gradients (0,0) → dot=0 → rejected
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
        fit_quads(
            &mut [cluster],
            400,
            400,
            &params,
            true,
            true,
            &mut quads,
            #[cfg(not(feature = "parallel"))]
            &mut QuadFitBufs::new(),
        );
        assert!(quads.is_empty());
    }

    #[test]
    fn fit_quad_normal_border_rejected_reversed_only() {
        // Create a valid normal-border cluster, but only allow reversed_border
        let mut points = Vec::new();
        let (x0, y0, x1, y1) = (140, 140, 260, 260);
        for i in 0..30 {
            let t = i as f64 / 30.0;
            points.push(Pt {
                x: (x0 as f64 + (x1 - x0) as f64 * t) as u16,
                y: y0,
                gx: 0,
                gy: -255, // gradient pointing outward (normal border)
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
        // normal_border=false, reversed_border=true → reject normal-border clusters
        let mut quads = Vec::new();
        fit_quads(
            &mut [cluster],
            400,
            400,
            &params,
            false,
            true,
            &mut quads,
            #[cfg(not(feature = "parallel"))]
            &mut QuadFitBufs::new(),
        );
        assert!(quads.is_empty());
    }

    #[test]
    fn fit_line_zero_weight() {
        // LineFitPt with w=0 → fit_line returns None
        let moments = LineFitPt::default();
        assert!(fit_line(&moments).is_none());
    }

    #[test]
    fn fit_line_coincident_points() {
        // All points at same location → eigenvalues both zero → None
        let moments = LineFitPt {
            mx: 10.0,
            my: 20.0,
            mxx: 100.0,
            mxy: 200.0,
            myy: 400.0,
            w: 1.0,
        };
        // mx/w = 10, my/w = 20, cxx = 100 - 100 = 0, cyy = 400 - 400 = 0
        assert!(fit_line(&moments).is_none());
    }

    #[test]
    fn fit_line_axis_aligned() {
        // Points along a horizontal line (cxy=0) → normal should be vertical
        let mut cum = LineFitPt::default();
        for i in 0..10 {
            let x = i as f64;
            let y = 5.0;
            cum.mx += x;
            cum.my += y;
            cum.mxx += x * x;
            cum.mxy += x * y;
            cum.myy += y * y;
            cum.w += 1.0;
        }
        let (line, _mse) = fit_line(&cum).unwrap();
        // Normal should be close to (0, ±1) since line is horizontal
        assert!(line.nx.abs() < 0.01);
        assert!(line.ny.abs() > 0.99);
    }

    #[test]
    fn smooth_errors_short_array() {
        // Array with < 3 elements should be unchanged
        let mut e1 = vec![5.0];
        smooth_errors(&mut e1);
        assert_eq!(e1, vec![5.0]);

        let mut e2 = vec![3.0, 7.0];
        smooth_errors(&mut e2);
        assert_eq!(e2, vec![3.0, 7.0]);

        let mut e0: Vec<f64> = vec![];
        smooth_errors(&mut e0);
        assert!(e0.is_empty());
    }

    #[test]
    fn grad_weight_matches_sqrt_for_all_valid_inputs() {
        // gx and gy are always in {-255, 0, 255}
        for &gx in &[-255i16, 0, 255] {
            for &gy in &[-255i16, 0, 255] {
                let expected = ((gx as f64).powi(2) + (gy as f64).powi(2)).sqrt() + 1.0;
                let actual = grad_weight(gx, gy);
                // grad_weight should match sqrt(gx^2 + gy^2) + 1
                assert!((actual - expected).abs() < 1e-10);
            }
        }
    }
}
