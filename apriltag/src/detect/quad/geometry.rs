use super::line_fitting::{fit_line, range_moments, FittedLine, LineFitPt};
use super::QuadThreshParams;
use crate::detect::geometry::Vec2;

/// Compute quad corner positions from line intersections.
pub(super) fn compute_quad_corners(
    lfps: &[LineFitPt],
    indices: &[usize; 4],
    _sz: usize,
) -> Option<[Vec2; 4]> {
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

    let mut corners = [Vec2::new(0.0, 0.0); 4];
    for i in 0..4 {
        let j = (i + 1) % 4;
        let (cx, cy) = intersect_lines(&lines[i], &lines[j])?;
        corners[i] = Vec2::new(cx, cy);
    }

    Some(corners)
}

/// Compute intersection of two fitted lines.
pub(super) fn intersect_lines(l0: &FittedLine, l1: &FittedLine) -> Option<(f64, f64)> {
    let a00 = l0.ny;
    let a01 = -l1.ny;
    let a10 = -l0.nx;
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
pub(super) fn validate_quad(corners: &[Vec2; 4], _params: &QuadThreshParams) -> Option<()> {
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
pub(super) fn quad_area(corners: &[Vec2; 4]) -> f64 {
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
    fn intersect_perpendicular_lines() {
        let l0 = FittedLine {
            px: 0.0,
            py: 0.0,
            nx: 0.0,
            ny: 1.0,
        };
        let l1 = FittedLine {
            px: 5.0,
            py: 0.0,
            nx: 1.0,
            ny: 0.0,
        };
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

    fn v(corners: [[f64; 2]; 4]) -> [Vec2; 4] {
        corners.map(Vec2::from)
    }

    #[test]
    fn quad_area_unit_square() {
        let corners = v([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]);
        let area = quad_area(&corners);
        assert!((area - 1.0).abs() < 1e-10);
    }

    #[test]
    fn quad_area_ccw_positive() {
        let corners = v([[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]]);
        assert!(quad_area(&corners) > 0.0);
    }

    #[test]
    fn validate_quad_convex_ccw_passes() {
        let corners = v([[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]]);
        let params = QuadThreshParams::default();
        assert!(validate_quad(&corners, &params).is_some());
    }

    #[test]
    fn validate_quad_clockwise_fails() {
        let corners = v([[0.0, 0.0], [0.0, 10.0], [10.0, 10.0], [10.0, 0.0]]);
        let params = QuadThreshParams::default();
        assert!(validate_quad(&corners, &params).is_none());
    }

    #[test]
    fn validate_quad_non_convex_fails() {
        let corners = v([[0.0, 0.0], [10.0, 0.0], [5.0, 2.0], [0.0, 10.0]]);
        let params = QuadThreshParams::default();
        assert!(validate_quad(&corners, &params).is_none());
    }
}
