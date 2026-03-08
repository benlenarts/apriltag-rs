use super::super::cluster::Pt;

/// Cumulative weighted moments for efficient range line fitting.
#[derive(Debug, Clone, Copy, Default)]
pub(super) struct LineFitPt {
    pub mx: f64,
    pub my: f64,
    pub mxx: f64,
    pub mxy: f64,
    pub myy: f64,
    pub w: f64,
}

/// A fitted line parameterized by a point (px, py) and unit normal (nx, ny).
#[derive(Debug, Clone, Copy)]
pub(super) struct FittedLine {
    pub px: f64,
    pub py: f64,
    pub nx: f64,
    pub ny: f64,
}

/// Gradient magnitude weight lookup, indexed by `(gx != 0) << 1 | (gy != 0)`.
/// Since `gx` and `gy` are always in `{-255, 0, 255}`, there are only 4 distinct values.
#[inline]
pub(super) fn grad_weight(gx: i16, gy: i16) -> f64 {
    // sqrt(255² + 255²) + 1 = sqrt(130050) + 1 ≈ 361.624...
    const DIAG: f64 = 361.62445840513925;
    const TABLE: [f64; 4] = [1.0, 256.0, 256.0, DIAG];
    TABLE[((gx != 0) as usize) << 1 | (gy != 0) as usize]
}

/// Build cumulative weighted moments for line fitting into a reusable buffer.
pub(super) fn build_line_fit_pts(points: &[Pt], lfps: &mut Vec<LineFitPt>) {
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
pub(super) fn range_moments(lfps: &[LineFitPt], i0: usize, i1: usize) -> LineFitPt {
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
pub(super) fn fit_line(moments: &LineFitPt) -> Option<(FittedLine, f64)> {
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

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use super::*;

    #[test]
    fn fit_line_collinear_points() {
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
        let dot = line.nx * 1.0 + line.ny * 2.0;
        assert!(dot.abs() < 1e-10, "normal not perpendicular: {dot}");
    }

    #[test]
    fn fit_line_zero_weight() {
        let moments = LineFitPt::default();
        assert!(fit_line(&moments).is_none());
    }

    #[test]
    fn fit_line_coincident_points() {
        let moments = LineFitPt {
            mx: 10.0,
            my: 20.0,
            mxx: 100.0,
            mxy: 200.0,
            myy: 400.0,
            w: 1.0,
        };
        assert!(fit_line(&moments).is_none());
    }

    #[test]
    fn fit_line_axis_aligned() {
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
        assert!(line.nx.abs() < 0.01);
        assert!(line.ny.abs() > 0.99);
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
        let m = range_moments(&lfps, 2, 0);
        let w2 = lfps[2].w - lfps[1].w;
        let w0 = lfps[0].w;
        assert!((m.w - (w2 + w0)).abs() < 1e-10);
    }

    #[test]
    fn build_line_fit_pts_pixel_center_delta() {
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
        assert!((py - 10.5).abs() < 1e-10);
    }

    #[test]
    fn grad_weight_matches_sqrt_for_all_valid_inputs() {
        for &gx in &[-255i16, 0, 255] {
            for &gy in &[-255i16, 0, 255] {
                let expected = ((gx as f64).powi(2) + (gy as f64).powi(2)).sqrt() + 1.0;
                let actual = grad_weight(gx, gy);
                assert!((actual - expected).abs() < 1e-10);
            }
        }
    }
}
