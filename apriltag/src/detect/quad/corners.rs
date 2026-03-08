use super::line_fitting::{fit_line, range_moments, LineFitPt};
use super::QuadThreshParams;

/// Find 4 corner indices that partition the sorted points into quad segments.
pub(super) fn find_corners(
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
    let mut prev_line: Option<super::line_fitting::FittedLine> = None;

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
pub(super) fn smooth_errors(errors: &mut [f64]) {
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

#[cfg(test)]
mod tests {
    use super::super::line_fitting::build_line_fit_pts;
    use super::*;
    use crate::detect::cluster::Pt;

    #[test]
    fn find_corners_collinear_returns_none() {
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
        assert!(find_corners(&lfps, &mut errors, &mut maxima, &params).is_none());
    }

    #[test]
    fn find_corners_sort_with_nan_error_does_not_panic() {
        let mut maxima: Vec<(usize, f64)> = vec![(0, 1.0), (1, f64::NAN), (2, 3.0), (3, 2.0)];
        maxima.sort_by(|a, b| b.1.total_cmp(&a.1));
    }

    #[test]
    fn smooth_errors_reduces_spike() {
        let mut errors = vec![0.0, 0.0, 100.0, 0.0, 0.0];
        smooth_errors(&mut errors);
        assert!(errors[2] < 100.0);
        assert!(errors[1] > 0.0);
    }

    #[test]
    fn smooth_errors_short_array() {
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
}
