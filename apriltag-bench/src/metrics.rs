/// Detection quality metrics: corner accuracy, detection rate, scoring.
use apriltag::detect::detector::Detection;
use serde::{Deserialize, Serialize};

use crate::scene::PlacedTag;

/// Result of evaluating detections against ground truth for a single scene.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SceneResult {
    /// Matched ground-truth tags with their corresponding detections.
    pub matches: Vec<DetectionMatch>,
    /// Detections that don't correspond to any ground-truth tag.
    pub false_positives: Vec<DetectionSummary>,
    /// Fraction of ground-truth tags that were detected (0.0–1.0).
    pub detection_rate: f64,
    /// Root mean square of all per-corner Euclidean distances across all matches.
    pub corner_rmse: f64,
    /// Maximum per-corner error across all matches (pixels).
    pub max_corner_error: f64,
    /// Mean per-corner error across all matches (pixels).
    pub mean_corner_error: f64,
    /// Detection time in microseconds.
    pub detection_time_us: u64,
}

/// A ground-truth tag matched (or unmatched) with a detection.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DetectionMatch {
    /// The ground-truth tag.
    pub ground_truth: PlacedTag,
    /// The matched detection, if any.
    pub detection: Option<DetectionSummary>,
    /// Per-corner Euclidean distance (pixels), if matched. [TL, TR, BR, BL].
    pub corner_errors: Option<[f64; 4]>,
}

/// Serializable summary of a detection (avoids needing Detection to be Serialize).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DetectionSummary {
    pub family_name: String,
    pub id: i32,
    pub hamming: i32,
    pub decision_margin: f32,
    pub corners: [[f64; 2]; 4],
    pub center: [f64; 2],
}

impl From<&Detection> for DetectionSummary {
    fn from(det: &Detection) -> Self {
        DetectionSummary {
            family_name: det.family_name.clone(),
            id: det.id,
            hamming: det.hamming,
            decision_margin: det.decision_margin,
            corners: det.corners,
            center: det.center,
        }
    }
}

/// Evaluate detections against ground truth.
///
/// For each ground-truth tag, finds the detection with matching family+ID.
/// Corner errors account for the 4 possible rotational alignments of corner
/// ordering and pick the one with minimum RMSE.
pub fn evaluate(
    ground_truth: &[PlacedTag],
    detections: &[Detection],
    detection_time_us: u64,
) -> SceneResult {
    let mut matches = Vec::new();
    let mut used = vec![false; detections.len()];

    for gt in ground_truth {
        // Find matching detection: same family name and tag ID
        let matched = detections.iter().enumerate().find(|(i, det)| {
            !used[*i] && det.family_name == gt.family_name && det.id == gt.tag_id as i32
        });

        if let Some((idx, det)) = matched {
            used[idx] = true;
            let corner_errors = best_corner_errors(&gt.corners, &det.corners);
            matches.push(DetectionMatch {
                ground_truth: gt.clone(),
                detection: Some(det.into()),
                corner_errors: Some(corner_errors),
            });
        } else {
            matches.push(DetectionMatch {
                ground_truth: gt.clone(),
                detection: None,
                corner_errors: None,
            });
        }
    }

    // False positives: detections not matched to any ground truth
    let false_positives: Vec<DetectionSummary> = detections
        .iter()
        .enumerate()
        .filter(|(i, _)| !used[*i])
        .map(|(_, det)| det.into())
        .collect();

    // Compute aggregate metrics
    let detected_count = matches.iter().filter(|m| m.detection.is_some()).count();
    let detection_rate = if ground_truth.is_empty() {
        1.0
    } else {
        detected_count as f64 / ground_truth.len() as f64
    };

    let all_errors: Vec<f64> = matches
        .iter()
        .filter_map(|m| m.corner_errors)
        .flat_map(|e| e.into_iter())
        .collect();

    let (corner_rmse, max_corner_error, mean_corner_error) = if all_errors.is_empty() {
        (0.0, 0.0, 0.0)
    } else {
        let sum_sq: f64 = all_errors.iter().map(|e| e * e).sum();
        let rmse = (sum_sq / all_errors.len() as f64).sqrt();
        let max = all_errors.iter().cloned().fold(0.0_f64, f64::max);
        let mean: f64 = all_errors.iter().sum::<f64>() / all_errors.len() as f64;
        (rmse, max, mean)
    };

    SceneResult {
        matches,
        false_positives,
        detection_rate,
        corner_rmse,
        max_corner_error,
        mean_corner_error,
        detection_time_us,
    }
}

/// Compute per-corner Euclidean errors, trying all 4 rotational alignments.
///
/// The detector may report corners in any of 4 rotational orderings.
/// We try all 4 and pick the one with the lowest total error.
fn best_corner_errors(gt: &[[f64; 2]; 4], det: &[[f64; 2]; 4]) -> [f64; 4] {
    let mut best_errors = [f64::MAX; 4];
    let mut best_total = f64::MAX;

    for rotation in 0..4 {
        let mut errors = [0.0; 4];
        let mut total = 0.0;
        for i in 0..4 {
            let j = (i + rotation) % 4;
            let dx = gt[i][0] - det[j][0];
            let dy = gt[i][1] - det[j][1];
            let dist = (dx * dx + dy * dy).sqrt();
            errors[i] = dist;
            total += dist;
        }
        if total < best_total {
            best_total = total;
            best_errors = errors;
        }
    }

    best_errors
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_gt(family: &str, id: u32, corners: [[f64; 2]; 4]) -> PlacedTag {
        let cx = corners.iter().map(|c| c[0]).sum::<f64>() / 4.0;
        let cy = corners.iter().map(|c| c[1]).sum::<f64>() / 4.0;
        PlacedTag {
            family_name: family.to_string(),
            tag_id: id,
            corners,
            center: [cx, cy],
        }
    }

    fn make_det(family: &str, id: i32, corners: [[f64; 2]; 4]) -> Detection {
        let cx = corners.iter().map(|c| c[0]).sum::<f64>() / 4.0;
        let cy = corners.iter().map(|c| c[1]).sum::<f64>() / 4.0;
        Detection {
            family_name: family.to_string(),
            id,
            hamming: 0,
            decision_margin: 100.0,
            corners,
            center: [cx, cy],
        }
    }

    #[test]
    fn perfect_detection_zero_error() {
        let corners = [[50.0, 50.0], [150.0, 50.0], [150.0, 150.0], [50.0, 150.0]];
        let gt = vec![make_gt("tag36h11", 0, corners)];
        let dets = vec![make_det("tag36h11", 0, corners)];

        let result = evaluate(&gt, &dets, 0);

        assert_eq!(result.detection_rate, 1.0);
        assert!((result.corner_rmse).abs() < 1e-10);
        assert!((result.max_corner_error).abs() < 1e-10);
        assert!((result.mean_corner_error).abs() < 1e-10);
        assert!(result.false_positives.is_empty());
    }

    #[test]
    fn known_offset_correct_rmse() {
        let gt_corners = [[50.0, 50.0], [150.0, 50.0], [150.0, 150.0], [50.0, 150.0]];
        // Shift all corners by (1, 0) → error = 1.0 per corner
        let det_corners = [[51.0, 50.0], [151.0, 50.0], [151.0, 150.0], [51.0, 150.0]];

        let gt = vec![make_gt("tag36h11", 0, gt_corners)];
        let dets = vec![make_det("tag36h11", 0, det_corners)];

        let result = evaluate(&gt, &dets, 0);

        assert_eq!(result.detection_rate, 1.0);
        assert!((result.corner_rmse - 1.0).abs() < 1e-10);
        assert!((result.max_corner_error - 1.0).abs() < 1e-10);
        assert!((result.mean_corner_error - 1.0).abs() < 1e-10);
    }

    #[test]
    fn missed_detection() {
        let gt = vec![make_gt(
            "tag36h11",
            0,
            [[50.0, 50.0], [150.0, 50.0], [150.0, 150.0], [50.0, 150.0]],
        )];
        let dets: Vec<Detection> = vec![];

        let result = evaluate(&gt, &dets, 0);

        assert_eq!(result.detection_rate, 0.0);
        assert_eq!(result.matches.len(), 1);
        assert!(result.matches[0].detection.is_none());
    }

    #[test]
    fn false_positive() {
        let gt: Vec<PlacedTag> = vec![];
        let dets = vec![make_det(
            "tag36h11",
            5,
            [[50.0, 50.0], [150.0, 50.0], [150.0, 150.0], [50.0, 150.0]],
        )];

        let result = evaluate(&gt, &dets, 0);

        assert_eq!(result.detection_rate, 1.0); // no GT → vacuously true
        assert_eq!(result.false_positives.len(), 1);
        assert_eq!(result.false_positives[0].id, 5);
    }

    #[test]
    fn rotated_corner_ordering() {
        // GT corners: TL, TR, BR, BL
        let gt_corners = [[50.0, 50.0], [150.0, 50.0], [150.0, 150.0], [50.0, 150.0]];
        // Detection reports corners rotated by 1: TR, BR, BL, TL
        let det_corners = [[150.0, 50.0], [150.0, 150.0], [50.0, 150.0], [50.0, 50.0]];

        let gt = vec![make_gt("tag36h11", 0, gt_corners)];
        let dets = vec![make_det("tag36h11", 0, det_corners)];

        let result = evaluate(&gt, &dets, 0);

        // Should find the rotation and report zero error
        assert!((result.corner_rmse).abs() < 1e-10);
    }

    #[test]
    fn multiple_tags_partial_detection() {
        let gt = vec![
            make_gt(
                "tag36h11",
                0,
                [[50.0, 50.0], [100.0, 50.0], [100.0, 100.0], [50.0, 100.0]],
            ),
            make_gt(
                "tag36h11",
                1,
                [[200.0, 50.0], [250.0, 50.0], [250.0, 100.0], [200.0, 100.0]],
            ),
        ];
        // Only detect tag 0
        let dets = vec![make_det(
            "tag36h11",
            0,
            [[50.0, 50.0], [100.0, 50.0], [100.0, 100.0], [50.0, 100.0]],
        )];

        let result = evaluate(&gt, &dets, 1000);

        assert_eq!(result.detection_rate, 0.5);
        assert_eq!(result.detection_time_us, 1000);
        assert_eq!(result.matches.len(), 2);
        assert!(result.matches[0].detection.is_some());
        assert!(result.matches[1].detection.is_none());
    }

    #[test]
    fn different_families_not_matched() {
        let gt = vec![make_gt(
            "tag36h11",
            0,
            [[50.0, 50.0], [100.0, 50.0], [100.0, 100.0], [50.0, 100.0]],
        )];
        // Same ID but different family → should not match
        let dets = vec![make_det(
            "tag16h5",
            0,
            [[50.0, 50.0], [100.0, 50.0], [100.0, 100.0], [50.0, 100.0]],
        )];

        let result = evaluate(&gt, &dets, 0);

        assert_eq!(result.detection_rate, 0.0);
        assert_eq!(result.false_positives.len(), 1);
    }

    #[test]
    fn best_corner_errors_identity() {
        let corners = [[10.0, 10.0], [20.0, 10.0], [20.0, 20.0], [10.0, 20.0]];
        let errors = best_corner_errors(&corners, &corners);
        for e in &errors {
            assert!(e.abs() < 1e-10);
        }
    }

    #[test]
    fn best_corner_errors_diagonal_offset() {
        let gt = [[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]];
        let det = [[3.0, 4.0], [13.0, 4.0], [13.0, 14.0], [3.0, 14.0]];
        let errors = best_corner_errors(&gt, &det);
        // Each corner is offset by (3,4) → distance = 5
        for e in &errors {
            assert!((*e - 5.0).abs() < 1e-10);
        }
    }
}
