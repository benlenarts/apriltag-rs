use super::detector::Detection;

/// Remove duplicate detections of the same tag, keeping the best one.
///
/// Two detections are considered duplicates if they have the same family and ID
/// and their quad polygons overlap (separating axis theorem).
pub fn deduplicate(detections: &mut Vec<Detection>) {
    let mut i = 0;
    while i < detections.len() {
        let mut j = i + 1;
        while j < detections.len() {
            if detections[i].family_name == detections[j].family_name
                && detections[i].id == detections[j].id
                && polygons_overlap(&detections[i].corners, &detections[j].corners)
            {
                // Keep the better one
                let keep_j = is_better(&detections[j], &detections[i]);
                if keep_j {
                    detections.swap(i, j);
                }
                detections.swap_remove(j);
                // Don't increment j, check the swapped-in element
            } else {
                j += 1;
            }
        }
        i += 1;
    }
}

/// Return true if `a` is a better detection than `b`.
fn is_better(a: &Detection, b: &Detection) -> bool {
    if a.hamming != b.hamming {
        return a.hamming < b.hamming;
    }
    if (a.decision_margin - b.decision_margin).abs() > 1e-6 {
        return a.decision_margin > b.decision_margin;
    }
    // Deterministic tiebreaker: lexicographic comparison of corners
    for i in 0..4 {
        for j in 0..2 {
            if (a.corners[i][j] - b.corners[i][j]).abs() > 1e-10 {
                return a.corners[i][j] < b.corners[i][j];
            }
        }
    }
    false
}

/// Test if two convex quadrilaterals overlap using the separating axis theorem.
fn polygons_overlap(p: &[[f64; 2]; 4], q: &[[f64; 2]; 4]) -> bool {
    // Check all 8 potential separating axes (4 edge normals per polygon)
    for poly in [p, q] {
        for i in 0..4 {
            let j = (i + 1) % 4;
            let edge_x = poly[j][0] - poly[i][0];
            let edge_y = poly[j][1] - poly[i][1];

            // Normal to the edge
            let nx = -edge_y;
            let ny = edge_x;

            // Project both polygons onto this axis
            let (p_min, p_max) = project_polygon(p, nx, ny);
            let (q_min, q_max) = project_polygon(q, nx, ny);

            // Check for separation
            if p_max < q_min || q_max < p_min {
                return false;
            }
        }
    }
    true
}

/// Project a polygon onto an axis and return (min, max) projections.
fn project_polygon(poly: &[[f64; 2]; 4], nx: f64, ny: f64) -> (f64, f64) {
    let mut min = f64::MAX;
    let mut max = f64::MIN;
    for pt in poly {
        let d = pt[0] * nx + pt[1] * ny;
        min = min.min(d);
        max = max.max(d);
    }
    (min, max)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_detection(id: i32, hamming: i32, margin: f32, corners: [[f64; 2]; 4]) -> Detection {
        Detection {
            family_name: "test".to_string(),
            id,
            hamming,
            decision_margin: margin,
            corners,
            center: [0.0, 0.0],
        }
    }

    #[test]
    fn polygons_overlap_identical() {
        let p = [[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]];
        assert!(polygons_overlap(&p, &p));
    }

    #[test]
    fn polygons_overlap_separated() {
        let p = [[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]];
        let q = [[20.0, 0.0], [30.0, 0.0], [30.0, 10.0], [20.0, 10.0]];
        assert!(!polygons_overlap(&p, &q));
    }

    #[test]
    fn polygons_overlap_partial() {
        let p = [[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]];
        let q = [[5.0, 5.0], [15.0, 5.0], [15.0, 15.0], [5.0, 15.0]];
        assert!(polygons_overlap(&p, &q));
    }

    #[test]
    fn dedup_removes_worse_duplicate() {
        let corners = [[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]];
        let mut dets = vec![
            make_detection(0, 2, 50.0, corners), // worse (higher hamming)
            make_detection(0, 0, 50.0, corners), // better (lower hamming)
        ];
        deduplicate(&mut dets);
        assert_eq!(dets.len(), 1);
        assert_eq!(dets[0].hamming, 0);
    }

    #[test]
    fn dedup_keeps_different_ids() {
        let corners = [[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]];
        let mut dets = vec![
            make_detection(0, 0, 50.0, corners),
            make_detection(1, 0, 50.0, corners),
        ];
        deduplicate(&mut dets);
        assert_eq!(dets.len(), 2);
    }

    #[test]
    fn dedup_keeps_non_overlapping() {
        let c1 = [[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]];
        let c2 = [[20.0, 20.0], [30.0, 20.0], [30.0, 30.0], [20.0, 30.0]];
        let mut dets = vec![
            make_detection(0, 0, 50.0, c1),
            make_detection(0, 0, 50.0, c2),
        ];
        deduplicate(&mut dets);
        assert_eq!(dets.len(), 2);
    }

    #[test]
    fn dedup_prefers_higher_margin_on_tie() {
        let corners = [[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]];
        let mut dets = vec![
            make_detection(0, 0, 30.0, corners),
            make_detection(0, 0, 50.0, corners),
        ];
        deduplicate(&mut dets);
        assert_eq!(dets.len(), 1);
        assert!((dets[0].decision_margin - 50.0).abs() < 1e-6);
    }

    #[test]
    fn dedup_lexicographic_tiebreaker() {
        // Same hamming, same margin → falls through to corner comparison
        let c1 = [[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]];
        let c2 = [[1.0, 0.0], [11.0, 0.0], [11.0, 10.0], [1.0, 10.0]];
        // c1 has smaller first corner x → c1 is "better"
        assert!(is_better(
            &make_detection(0, 0, 50.0, c1),
            &make_detection(0, 0, 50.0, c2),
        ));
        assert!(!is_better(
            &make_detection(0, 0, 50.0, c2),
            &make_detection(0, 0, 50.0, c1),
        ));
    }

    #[test]
    fn dedup_equal_detections_not_better() {
        let c = [[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]];
        // Identical detections: is_better returns false
        assert!(!is_better(
            &make_detection(0, 0, 50.0, c),
            &make_detection(0, 0, 50.0, c),
        ));
    }
}
