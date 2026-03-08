mod svd;

use svd::project_to_so3;

use super::detector::Detection;
use super::geometry::{Mat3, Vec3};
use super::homography::Homography;

/// A 3D pose estimate (rotation + translation).
#[derive(Debug, Clone)]
pub struct Pose {
    /// 3x3 rotation matrix (row-major): camera <- tag
    pub r: [[f64; 3]; 3],
    /// Translation vector: camera <- tag
    pub t: [f64; 3],
}

/// Camera intrinsics and tag geometry for pose estimation.
#[derive(Debug, Clone)]
pub struct PoseParams {
    pub tagsize: f64,
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
}

// ── Pose estimation ──

/// Extract initial R, t from the detection homography.
fn homography_to_pose(h: &Homography, params: &PoseParams) -> Pose {
    let fx = params.fx;
    let fy = params.fy;
    let cx = params.cx;
    let cy = params.cy;

    // K^{-1} * H columns
    let hd = &h.data.0;

    let mut c0 = Vec3::new(
        (hd[0][0] - cx * hd[2][0]) / fx,
        (hd[1][0] - cy * hd[2][0]) / fy,
        hd[2][0],
    );
    let mut c1 = Vec3::new(
        (hd[0][1] - cx * hd[2][1]) / fx,
        (hd[1][1] - cy * hd[2][1]) / fy,
        hd[2][1],
    );
    let mut c2 = Vec3::new(
        (hd[0][2] - cx * hd[2][2]) / fx,
        (hd[1][2] - cy * hd[2][2]) / fy,
        hd[2][2],
    );

    // Normalize scale
    let scale = (c0.norm() + c1.norm()) / 2.0;
    c0 = c0 / scale;
    c1 = c1 / scale;
    c2 = c2 / scale;

    // Negate c1: tag parameter ty maps to -s*ty in 3D tag frame
    // so column 1 of K^{-1}*H has an embedded sign flip
    let r0 = c0;
    let r1 = -c1;
    let r2 = r0.cross(r1);

    // Assemble R (columns are r0, r1, r2) and project onto SO(3)
    let r_raw = Mat3([
        [r0[0], r1[0], r2[0]],
        [r0[1], r1[1], r2[1]],
        [r0[2], r1[2], r2[2]],
    ]);
    let r = project_to_so3(&r_raw);

    // Scale translation by tagsize/2
    let t = c2 * (params.tagsize / 2.0);

    Pose { r: r.0, t: t.0 }
}

/// Estimate the pose of a detected tag.
///
/// Returns `(best_pose, best_error, alt_pose, alt_error)`.
/// `alt_pose` is `None` when no second local minimum exists.
#[allow(clippy::needless_range_loop)]
pub fn estimate_tag_pose(det: &Detection, params: &PoseParams) -> (Pose, f64, Option<Pose>, f64) {
    // Build homography from detection corners
    let h = match Homography::from_quad_corners(&det.corners) {
        Some(h) => h,
        None => {
            return (
                Pose {
                    r: Mat3::IDENTITY.0,
                    t: [0.0, 0.0, 1.0],
                },
                f64::MAX,
                None,
                f64::MAX,
            );
        }
    };

    // Object points in tag frame (z=0 plane)
    let s = params.tagsize / 2.0;
    let tag_pts: [Vec3; 4] = [
        Vec3::new(-s, s, 0.0),
        Vec3::new(s, s, 0.0),
        Vec3::new(s, -s, 0.0),
        Vec3::new(-s, -s, 0.0),
    ];

    // Image rays (normalized coordinates)
    let mut v = [Vec3::new(0.0, 0.0, 0.0); 4];
    for i in 0..4 {
        v[i] = Vec3::new(
            (det.corners[i][0] - params.cx) / params.fx,
            (det.corners[i][1] - params.cy) / params.fy,
            1.0,
        );
    }

    // Initial pose from homography decomposition
    let initial = homography_to_pose(&h, params);

    // Run orthogonal iteration from initial estimate
    let r_init = Mat3(initial.r);
    let t_init = Vec3(initial.t);
    let (pose1, err1) = orthogonal_iteration(&v, &tag_pts, &r_init, &t_init, 50);

    // Try to find a second local minimum
    let (pose2, err2) = find_second_minimum(&v, &tag_pts, &pose1);

    match pose2 {
        Some(p2) if err2 < err1 => (p2, err2, Some(pose1), err1),
        Some(p2) => (pose1, err1, Some(p2), err2),
        // COVERAGE: None requires a perfectly frontal tag (no second minimum)
        None => (pose1, err1, None, f64::MAX),
    }
}

/// Orthogonal iteration (Lu et al. 2000).
#[allow(clippy::needless_range_loop)]
fn orthogonal_iteration(
    image_rays: &[Vec3; 4],
    tag_pts: &[Vec3; 4],
    r_init: &Mat3,
    t_init: &Vec3,
    n_iters: u32,
) -> (Pose, f64) {
    let n = 4;

    // Precompute projection operators F[i] = v*v' / (v'*v)
    let mut f_ops = [Mat3([[0.0f64; 3]; 3]); 4];
    for i in 0..n {
        let vv = image_rays[i].dot(image_rays[i]);
        f_ops[i] = image_rays[i].outer(image_rays[i]) / vv;
    }

    // Mean of object points
    let mut p_mean = Vec3::new(0.0, 0.0, 0.0);
    for i in 0..n {
        p_mean = p_mean + tag_pts[i];
    }
    p_mean = p_mean / n as f64;

    // Residuals
    let mut p_res = [Vec3::new(0.0, 0.0, 0.0); 4];
    for i in 0..n {
        p_res[i] = tag_pts[i] - p_mean;
    }

    // M1_inv = (I - mean(F))^{-1}
    let mut f_mean = Mat3([[0.0f64; 3]; 3]);
    for i in 0..n {
        f_mean += f_ops[i];
    }
    f_mean = f_mean / n as f64;
    let i_minus_fmean = Mat3::IDENTITY - f_mean;
    let m1_inv = i_minus_fmean.inv().unwrap_or(Mat3::IDENTITY);

    let mut r = *r_init;
    let mut t = *t_init;

    for _ in 0..n_iters {
        // Update translation: t = M1_inv * (1/n) * sum((F[i] - I) * R * p[i])
        let mut m2 = Vec3::new(0.0, 0.0, 0.0);
        for i in 0..n {
            let rp = r * tag_pts[i];
            let f_rp = f_ops[i] * rp;
            m2 = m2 + (f_rp - rp) / n as f64;
        }
        t = m1_inv * m2;

        // Update rotation via SVD projection
        // q[i] = F[i] * (R * p[i] + t)
        let mut q = [Vec3::new(0.0, 0.0, 0.0); 4];
        let mut q_mean = Vec3::new(0.0, 0.0, 0.0);
        for i in 0..n {
            let rp = r * tag_pts[i];
            let rp_t = rp + t;
            q[i] = f_ops[i] * rp_t;
            q_mean = q_mean + q[i];
        }
        q_mean = q_mean / n as f64;

        // M3 = sum((q[i] - q_mean) * p_res[i]')
        let mut m3 = Mat3([[0.0f64; 3]; 3]);
        for i in 0..n {
            let q_res = q[i] - q_mean;
            m3 += q_res.outer(p_res[i]);
        }

        r = project_to_so3(&m3);
    }

    // Compute object-space error
    let err = compute_error(&f_ops, &r, &t, tag_pts);

    (Pose { r: r.0, t: t.0 }, err)
}

/// Compute object-space reprojection error.
fn compute_error(f_ops: &[Mat3; 4], r: &Mat3, t: &Vec3, tag_pts: &[Vec3; 4]) -> f64 {
    let mut err = 0.0;
    for i in 0..4 {
        let rp = *r * tag_pts[i];
        let rp_t = rp + *t;
        let f_rp_t = f_ops[i] * rp_t;
        // (I - F[i]) * (R*p[i] + t)
        let diff = rp_t - f_rp_t;
        err += diff.dot(diff);
    }
    err
}

/// Search for a second local minimum (Schweighofer & Pinz 2006).
fn find_second_minimum(
    image_rays: &[Vec3; 4],
    tag_pts: &[Vec3; 4],
    pose1: &Pose,
) -> (Option<Pose>, f64) {
    let t_dir = Vec3(pose1.t);
    let t_norm = t_dir.norm();
    if t_norm < 1e-10 {
        return (None, f64::MAX);
    }
    let n = t_dir / t_norm;

    // Reflect R: R2 = (2*n*n' - I) * R
    let nn = n.outer(n);
    let reflect = nn * 2.0 - Mat3::IDENTITY;
    let r1 = Mat3(pose1.r);
    let r2 = reflect * r1;

    // Note: the angle between R and R2 = (2nn'-I)*R is always π because
    // trace(R^T*(2nn'-I)*R) = trace(2nn'-I) = -1, giving acos(-1) = π.
    // A previous "small angle" early-return was dead code and has been removed.

    // Run orthogonal iteration from the alternative starting point
    let t1 = Vec3(pose1.t);
    let (pose2, err2) = orthogonal_iteration(image_rays, tag_pts, &r2, &t1, 50);

    (Some(pose2), err2)
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use super::*;

    #[test]
    fn mat3_inv_identity() {
        let inv = Mat3::IDENTITY.inv().unwrap();
        assert_eq!(inv, Mat3::IDENTITY);
    }

    #[test]
    fn mat3_inv_roundtrip() {
        let m = Mat3([[2.0, 1.0, 0.0], [0.0, 3.0, 1.0], [1.0, 0.0, 2.0]]);
        let inv = m.inv().unwrap();
        let prod = m * inv;
        for i in 0..3 {
            for j in 0..3 {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!((prod.0[i][j] - expected).abs() < 1e-10);
            }
        }
    }

    #[test]
    fn mat3_inv_singular_returns_none() {
        let m = Mat3([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]]);
        assert!(m.inv().is_none());
    }

    #[test]
    fn pose_frontal_tag() {
        let params = PoseParams {
            tagsize: 0.1,
            fx: 500.0,
            fy: 500.0,
            cx: 320.0,
            cy: 240.0,
        };

        let s = params.tagsize / 2.0;
        let z = 5.0;
        let tag_corners_3d = [[-s, s, 0.0], [s, s, 0.0], [s, -s, 0.0], [-s, -s, 0.0]];

        let mut corners = [[0.0f64; 2]; 4];
        for i in 0..4 {
            corners[i][0] = params.cx + params.fx * tag_corners_3d[i][0] / z;
            corners[i][1] = params.cy + params.fy * tag_corners_3d[i][1] / z;
        }

        let det = Detection {
            family_id: crate::family::FamilyId::from("test"),
            id: 0,
            hamming: 0,
            decision_margin: 100.0,
            corners,
            center: [params.cx, params.cy],
        };

        let (pose, err, _, _) = estimate_tag_pose(&det, &params);

        for i in 0..3 {
            for j in 0..3 {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!((pose.r[i][j] - expected).abs() < 0.1);
            }
        }

        assert!(pose.t[0].abs() < 0.1);
        assert!(pose.t[1].abs() < 0.1);
        assert!((pose.t[2] - z).abs() < 0.5);
        assert!(err < 1e-4);
    }

    #[test]
    fn pose_offset_tag() {
        let params = PoseParams {
            tagsize: 0.2,
            fx: 500.0,
            fy: 500.0,
            cx: 320.0,
            cy: 240.0,
        };

        let s = params.tagsize / 2.0;
        let z = 3.0;
        let tx_world = 1.0;
        let tag_corners_3d = [
            [tx_world - s, s, 0.0],
            [tx_world + s, s, 0.0],
            [tx_world + s, -s, 0.0],
            [tx_world - s, -s, 0.0],
        ];

        let mut corners = [[0.0f64; 2]; 4];
        for i in 0..4 {
            corners[i][0] = params.cx + params.fx * tag_corners_3d[i][0] / z;
            corners[i][1] = params.cy + params.fy * tag_corners_3d[i][1] / z;
        }

        let det = Detection {
            family_id: crate::family::FamilyId::from("test"),
            id: 0,
            hamming: 0,
            decision_margin: 100.0,
            corners,
            center: [params.cx + params.fx * tx_world / z, params.cy],
        };

        let (pose, err, _, _) = estimate_tag_pose(&det, &params);
        assert!((pose.t[0] - tx_world).abs() < 0.2);
        assert!((pose.t[2] - z).abs() < 0.5);
        assert!(err < 1e-4);
    }

    #[test]
    fn pose_degenerate_detection() {
        let params = PoseParams {
            tagsize: 0.1,
            fx: 500.0,
            fy: 500.0,
            cx: 320.0,
            cy: 240.0,
        };
        let det = Detection {
            family_id: crate::family::FamilyId::from("test"),
            id: 0,
            hamming: 0,
            decision_margin: 100.0,
            corners: [[320.0, 240.0]; 4],
            center: [320.0, 240.0],
        };
        let (_pose, err, alt, _) = estimate_tag_pose(&det, &params);
        assert_eq!(err, f64::MAX);
        assert!(alt.is_none());
    }

    #[test]
    fn pose_oblique_tag_finds_two_solutions() {
        let params = PoseParams {
            tagsize: 0.2,
            fx: 500.0,
            fy: 500.0,
            cx: 320.0,
            cy: 240.0,
        };

        let s = params.tagsize / 2.0;
        let z = 3.0;

        let angle: f64 = 0.7;
        let ca = angle.cos();
        let sa = angle.sin();
        let tag_corners_3d: [[f64; 3]; 4] =
            [[-s, s, 0.0], [s, s, 0.0], [s, -s, 0.0], [-s, -s, 0.0]];

        let mut corners = [[0.0f64; 2]; 4];
        for i in 0..4 {
            let rx = ca * tag_corners_3d[i][0] + sa * tag_corners_3d[i][2];
            let ry = tag_corners_3d[i][1];
            let rz = -sa * tag_corners_3d[i][0] + ca * tag_corners_3d[i][2] + z;

            corners[i][0] = params.fx * rx / rz + params.cx;
            corners[i][1] = params.fy * ry / rz + params.cy;
        }

        let det = Detection {
            family_id: crate::family::FamilyId::from("test"),
            id: 0,
            hamming: 0,
            decision_margin: 100.0,
            corners,
            center: [params.cx, params.cy],
        };

        let (pose, err, alt, _) = estimate_tag_pose(&det, &params);
        assert!(err < 1.0);
        assert!(alt.is_some());
        assert!((pose.t[2] - z).abs() < 1.0);
    }

    #[test]
    fn pose_oblique_sweep() {
        let params = PoseParams {
            tagsize: 0.2,
            fx: 500.0,
            fy: 500.0,
            cx: 320.0,
            cy: 240.0,
        };

        let s = params.tagsize / 2.0;
        let tag_corners_3d: [[f64; 3]; 4] =
            [[-s, s, 0.0], [s, s, 0.0], [s, -s, 0.0], [-s, -s, 0.0]];

        for z in [0.15, 1.0, 1.5, 2.0, 3.0, 5.0] {
            for tx in [0.0, 0.3, -0.5] {
                for angle_y_deg in (20..=85).step_by(5) {
                    for angle_x_deg in [0, 15, 30, 60, 80] {
                        let ay = (angle_y_deg as f64).to_radians();
                        let ax = (angle_x_deg as f64).to_radians();
                        let (cy_, sy) = (ay.cos(), ay.sin());
                        let (cx_, sx) = (ax.cos(), ax.sin());
                        let r = [
                            [cy_, 0.0, sy],
                            [sx * sy, cx_, -sx * cy_],
                            [-cx_ * sy, sx, cx_ * cy_],
                        ];

                        let mut corners = [[0.0f64; 2]; 4];
                        let mut all_valid = true;
                        for i in 0..4 {
                            let px = r[0][0] * tag_corners_3d[i][0]
                                + r[0][1] * tag_corners_3d[i][1]
                                + r[0][2] * tag_corners_3d[i][2]
                                + tx;
                            let py = r[1][0] * tag_corners_3d[i][0]
                                + r[1][1] * tag_corners_3d[i][1]
                                + r[1][2] * tag_corners_3d[i][2];
                            let pz = r[2][0] * tag_corners_3d[i][0]
                                + r[2][1] * tag_corners_3d[i][1]
                                + r[2][2] * tag_corners_3d[i][2]
                                + z;
                            // COVERAGE: pz <= 0.01 filters poses where a corner projects behind
                            // the camera — only reachable in the sweep's extreme angles, which
                            // are test infrastructure (not production code).
                            if pz <= 0.01 {
                                all_valid = false;
                                break;
                            }
                            corners[i][0] = params.fx * px / pz + params.cx;
                            corners[i][1] = params.fy * py / pz + params.cy;
                        }
                        // COVERAGE: continuation of the pz <= 0.01 test-only filter above
                        if !all_valid {
                            continue;
                        }

                        let det = Detection {
                            family_id: crate::family::FamilyId::from("test"),
                            id: 0,
                            hamming: 0,
                            decision_margin: 100.0,
                            corners,
                            center: [
                                corners.iter().map(|c| c[0]).sum::<f64>() / 4.0,
                                corners.iter().map(|c| c[1]).sum::<f64>() / 4.0,
                            ],
                        };

                        let (pose, err, _alt, _alt_err) = estimate_tag_pose(&det, &params);
                        if err < f64::MAX {
                            assert!(pose.t[2].is_finite());
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn find_second_minimum_near_zero_translation() {
        let s = 0.05;
        let tag_pts: [Vec3; 4] = [
            Vec3::new(-s, s, 0.0),
            Vec3::new(s, s, 0.0),
            Vec3::new(s, -s, 0.0),
            Vec3::new(-s, -s, 0.0),
        ];
        let image_rays: [Vec3; 4] = [
            Vec3::new(-0.01, 0.01, 1.0),
            Vec3::new(0.01, 0.01, 1.0),
            Vec3::new(0.01, -0.01, 1.0),
            Vec3::new(-0.01, -0.01, 1.0),
        ];
        let pose = Pose {
            r: Mat3::IDENTITY.0,
            t: [0.0, 0.0, 1e-15],
        };
        let (alt, err) = find_second_minimum(&image_rays, &tag_pts, &pose);
        assert!(alt.is_none());
        assert_eq!(err, f64::MAX);
    }
}
