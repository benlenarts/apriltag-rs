use super::detector::Detection;
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

// ── 3x3 matrix helpers ──

fn mat_mul(a: &[[f64; 3]; 3], b: &[[f64; 3]; 3]) -> [[f64; 3]; 3] {
    let mut c = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            c[i][j] = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j];
        }
    }
    c
}

fn mat_vec(m: &[[f64; 3]; 3], v: &[f64; 3]) -> [f64; 3] {
    [
        m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
        m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
        m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
    ]
}

fn mat_transpose(m: &[[f64; 3]; 3]) -> [[f64; 3]; 3] {
    [
        [m[0][0], m[1][0], m[2][0]],
        [m[0][1], m[1][1], m[2][1]],
        [m[0][2], m[1][2], m[2][2]],
    ]
}

fn mat_det(m: &[[f64; 3]; 3]) -> f64 {
    m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
        - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
        + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
}

fn mat_inv(m: &[[f64; 3]; 3]) -> Option<[[f64; 3]; 3]> {
    let det = mat_det(m);
    if det.abs() < 1e-10 {
        return None;
    }
    let inv_det = 1.0 / det;
    Some([
        [
            (m[1][1] * m[2][2] - m[1][2] * m[2][1]) * inv_det,
            (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * inv_det,
            (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * inv_det,
        ],
        [
            (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * inv_det,
            (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * inv_det,
            (m[0][2] * m[1][0] - m[0][0] * m[1][2]) * inv_det,
        ],
        [
            (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * inv_det,
            (m[0][1] * m[2][0] - m[0][0] * m[2][1]) * inv_det,
            (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * inv_det,
        ],
    ])
}

fn vec_norm(v: &[f64; 3]) -> f64 {
    (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt()
}

fn cross(a: &[f64; 3], b: &[f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn outer(a: &[f64; 3], b: &[f64; 3]) -> [[f64; 3]; 3] {
    [
        [a[0] * b[0], a[0] * b[1], a[0] * b[2]],
        [a[1] * b[0], a[1] * b[1], a[1] * b[2]],
        [a[2] * b[0], a[2] * b[1], a[2] * b[2]],
    ]
}

fn dot(a: &[f64; 3], b: &[f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

const IDENTITY: [[f64; 3]; 3] = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

// ── SVD for 3x3 (Jacobi iteration) ──

/// Compute SVD of a 3x3 matrix: M = U * diag(S) * V^T.
/// Returns (U, S, V) where S is [s0, s1, s2] in decreasing order.
fn svd_3x3(m: &[[f64; 3]; 3]) -> ([[f64; 3]; 3], [f64; 3], [[f64; 3]; 3]) {
    // Compute M^T * M
    let mt = mat_transpose(m);
    let mut ata = mat_mul(&mt, m);

    // Jacobi eigendecomposition of A^T A → V, eigenvalues
    let mut v = IDENTITY;

    for _ in 0..100 {
        // Find largest off-diagonal element
        let mut max_val = 0.0;
        let mut p = 0;
        let mut q = 1;
        for i in 0..3 {
            for j in (i + 1)..3 {
                if ata[i][j].abs() > max_val {
                    max_val = ata[i][j].abs();
                    p = i;
                    q = j;
                }
            }
        }
        if max_val < 1e-15 {
            break;
        }

        // Compute Jacobi rotation angle
        let theta = 0.5 * f64::atan2(2.0 * ata[p][q], ata[p][p] - ata[q][q]);
        let c = theta.cos();
        let s = theta.sin();

        // Apply rotation to ata: ata = G^T * ata * G
        let mut new_ata = ata;
        for i in 0..3 {
            new_ata[i][p] = c * ata[i][p] + s * ata[i][q];
            new_ata[i][q] = -s * ata[i][p] + c * ata[i][q];
        }
        let tmp = new_ata;
        for j in 0..3 {
            new_ata[p][j] = c * tmp[p][j] + s * tmp[q][j];
            new_ata[q][j] = -s * tmp[p][j] + c * tmp[q][j];
        }
        ata = new_ata;

        // Accumulate V
        let mut new_v = v;
        for i in 0..3 {
            new_v[i][p] = c * v[i][p] + s * v[i][q];
            new_v[i][q] = -s * v[i][p] + c * v[i][q];
        }
        v = new_v;
    }

    // Eigenvalues are diagonal of ata; singular values are sqrt
    let eigenvalues = [ata[0][0], ata[1][1], ata[2][2]];

    // Sort by decreasing eigenvalue
    let mut order = [0usize, 1, 2];
    if eigenvalues[order[0]] < eigenvalues[order[1]] {
        order.swap(0, 1);
    }
    if eigenvalues[order[1]] < eigenvalues[order[2]] {
        order.swap(1, 2);
    }
    if eigenvalues[order[0]] < eigenvalues[order[1]] {
        order.swap(0, 1);
    }

    let sigma = [
        eigenvalues[order[0]].max(0.0).sqrt(),
        eigenvalues[order[1]].max(0.0).sqrt(),
        eigenvalues[order[2]].max(0.0).sqrt(),
    ];

    // Reorder V columns
    let mut v_sorted = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            v_sorted[i][j] = v[i][order[j]];
        }
    }

    // Ensure det(V) = +1 (we want V to be a proper rotation)
    if mat_det(&v_sorted) < 0.0 {
        for i in 0..3 {
            v_sorted[i][2] = -v_sorted[i][2];
        }
    }

    // Compute U = M * V * Sigma^{-1}
    let mv = mat_mul(m, &v_sorted);
    let mut u = [[0.0f64; 3]; 3];
    for j in 0..3 {
        if sigma[j] > 1e-10 {
            for i in 0..3 {
                u[i][j] = mv[i][j] / sigma[j];
            }
        }
    }

    // Fill in missing U columns if needed (rank-deficient case)
    if sigma[2] < 1e-10 {
        let u0 = [u[0][0], u[1][0], u[2][0]];
        let u1 = [u[0][1], u[1][1], u[2][1]];
        if sigma[1] < 1e-10 {
            // Rank <= 1
            let perp = if u0[0].abs() < 0.9 {
                [1.0, 0.0, 0.0]
            } else {
                [0.0, 1.0, 0.0]
            };
            let u1_raw = cross(&u0, &perp);
            let n1 = vec_norm(&u1_raw);
            if n1 > 1e-10 {
                let u1 = [u1_raw[0] / n1, u1_raw[1] / n1, u1_raw[2] / n1];
                let u2 = cross(&u0, &u1);
                for i in 0..3 {
                    u[i][1] = u1[i];
                    u[i][2] = u2[i];
                }
            }
        } else {
            let u2 = cross(&u0, &u1);
            for i in 0..3 {
                u[i][2] = u2[i];
            }
        }
    }

    (u, sigma, v_sorted)
}

/// Project a matrix onto SO(3) via SVD: R = U * V^T, with sign correction.
fn project_to_so3(m: &[[f64; 3]; 3]) -> [[f64; 3]; 3] {
    let (u, _s, v) = svd_3x3(m);
    let vt = mat_transpose(&v);
    let mut r = mat_mul(&u, &vt);
    if mat_det(&r) < 0.0 {
        // Negate third column of U and recompute
        let mut u_fixed = u;
        for i in 0..3 {
            u_fixed[i][2] = -u_fixed[i][2];
        }
        r = mat_mul(&u_fixed, &vt);
    }
    r
}

// ── Pose estimation ──

/// Extract initial R, t from the detection homography.
fn homography_to_pose(h: &Homography, params: &PoseParams) -> Pose {
    let fx = params.fx;
    let fy = params.fy;
    let cx = params.cx;
    let cy = params.cy;

    // K^{-1} = [[1/fx, 0, -cx/fx], [0, 1/fy, -cy/fy], [0, 0, 1]]
    let hd = &h.data;

    // K^{-1} * H columns
    let mut c0 = [
        (hd[0][0] - cx * hd[2][0]) / fx,
        (hd[1][0] - cy * hd[2][0]) / fy,
        hd[2][0],
    ];
    let mut c1 = [
        (hd[0][1] - cx * hd[2][1]) / fx,
        (hd[1][1] - cy * hd[2][1]) / fy,
        hd[2][1],
    ];
    let mut c2 = [
        (hd[0][2] - cx * hd[2][2]) / fx,
        (hd[1][2] - cy * hd[2][2]) / fy,
        hd[2][2],
    ];

    // Normalize scale
    let scale = (vec_norm(&c0) + vec_norm(&c1)) / 2.0;
    for i in 0..3 {
        c0[i] /= scale;
        c1[i] /= scale;
        c2[i] /= scale;
    }

    // Negate c1: tag parameter ty maps to -s*ty in 3D tag frame
    // so column 1 of K^{-1}*H has an embedded sign flip
    let r0 = c0;
    let r1 = [-c1[0], -c1[1], -c1[2]];
    let r2 = cross(&r0, &r1);

    // Assemble R (columns are r0, r1, r2) and project onto SO(3)
    let r_raw = [
        [r0[0], r1[0], r2[0]],
        [r0[1], r1[1], r2[1]],
        [r0[2], r1[2], r2[2]],
    ];
    let r = project_to_so3(&r_raw);

    // Scale translation by tagsize/2
    let t = [
        c2[0] * params.tagsize / 2.0,
        c2[1] * params.tagsize / 2.0,
        c2[2] * params.tagsize / 2.0,
    ];

    Pose { r, t }
}

/// Estimate the pose of a detected tag.
///
/// Returns `(best_pose, best_error, alt_pose, alt_error)`.
/// `alt_pose` is `None` when no second local minimum exists.
pub fn estimate_tag_pose(det: &Detection, params: &PoseParams) -> (Pose, f64, Option<Pose>, f64) {
    // Build homography from detection corners
    let h = match Homography::from_quad_corners(&det.corners) {
        Some(h) => h,
        None => {
            return (
                Pose {
                    r: IDENTITY,
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
    let tag_pts: [[f64; 3]; 4] = [[-s, s, 0.0], [s, s, 0.0], [s, -s, 0.0], [-s, -s, 0.0]];

    // Image rays (normalized coordinates)
    let mut v = [[0.0f64; 3]; 4];
    for i in 0..4 {
        v[i] = [
            (det.corners[i][0] - params.cx) / params.fx,
            (det.corners[i][1] - params.cy) / params.fy,
            1.0,
        ];
    }

    // Initial pose from homography decomposition
    let initial = homography_to_pose(&h, params);

    // Run orthogonal iteration from initial estimate
    let (pose1, err1) = orthogonal_iteration(&v, &tag_pts, &initial.r, &initial.t, 50);

    // Try to find a second local minimum
    let (pose2, err2) = find_second_minimum(&v, &tag_pts, &pose1);

    if err2 < err1 {
        (pose2.unwrap(), err2, Some(pose1), err1)
    } else if let Some(p2) = pose2 {
        (pose1, err1, Some(p2), err2)
    } else {
        (pose1, err1, None, f64::MAX)
    }
}

/// Orthogonal iteration (Lu et al. 2000).
fn orthogonal_iteration(
    image_rays: &[[f64; 3]; 4],
    tag_pts: &[[f64; 3]; 4],
    r_init: &[[f64; 3]; 3],
    t_init: &[f64; 3],
    n_iters: u32,
) -> (Pose, f64) {
    let n = 4;

    // Precompute projection operators F[i] = v*v' / (v'*v)
    let mut f_ops = [[[0.0f64; 3]; 3]; 4];
    for i in 0..n {
        let vv = dot(&image_rays[i], &image_rays[i]);
        f_ops[i] = outer(&image_rays[i], &image_rays[i]);
        for r in 0..3 {
            for c in 0..3 {
                f_ops[i][r][c] /= vv;
            }
        }
    }

    // Mean of object points
    let mut p_mean = [0.0; 3];
    for i in 0..n {
        for j in 0..3 {
            p_mean[j] += tag_pts[i][j];
        }
    }
    for j in 0..3 {
        p_mean[j] /= n as f64;
    }

    // Residuals
    let mut p_res = [[0.0f64; 3]; 4];
    for i in 0..n {
        for j in 0..3 {
            p_res[i][j] = tag_pts[i][j] - p_mean[j];
        }
    }

    // M1_inv = (I - mean(F))^{-1}
    let mut f_mean = [[0.0f64; 3]; 3];
    for i in 0..n {
        for r in 0..3 {
            for c in 0..3 {
                f_mean[r][c] += f_ops[i][r][c];
            }
        }
    }
    for r in 0..3 {
        for c in 0..3 {
            f_mean[r][c] /= n as f64;
        }
    }
    let mut i_minus_fmean = IDENTITY;
    for r in 0..3 {
        for c in 0..3 {
            i_minus_fmean[r][c] -= f_mean[r][c];
        }
    }
    let m1_inv = mat_inv(&i_minus_fmean).unwrap_or(IDENTITY);

    let mut r = *r_init;
    let mut t = *t_init;

    for _ in 0..n_iters {
        // Update translation: t = M1_inv * (1/n) * sum((F[i] - I) * R * p[i])
        let mut m2 = [0.0f64; 3];
        for i in 0..n {
            let rp = mat_vec(&r, &tag_pts[i]);
            let f_rp = mat_vec(&f_ops[i], &rp);
            for j in 0..3 {
                m2[j] += (f_rp[j] - rp[j]) / n as f64;
            }
        }
        t = mat_vec(&m1_inv, &m2);

        // Update rotation via SVD projection
        // q[i] = F[i] * (R * p[i] + t)
        let mut q = [[0.0f64; 3]; 4];
        let mut q_mean = [0.0f64; 3];
        for i in 0..n {
            let rp = mat_vec(&r, &tag_pts[i]);
            let rp_t = [rp[0] + t[0], rp[1] + t[1], rp[2] + t[2]];
            q[i] = mat_vec(&f_ops[i], &rp_t);
            for j in 0..3 {
                q_mean[j] += q[i][j];
            }
        }
        for j in 0..3 {
            q_mean[j] /= n as f64;
        }

        // M3 = sum((q[i] - q_mean) * p_res[i]')
        let mut m3 = [[0.0f64; 3]; 3];
        for i in 0..n {
            let q_res = [
                q[i][0] - q_mean[0],
                q[i][1] - q_mean[1],
                q[i][2] - q_mean[2],
            ];
            let op = outer(&q_res, &p_res[i]);
            for a in 0..3 {
                for b in 0..3 {
                    m3[a][b] += op[a][b];
                }
            }
        }

        r = project_to_so3(&m3);
    }

    // Compute object-space error
    let err = compute_error(&f_ops, &r, &t, tag_pts);

    (Pose { r, t }, err)
}

/// Compute object-space reprojection error.
fn compute_error(
    f_ops: &[[[f64; 3]; 3]; 4],
    r: &[[f64; 3]; 3],
    t: &[f64; 3],
    tag_pts: &[[f64; 3]; 4],
) -> f64 {
    let mut err = 0.0;
    for i in 0..4 {
        let rp = mat_vec(r, &tag_pts[i]);
        let rp_t = [rp[0] + t[0], rp[1] + t[1], rp[2] + t[2]];
        let f_rp_t = mat_vec(&f_ops[i], &rp_t);
        // (I - F[i]) * (R*p[i] + t)
        for j in 0..3 {
            let diff = rp_t[j] - f_rp_t[j];
            err += diff * diff;
        }
    }
    err
}

/// Search for a second local minimum (Schweighofer & Pinz 2006).
fn find_second_minimum(
    image_rays: &[[f64; 3]; 4],
    tag_pts: &[[f64; 3]; 4],
    pose1: &Pose,
) -> (Option<Pose>, f64) {
    // The second minimum lies at a rotation of ~180 degrees around the
    // axis connecting the camera to the tag center.
    //
    // We construct the alternative initial rotation by reflecting the
    // rotation matrix across the viewing direction.
    let t_dir = [pose1.t[0], pose1.t[1], pose1.t[2]];
    let t_norm = vec_norm(&t_dir);
    if t_norm < 1e-10 {
        return (None, f64::MAX);
    }
    let n = [t_dir[0] / t_norm, t_dir[1] / t_norm, t_dir[2] / t_norm];

    // Reflect R: R2 = (2*n*n' - I) * R
    // This is equivalent to a 180-degree rotation about the viewing axis
    let nn = outer(&n, &n);
    let mut reflect = [[0.0f64; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            reflect[i][j] = 2.0 * nn[i][j] - IDENTITY[i][j];
        }
    }
    let r2 = mat_mul(&reflect, &pose1.r);

    // Check if the alternative rotation differs enough (> 0.1 rad)
    let rt = mat_transpose(&pose1.r);
    let diff_rot = mat_mul(&rt, &r2);
    // Rotation angle from trace: cos(angle) = (trace - 1) / 2
    let trace = diff_rot[0][0] + diff_rot[1][1] + diff_rot[2][2];
    let cos_angle = ((trace - 1.0) / 2.0).clamp(-1.0, 1.0);
    let angle = cos_angle.acos();

    if angle < 0.1 {
        return (None, f64::MAX);
    }

    // Run orthogonal iteration from the alternative starting point
    let (pose2, err2) = orthogonal_iteration(image_rays, tag_pts, &r2, &pose1.t, 50);

    (Some(pose2), err2)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mat_mul_identity() {
        let a = [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]];
        let result = mat_mul(&IDENTITY, &a);
        for i in 0..3 {
            for j in 0..3 {
                assert!((result[i][j] - a[i][j]).abs() < 1e-10);
            }
        }
    }

    #[test]
    fn mat_inv_identity() {
        let inv = mat_inv(&IDENTITY).unwrap();
        for i in 0..3 {
            for j in 0..3 {
                assert!((inv[i][j] - IDENTITY[i][j]).abs() < 1e-10);
            }
        }
    }

    #[test]
    fn mat_inv_roundtrip() {
        let m = [[2.0, 1.0, 0.0], [0.0, 3.0, 1.0], [1.0, 0.0, 2.0]];
        let inv = mat_inv(&m).unwrap();
        let prod = mat_mul(&m, &inv);
        for i in 0..3 {
            for j in 0..3 {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!(
                    (prod[i][j] - expected).abs() < 1e-10,
                    "prod[{i}][{j}] = {}",
                    prod[i][j]
                );
            }
        }
    }

    #[test]
    fn svd_identity() {
        let (u, s, v) = svd_3x3(&IDENTITY);
        for i in 0..3 {
            assert!((s[i] - 1.0).abs() < 1e-10, "s[{i}] = {}", s[i]);
        }
        // U*V^T should be identity
        let vt = mat_transpose(&v);
        let r = mat_mul(&u, &vt);
        for i in 0..3 {
            for j in 0..3 {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!(
                    (r[i][j] - expected).abs() < 1e-10,
                    "r[{i}][{j}] = {}",
                    r[i][j]
                );
            }
        }
    }

    #[test]
    fn svd_diagonal() {
        let m = [[3.0, 0.0, 0.0], [0.0, 2.0, 0.0], [0.0, 0.0, 1.0]];
        let (_u, s, _v) = svd_3x3(&m);
        assert!((s[0] - 3.0).abs() < 1e-10);
        assert!((s[1] - 2.0).abs() < 1e-10);
        assert!((s[2] - 1.0).abs() < 1e-10);
    }

    #[test]
    fn svd_reconstructs_matrix() {
        let m = [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 10.0]];
        let (u, s, v) = svd_3x3(&m);
        // Reconstruct: U * diag(S) * V^T
        let mut us = [[0.0; 3]; 3];
        for i in 0..3 {
            for j in 0..3 {
                us[i][j] = u[i][j] * s[j];
            }
        }
        let vt = mat_transpose(&v);
        let recon = mat_mul(&us, &vt);
        for i in 0..3 {
            for j in 0..3 {
                assert!(
                    (recon[i][j] - m[i][j]).abs() < 1e-8,
                    "recon[{i}][{j}]={} vs m={}",
                    recon[i][j],
                    m[i][j],
                );
            }
        }
    }

    #[test]
    fn project_to_so3_rotation() {
        // A proper rotation should remain unchanged
        let angle: f64 = 0.3;
        let r = [
            [angle.cos(), -angle.sin(), 0.0],
            [angle.sin(), angle.cos(), 0.0],
            [0.0, 0.0, 1.0],
        ];
        let proj = project_to_so3(&r);
        for i in 0..3 {
            for j in 0..3 {
                assert!(
                    (proj[i][j] - r[i][j]).abs() < 1e-10,
                    "proj[{i}][{j}]={} vs r={}",
                    proj[i][j],
                    r[i][j]
                );
            }
        }
    }

    #[test]
    fn project_to_so3_noisy() {
        // A nearly-rotation matrix should project correctly
        let angle: f64 = 0.5;
        let mut r = [
            [angle.cos(), -angle.sin(), 0.0],
            [angle.sin(), angle.cos(), 0.0],
            [0.0, 0.0, 1.0],
        ];
        // Add noise
        r[0][0] += 0.05;
        r[1][1] -= 0.03;
        let proj = project_to_so3(&r);
        // Should be orthogonal
        let rrt = mat_mul(&proj, &mat_transpose(&proj));
        for i in 0..3 {
            for j in 0..3 {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!(
                    (rrt[i][j] - expected).abs() < 1e-10,
                    "R*R^T[{i}][{j}]={}",
                    rrt[i][j]
                );
            }
        }
        assert!((mat_det(&proj) - 1.0).abs() < 1e-10);
    }

    #[test]
    fn pose_frontal_tag() {
        // Simulate a frontal tag at z=5, centered at image center
        let params = PoseParams {
            tagsize: 0.1,
            fx: 500.0,
            fy: 500.0,
            cx: 320.0,
            cy: 240.0,
        };

        // Tag at z=5 → projects to:
        // corner i at (cx + fx * tag_x / z, cy + fy * tag_y / z)
        let s = params.tagsize / 2.0;
        let z = 5.0;
        let tag_corners_3d = [
            [-s, s, 0.0],  // (-s, +s)
            [s, s, 0.0],   // (+s, +s)
            [s, -s, 0.0],  // (+s, -s)
            [-s, -s, 0.0], // (-s, -s)
        ];

        let mut corners = [[0.0f64; 2]; 4];
        for i in 0..4 {
            corners[i][0] = params.cx + params.fx * tag_corners_3d[i][0] / z;
            corners[i][1] = params.cy + params.fy * tag_corners_3d[i][1] / z;
        }

        let det = Detection {
            family_name: "test".to_string(),
            id: 0,
            hamming: 0,
            decision_margin: 100.0,
            corners,
            center: [params.cx, params.cy],
        };

        let (pose, err, _, _) = estimate_tag_pose(&det, &params);

        // R should be close to identity
        for i in 0..3 {
            for j in 0..3 {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!(
                    (pose.r[i][j] - expected).abs() < 0.1,
                    "R[{i}][{j}]={}, expected ~{}",
                    pose.r[i][j],
                    expected,
                );
            }
        }

        // t should be ~[0, 0, 5]
        assert!(pose.t[0].abs() < 0.1, "tx={}", pose.t[0]);
        assert!(pose.t[1].abs() < 0.1, "ty={}", pose.t[1]);
        assert!(
            (pose.t[2] - z).abs() < 0.5,
            "tz={}, expected ~{z}",
            pose.t[2],
        );

        // Error should be small
        assert!(err < 1e-4, "error={err}");
    }

    #[test]
    fn pose_offset_tag() {
        // Tag at z=3, shifted to the right by 1 meter
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
            family_name: "test".to_string(),
            id: 0,
            hamming: 0,
            decision_margin: 100.0,
            corners,
            center: [params.cx + params.fx * tx_world / z, params.cy],
        };

        let (pose, err, _, _) = estimate_tag_pose(&det, &params);

        // t should be ~[1, 0, 3]
        assert!(
            (pose.t[0] - tx_world).abs() < 0.2,
            "tx={}, expected ~{tx_world}",
            pose.t[0],
        );
        assert!(
            (pose.t[2] - z).abs() < 0.5,
            "tz={}, expected ~{z}",
            pose.t[2],
        );
        assert!(err < 1e-4, "error={err}");
    }

    #[test]
    fn mat_inv_singular_returns_none() {
        let m = [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]]; // det = 0
        assert!(mat_inv(&m).is_none());
    }

    #[test]
    fn svd_rank_deficient() {
        // Rank-1 matrix: only one nonzero singular value
        let m = [[1.0, 2.0, 3.0], [2.0, 4.0, 6.0], [3.0, 6.0, 9.0]];
        let (u, s, v) = svd_3x3(&m);
        // Only first singular value should be nonzero
        assert!(s[0] > 1.0, "s[0]={}", s[0]);
        assert!(s[1] < 1e-8, "s[1]={}", s[1]);
        assert!(s[2] < 1e-8, "s[2]={}", s[2]);

        // Reconstruct
        let mut us = [[0.0; 3]; 3];
        for i in 0..3 {
            for j in 0..3 {
                us[i][j] = u[i][j] * s[j];
            }
        }
        let vt = mat_transpose(&v);
        let recon = mat_mul(&us, &vt);
        for i in 0..3 {
            for j in 0..3 {
                assert!(
                    (recon[i][j] - m[i][j]).abs() < 1e-6,
                    "recon[{i}][{j}]={} vs m={}",
                    recon[i][j],
                    m[i][j],
                );
            }
        }
    }

    #[test]
    fn project_to_so3_negative_det() {
        // A matrix with negative determinant should still project to SO(3)
        let m = [[-1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let r = project_to_so3(&m);
        let rrt = mat_mul(&r, &mat_transpose(&r));
        for i in 0..3 {
            for j in 0..3 {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!(
                    (rrt[i][j] - expected).abs() < 1e-10,
                    "R*R^T[{i}][{j}]={}",
                    rrt[i][j]
                );
            }
        }
        assert!((mat_det(&r) - 1.0).abs() < 1e-10, "det={}", mat_det(&r));
    }

    #[test]
    fn pose_degenerate_detection() {
        // All corners at the same point → degenerate homography
        let params = PoseParams {
            tagsize: 0.1,
            fx: 500.0,
            fy: 500.0,
            cx: 320.0,
            cy: 240.0,
        };
        let det = Detection {
            family_name: "test".to_string(),
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
        // Tag at an oblique angle — should find two pose solutions
        let params = PoseParams {
            tagsize: 0.2,
            fx: 500.0,
            fy: 500.0,
            cx: 320.0,
            cy: 240.0,
        };

        let s = params.tagsize / 2.0;
        let z = 3.0;

        // Rotate tag 45 degrees around Y axis
        let angle: f64 = 0.7;
        let ca = angle.cos();
        let sa = angle.sin();
        // R_y(angle) = [[cos, 0, sin], [0, 1, 0], [-sin, 0, cos]]
        let tag_corners_3d: [[f64; 3]; 4] =
            [[-s, s, 0.0], [s, s, 0.0], [s, -s, 0.0], [-s, -s, 0.0]];

        let mut corners = [[0.0f64; 2]; 4];
        for i in 0..4 {
            // Apply rotation around Y
            let rx = ca * tag_corners_3d[i][0] + sa * tag_corners_3d[i][2];
            let ry = tag_corners_3d[i][1];
            let rz = -sa * tag_corners_3d[i][0] + ca * tag_corners_3d[i][2] + z;

            corners[i][0] = params.fx * rx / rz + params.cx;
            corners[i][1] = params.fy * ry / rz + params.cy;
        }

        let det = Detection {
            family_name: "test".to_string(),
            id: 0,
            hamming: 0,
            decision_margin: 100.0,
            corners,
            center: [params.cx, params.cy],
        };

        let (pose, err, alt, _) = estimate_tag_pose(&det, &params);
        // Should find a pose with reasonable error
        assert!(err < 1.0, "error={err}");
        // Oblique tag should produce two solutions
        assert!(alt.is_some(), "Expected two pose solutions for oblique tag");
        // The best pose should place the tag at approximately z=3
        assert!(
            (pose.t[2] - z).abs() < 1.0,
            "tz={}, expected ~{z}",
            pose.t[2],
        );
    }

    #[test]
    fn svd_eigenvalue_ordering() {
        // Matrix whose eigenvalues of M^T*M need re-sorting
        let m = [[0.0, 0.0, 5.0], [0.0, 3.0, 0.0], [1.0, 0.0, 0.0]];
        let (_u, s, _v) = svd_3x3(&m);
        // Singular values should be in decreasing order
        assert!(s[0] >= s[1], "s[0]={} < s[1]={}", s[0], s[1]);
        assert!(s[1] >= s[2], "s[1]={} < s[2]={}", s[1], s[2]);
        assert!((s[0] - 5.0).abs() < 1e-8, "s[0]={}", s[0]);
        assert!((s[1] - 3.0).abs() < 1e-8, "s[1]={}", s[1]);
        assert!((s[2] - 1.0).abs() < 1e-8, "s[2]={}", s[2]);
    }
}
