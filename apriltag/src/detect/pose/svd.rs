use super::super::mat3::{Mat3, Vec3};

/// Compute SVD of a 3x3 matrix: M = U * diag(S) * V^T.
/// Returns (U, S, V) where S is [s0, s1, s2] in decreasing order.
#[allow(clippy::needless_range_loop)]
pub(super) fn svd_3x3(m: &Mat3) -> (Mat3, [f64; 3], Mat3) {
    // Compute M^T * M
    let mt = m.transpose();
    let mut ata = mt * *m;

    // Jacobi eigendecomposition of A^T A → V, eigenvalues
    let mut v = Mat3::IDENTITY;

    for _ in 0..100 {
        // Find largest off-diagonal element
        let mut max_val = 0.0;
        let mut p = 0;
        let mut q = 1;
        for i in 0..3 {
            for j in (i + 1)..3 {
                if ata.0[i][j].abs() > max_val {
                    max_val = ata.0[i][j].abs();
                    p = i;
                    q = j;
                }
            }
        }
        if max_val < 1e-15 {
            break;
        }

        // Compute Jacobi rotation angle
        let theta = 0.5 * f64::atan2(2.0 * ata.0[p][q], ata.0[p][p] - ata.0[q][q]);
        let c = theta.cos();
        let s = theta.sin();

        // Apply rotation to ata: ata = G^T * ata * G
        let mut new_ata = ata;
        for i in 0..3 {
            new_ata.0[i][p] = c * ata.0[i][p] + s * ata.0[i][q];
            new_ata.0[i][q] = -s * ata.0[i][p] + c * ata.0[i][q];
        }
        let tmp = new_ata;
        for j in 0..3 {
            new_ata.0[p][j] = c * tmp.0[p][j] + s * tmp.0[q][j];
            new_ata.0[q][j] = -s * tmp.0[p][j] + c * tmp.0[q][j];
        }
        ata = new_ata;

        // Accumulate V
        let mut new_v = v;
        for i in 0..3 {
            new_v.0[i][p] = c * v.0[i][p] + s * v.0[i][q];
            new_v.0[i][q] = -s * v.0[i][p] + c * v.0[i][q];
        }
        v = new_v;
    }

    // Eigenvalues are diagonal of ata; singular values are sqrt
    let eigenvalues = [ata.0[0][0], ata.0[1][1], ata.0[2][2]];

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
    let mut v_sorted = Mat3([[0.0; 3]; 3]);
    for i in 0..3 {
        for j in 0..3 {
            v_sorted.0[i][j] = v.0[i][order[j]];
        }
    }

    // Ensure det(V) = +1 (we want V to be a proper rotation)
    if v_sorted.det() < 0.0 {
        for i in 0..3 {
            v_sorted.0[i][2] = -v_sorted.0[i][2];
        }
    }

    // Compute U = M * V * Sigma^{-1}
    let mv = *m * v_sorted;
    let mut u = Mat3([[0.0f64; 3]; 3]);
    for j in 0..3 {
        if sigma[j] > 1e-10 {
            for i in 0..3 {
                u.0[i][j] = mv.0[i][j] / sigma[j];
            }
        }
    }

    // Fill in missing U columns if needed (rank-deficient case)
    if sigma[2] < 1e-10 {
        let u0 = Vec3::new(u.0[0][0], u.0[1][0], u.0[2][0]);
        let u1 = Vec3::new(u.0[0][1], u.0[1][1], u.0[2][1]);
        if sigma[1] < 1e-10 {
            // Rank <= 1
            let perp = if u0[0].abs() < 0.9 {
                Vec3::new(1.0, 0.0, 0.0)
            } else {
                Vec3::new(0.0, 1.0, 0.0)
            };
            let u1_raw = u0.cross(perp);
            let n1 = u1_raw.norm();
            if n1 > 1e-10 {
                let u1 = u1_raw / n1;
                let u2 = u0.cross(u1);
                for i in 0..3 {
                    u.0[i][1] = u1[i];
                    u.0[i][2] = u2[i];
                }
            }
        } else {
            let u2 = u0.cross(u1);
            for i in 0..3 {
                u.0[i][2] = u2[i];
            }
        }
    }

    (u, sigma, v_sorted)
}

/// Project a matrix onto SO(3) via SVD: R = U * V^T, with sign correction.
pub(super) fn project_to_so3(m: &Mat3) -> Mat3 {
    let (u, _s, v) = svd_3x3(m);
    let vt = v.transpose();
    let mut r = u * vt;
    if r.det() < 0.0 {
        // Negate third column of U and recompute
        let mut u_fixed = u;
        for i in 0..3 {
            u_fixed.0[i][2] = -u_fixed.0[i][2];
        }
        r = u_fixed * vt;
    }
    r
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use super::*;

    #[test]
    fn svd_identity() {
        let (u, s, v) = svd_3x3(&Mat3::IDENTITY);
        for i in 0..3 {
            assert!((s[i] - 1.0).abs() < 1e-10);
        }
        let r = u * v.transpose();
        for i in 0..3 {
            for j in 0..3 {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!((r.0[i][j] - expected).abs() < 1e-10);
            }
        }
    }

    #[test]
    fn svd_diagonal() {
        let m = Mat3([[3.0, 0.0, 0.0], [0.0, 2.0, 0.0], [0.0, 0.0, 1.0]]);
        let (_u, s, _v) = svd_3x3(&m);
        assert!((s[0] - 3.0).abs() < 1e-10);
        assert!((s[1] - 2.0).abs() < 1e-10);
        assert!((s[2] - 1.0).abs() < 1e-10);
    }

    #[test]
    fn svd_reconstructs_matrix() {
        let m = Mat3([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 10.0]]);
        let (u, s, v) = svd_3x3(&m);
        let mut us = Mat3([[0.0; 3]; 3]);
        for i in 0..3 {
            for j in 0..3 {
                us.0[i][j] = u.0[i][j] * s[j];
            }
        }
        let recon = us * v.transpose();
        for i in 0..3 {
            for j in 0..3 {
                assert!((recon.0[i][j] - m.0[i][j]).abs() < 1e-8);
            }
        }
    }

    #[test]
    fn svd_rank_deficient() {
        let m = Mat3([[1.0, 2.0, 3.0], [2.0, 4.0, 6.0], [3.0, 6.0, 9.0]]);
        let (u, s, v) = svd_3x3(&m);
        assert!(s[0] > 1.0);
        assert!(s[1] < 1e-8);
        assert!(s[2] < 1e-8);

        let mut us = Mat3([[0.0; 3]; 3]);
        for i in 0..3 {
            for j in 0..3 {
                us.0[i][j] = u.0[i][j] * s[j];
            }
        }
        let recon = us * v.transpose();
        for i in 0..3 {
            for j in 0..3 {
                assert!((recon.0[i][j] - m.0[i][j]).abs() < 1e-6);
            }
        }
    }

    #[test]
    fn svd_rank1_dominant_x() {
        let m = Mat3([[5.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]);
        let (u, s, v) = svd_3x3(&m);
        assert!(s[0] > 1.0);
        assert!(s[1] < 1e-8);
        assert!(s[2] < 1e-8);
        let uut = u * u.transpose();
        for i in 0..3 {
            for j in 0..3 {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!((uut.0[i][j] - expected).abs() < 1e-8);
            }
        }
        let mut us = Mat3([[0.0; 3]; 3]);
        for i in 0..3 {
            for j in 0..3 {
                us.0[i][j] = u.0[i][j] * s[j];
            }
        }
        let recon = us * v.transpose();
        for i in 0..3 {
            for j in 0..3 {
                assert!((recon.0[i][j] - m.0[i][j]).abs() < 1e-6);
            }
        }
    }

    #[test]
    fn svd_zero_matrix() {
        let m = Mat3([[0.0; 3]; 3]);
        let (_u, s, _v) = svd_3x3(&m);
        for i in 0..3 {
            assert!(s[i] < 1e-10);
        }
    }

    #[test]
    fn svd_eigenvalue_ordering() {
        let m = Mat3([[0.0, 0.0, 5.0], [0.0, 3.0, 0.0], [1.0, 0.0, 0.0]]);
        let (_u, s, _v) = svd_3x3(&m);
        assert!(s[0] >= s[1]);
        assert!(s[1] >= s[2]);
        assert!((s[0] - 5.0).abs() < 1e-8);
        assert!((s[1] - 3.0).abs() < 1e-8);
        assert!((s[2] - 1.0).abs() < 1e-8);
    }

    #[test]
    fn project_to_so3_rotation() {
        let angle: f64 = 0.3;
        let r = Mat3([
            [angle.cos(), -angle.sin(), 0.0],
            [angle.sin(), angle.cos(), 0.0],
            [0.0, 0.0, 1.0],
        ]);
        let proj = project_to_so3(&r);
        for i in 0..3 {
            for j in 0..3 {
                assert!((proj.0[i][j] - r.0[i][j]).abs() < 1e-10);
            }
        }
    }

    #[test]
    fn project_to_so3_noisy() {
        let angle: f64 = 0.5;
        let mut r = Mat3([
            [angle.cos(), -angle.sin(), 0.0],
            [angle.sin(), angle.cos(), 0.0],
            [0.0, 0.0, 1.0],
        ]);
        r.0[0][0] += 0.05;
        r.0[1][1] -= 0.03;
        let proj = project_to_so3(&r);
        let rrt = proj * proj.transpose();
        for i in 0..3 {
            for j in 0..3 {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!((rrt.0[i][j] - expected).abs() < 1e-10);
            }
        }
        assert!((proj.det() - 1.0).abs() < 1e-10);
    }

    #[test]
    fn project_to_so3_negative_det() {
        let m = Mat3([[-1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]);
        let r = project_to_so3(&m);
        let rrt = r * r.transpose();
        for i in 0..3 {
            for j in 0..3 {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!((rrt.0[i][j] - expected).abs() < 1e-10);
            }
        }
        assert!((r.det() - 1.0).abs() < 1e-10);
    }
}
