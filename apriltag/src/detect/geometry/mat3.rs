use core::ops;

use super::Vec3;

/// A 3×3 matrix, wrapping `[[f64; 3]; 3]` (row-major).
#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(transparent)]
pub struct Mat3(pub [[f64; 3]; 3]);

impl Mat3 {
    /// The 3×3 identity matrix.
    pub const IDENTITY: Self = Self([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]);

    /// Determinant.
    pub fn det(self) -> f64 {
        let m = &self.0;
        m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
            - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
    }

    /// Inverse, or `None` if singular.
    pub fn inv(self) -> Option<Self> {
        let d = self.det();
        if d.abs() < 1e-10 {
            return None;
        }
        let m = &self.0;
        let inv_det = 1.0 / d;
        Some(Self([
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
        ]))
    }

    /// Transpose.
    pub fn transpose(self) -> Self {
        let m = &self.0;
        Self([
            [m[0][0], m[1][0], m[2][0]],
            [m[0][1], m[1][1], m[2][1]],
            [m[0][2], m[1][2], m[2][2]],
        ])
    }
}

// Mat3 * Mat3
impl ops::Mul<Mat3> for Mat3 {
    type Output = Mat3;
    fn mul(self, rhs: Mat3) -> Mat3 {
        let (a, b) = (&self.0, &rhs.0);
        Mat3(core::array::from_fn(|i| {
            core::array::from_fn(|j| a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j])
        }))
    }
}

// Mat3 * Vec3
impl ops::Mul<Vec3> for Mat3 {
    type Output = Vec3;
    fn mul(self, v: Vec3) -> Vec3 {
        Vec3([
            self.0[0][0] * v.0[0] + self.0[0][1] * v.0[1] + self.0[0][2] * v.0[2],
            self.0[1][0] * v.0[0] + self.0[1][1] * v.0[1] + self.0[1][2] * v.0[2],
            self.0[2][0] * v.0[0] + self.0[2][1] * v.0[1] + self.0[2][2] * v.0[2],
        ])
    }
}

// Mat3 * f64
impl ops::Mul<f64> for Mat3 {
    type Output = Mat3;
    fn mul(self, s: f64) -> Mat3 {
        Mat3([
            [self.0[0][0] * s, self.0[0][1] * s, self.0[0][2] * s],
            [self.0[1][0] * s, self.0[1][1] * s, self.0[1][2] * s],
            [self.0[2][0] * s, self.0[2][1] * s, self.0[2][2] * s],
        ])
    }
}

// Mat3 / f64
impl ops::Div<f64> for Mat3 {
    type Output = Mat3;
    fn div(self, s: f64) -> Mat3 {
        self * (1.0 / s)
    }
}

// Mat3 + Mat3
impl ops::Add for Mat3 {
    type Output = Mat3;
    fn add(self, rhs: Mat3) -> Mat3 {
        Mat3(core::array::from_fn(|i| {
            core::array::from_fn(|j| self.0[i][j] + rhs.0[i][j])
        }))
    }
}

// Mat3 - Mat3
impl ops::Sub for Mat3 {
    type Output = Mat3;
    fn sub(self, rhs: Mat3) -> Mat3 {
        Mat3(core::array::from_fn(|i| {
            core::array::from_fn(|j| self.0[i][j] - rhs.0[i][j])
        }))
    }
}

// Mat3 += Mat3
impl ops::AddAssign for Mat3 {
    fn add_assign(&mut self, rhs: Mat3) {
        for (row, rhs_row) in self.0.iter_mut().zip(rhs.0.iter()) {
            for (a, b) in row.iter_mut().zip(rhs_row.iter()) {
                *a += b;
            }
        }
    }
}

// Mat3 -= Mat3
impl ops::SubAssign for Mat3 {
    fn sub_assign(&mut self, rhs: Mat3) {
        for (row, rhs_row) in self.0.iter_mut().zip(rhs.0.iter()) {
            for (a, b) in row.iter_mut().zip(rhs_row.iter()) {
                *a -= b;
            }
        }
    }
}

/// Determinant of a 3x3 matrix.
pub fn det(m: &[[f64; 3]; 3]) -> f64 {
    Mat3(*m).det()
}

/// Inverse of a 3x3 matrix, or `None` if singular.
pub fn inv(m: &[[f64; 3]; 3]) -> Option<[[f64; 3]; 3]> {
    Mat3(*m).inv().map(|m| m.0)
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use super::*;

    #[test]
    fn mat3_det_identity() {
        assert!((Mat3::IDENTITY.det() - 1.0).abs() < 1e-10);
    }

    #[test]
    fn mat3_det_singular() {
        let m = Mat3([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]]);
        assert!(m.det().abs() < 1e-10);
    }

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
    fn mat3_inv_singular() {
        let m = Mat3([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]]);
        assert!(m.inv().is_none());
    }

    #[test]
    fn mat3_transpose() {
        let m = Mat3([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]]);
        let t = m.transpose();
        assert!((t.0[0][1] - 4.0).abs() < 1e-10);
        assert!((t.0[1][0] - 2.0).abs() < 1e-10);
    }

    #[test]
    fn mat3_mul_identity() {
        let m = Mat3([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]]);
        let prod = Mat3::IDENTITY * m;
        assert_eq!(prod, m);
    }

    #[test]
    fn mat3_mul_vec3() {
        let m = Mat3::IDENTITY;
        let v = Vec3::new(1.0, 2.0, 3.0);
        let result = m * v;
        assert_eq!(result, v);
    }

    #[test]
    fn mat3_scalar_ops() {
        let m = Mat3([[1.0, 0.0, 0.0], [0.0, 2.0, 0.0], [0.0, 0.0, 3.0]]);
        let scaled = m * 2.0;
        assert!((scaled.0[1][1] - 4.0).abs() < 1e-10);
        let divided = m / 2.0;
        assert!((divided.0[2][2] - 1.5).abs() < 1e-10);
    }

    #[test]
    fn mat3_add_sub() {
        let a = Mat3::IDENTITY;
        let b = Mat3::IDENTITY;
        let sum = a + b;
        assert!((sum.0[0][0] - 2.0).abs() < 1e-10);
        let diff = a - b;
        assert!((diff.0[0][0]).abs() < 1e-10);
    }

    #[test]
    fn mat3_add_assign_sub_assign() {
        let mut m = Mat3::IDENTITY;
        m += Mat3::IDENTITY;
        assert!((m.0[0][0] - 2.0).abs() < 1e-10);
        m -= Mat3::IDENTITY;
        assert!((m.0[0][0] - 1.0).abs() < 1e-10);
    }

    #[test]
    fn legacy_det() {
        let m = [[2.0, 0.0, 0.0], [0.0, 3.0, 0.0], [0.0, 0.0, 4.0]];
        assert!((det(&m) - 24.0).abs() < 1e-10);
    }

    #[test]
    fn legacy_inv() {
        let m = [[2.0, 1.0, 0.0], [0.0, 3.0, 1.0], [1.0, 0.0, 2.0]];
        let i = inv(&m).unwrap();
        let prod = Mat3(m) * Mat3(i);
        for r in 0..3 {
            for c in 0..3 {
                let expected = if r == c { 1.0 } else { 0.0 };
                assert!((prod.0[r][c] - expected).abs() < 1e-10);
            }
        }
    }
}
