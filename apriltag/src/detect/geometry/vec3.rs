use core::ops;

use super::Mat3;

/// A 3D vector, wrapping `[f64; 3]`.
#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(transparent)]
pub struct Vec3(pub [f64; 3]);

impl Vec3 {
    /// Create a new `Vec3`.
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self([x, y, z])
    }

    /// Dot product.
    pub fn dot(self, rhs: Self) -> f64 {
        self.0[0] * rhs.0[0] + self.0[1] * rhs.0[1] + self.0[2] * rhs.0[2]
    }

    /// Cross product.
    pub fn cross(self, rhs: Self) -> Self {
        Self([
            self.0[1] * rhs.0[2] - self.0[2] * rhs.0[1],
            self.0[2] * rhs.0[0] - self.0[0] * rhs.0[2],
            self.0[0] * rhs.0[1] - self.0[1] * rhs.0[0],
        ])
    }

    /// Outer product: self * rhs^T → Mat3.
    pub fn outer(self, rhs: Self) -> Mat3 {
        Mat3([
            [
                self.0[0] * rhs.0[0],
                self.0[0] * rhs.0[1],
                self.0[0] * rhs.0[2],
            ],
            [
                self.0[1] * rhs.0[0],
                self.0[1] * rhs.0[1],
                self.0[1] * rhs.0[2],
            ],
            [
                self.0[2] * rhs.0[0],
                self.0[2] * rhs.0[1],
                self.0[2] * rhs.0[2],
            ],
        ])
    }

    /// Euclidean norm.
    pub fn norm(self) -> f64 {
        self.dot(self).sqrt()
    }

    /// Unit vector in the same direction, or zero vector if norm is near zero.
    pub fn normalized(self) -> Self {
        let n = self.norm();
        if n < 1e-15 {
            Self([0.0; 3])
        } else {
            self / n
        }
    }
}

impl ops::Index<usize> for Vec3 {
    type Output = f64;
    fn index(&self, i: usize) -> &f64 {
        &self.0[i]
    }
}

impl ops::IndexMut<usize> for Vec3 {
    fn index_mut(&mut self, i: usize) -> &mut f64 {
        &mut self.0[i]
    }
}

impl ops::Add for Vec3 {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        Self([
            self.0[0] + rhs.0[0],
            self.0[1] + rhs.0[1],
            self.0[2] + rhs.0[2],
        ])
    }
}

impl ops::Sub for Vec3 {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self {
        Self([
            self.0[0] - rhs.0[0],
            self.0[1] - rhs.0[1],
            self.0[2] - rhs.0[2],
        ])
    }
}

impl ops::Neg for Vec3 {
    type Output = Self;
    fn neg(self) -> Self {
        Self([-self.0[0], -self.0[1], -self.0[2]])
    }
}

impl ops::Mul<f64> for Vec3 {
    type Output = Self;
    fn mul(self, s: f64) -> Self {
        Self([self.0[0] * s, self.0[1] * s, self.0[2] * s])
    }
}

impl ops::Div<f64> for Vec3 {
    type Output = Self;
    fn div(self, s: f64) -> Self {
        Self([self.0[0] / s, self.0[1] / s, self.0[2] / s])
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use super::*;

    #[test]
    fn vec3_dot() {
        let a = Vec3::new(1.0, 2.0, 3.0);
        let b = Vec3::new(4.0, 5.0, 6.0);
        assert!((a.dot(b) - 32.0).abs() < 1e-10);
    }

    #[test]
    fn vec3_cross() {
        let x = Vec3::new(1.0, 0.0, 0.0);
        let y = Vec3::new(0.0, 1.0, 0.0);
        let z = x.cross(y);
        assert!((z[0]).abs() < 1e-10);
        assert!((z[1]).abs() < 1e-10);
        assert!((z[2] - 1.0).abs() < 1e-10);
    }

    #[test]
    fn vec3_outer() {
        let a = Vec3::new(1.0, 2.0, 3.0);
        let b = Vec3::new(4.0, 5.0, 6.0);
        let m = a.outer(b);
        assert!((m.0[0][0] - 4.0).abs() < 1e-10);
        assert!((m.0[1][0] - 8.0).abs() < 1e-10);
        assert!((m.0[2][2] - 18.0).abs() < 1e-10);
    }

    #[test]
    fn vec3_norm() {
        let v = Vec3::new(3.0, 4.0, 0.0);
        assert!((v.norm() - 5.0).abs() < 1e-10);
    }

    #[test]
    fn vec3_normalized() {
        let v = Vec3::new(3.0, 4.0, 0.0);
        let n = v.normalized();
        assert!((n.norm() - 1.0).abs() < 1e-10);
        assert!((n[0] - 0.6).abs() < 1e-10);
        assert!((n[1] - 0.8).abs() < 1e-10);
    }

    #[test]
    fn vec3_normalized_zero() {
        let v = Vec3::new(0.0, 0.0, 0.0);
        let n = v.normalized();
        assert_eq!(n, Vec3::new(0.0, 0.0, 0.0));
    }

    #[test]
    fn vec3_add_sub() {
        let a = Vec3::new(1.0, 2.0, 3.0);
        let b = Vec3::new(4.0, 5.0, 6.0);
        let sum = a + b;
        assert!((sum[0] - 5.0).abs() < 1e-10);
        let diff = a - b;
        assert!((diff[0] - -3.0).abs() < 1e-10);
    }

    #[test]
    fn vec3_neg() {
        let v = Vec3::new(1.0, -2.0, 3.0);
        let n = -v;
        assert!((n[0] - -1.0).abs() < 1e-10);
        assert!((n[1] - 2.0).abs() < 1e-10);
    }

    #[test]
    fn vec3_index_mut() {
        let mut v = Vec3::new(1.0, 2.0, 3.0);
        v[1] = 10.0;
        assert!((v[1] - 10.0).abs() < 1e-10);
    }

    #[test]
    fn vec3_mul_div() {
        let v = Vec3::new(2.0, 4.0, 6.0);
        let scaled = v * 3.0;
        assert!((scaled[0] - 6.0).abs() < 1e-10);
        let divided = v / 2.0;
        assert!((divided[2] - 3.0).abs() < 1e-10);
    }
}
