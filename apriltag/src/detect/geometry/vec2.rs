use core::ops;

/// A 2D vector, wrapping `[f64; 2]`.
///
/// Zero-cost abstraction over `[f64; 2]` via `#[repr(transparent)]`.
#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(transparent)]
pub struct Vec2(pub [f64; 2]);

impl Vec2 {
    /// Create a new `Vec2`.
    pub fn new(x: f64, y: f64) -> Self {
        Self([x, y])
    }

    /// Dot product.
    pub fn dot(self, rhs: Self) -> f64 {
        self.0[0] * rhs.0[0] + self.0[1] * rhs.0[1]
    }

    /// Euclidean norm.
    pub fn norm(self) -> f64 {
        self.dot(self).sqrt()
    }

    /// Unit vector in the same direction, or zero vector if norm is near zero.
    pub fn normalized(self) -> Self {
        let n = self.norm();
        if n < 1e-15 {
            Self([0.0; 2])
        } else {
            self / n
        }
    }
}

impl ops::Index<usize> for Vec2 {
    type Output = f64;
    fn index(&self, i: usize) -> &f64 {
        &self.0[i]
    }
}

impl ops::IndexMut<usize> for Vec2 {
    fn index_mut(&mut self, i: usize) -> &mut f64 {
        &mut self.0[i]
    }
}

impl ops::Add for Vec2 {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        Self([self.0[0] + rhs.0[0], self.0[1] + rhs.0[1]])
    }
}

impl ops::Sub for Vec2 {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self {
        Self([self.0[0] - rhs.0[0], self.0[1] - rhs.0[1]])
    }
}

impl ops::Neg for Vec2 {
    type Output = Self;
    fn neg(self) -> Self {
        Self([-self.0[0], -self.0[1]])
    }
}

impl ops::Mul<f64> for Vec2 {
    type Output = Self;
    fn mul(self, s: f64) -> Self {
        Self([self.0[0] * s, self.0[1] * s])
    }
}

impl ops::Div<f64> for Vec2 {
    type Output = Self;
    fn div(self, s: f64) -> Self {
        Self([self.0[0] / s, self.0[1] / s])
    }
}

impl From<[f64; 2]> for Vec2 {
    fn from(a: [f64; 2]) -> Self {
        Self(a)
    }
}

impl From<Vec2> for [f64; 2] {
    fn from(v: Vec2) -> Self {
        v.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn vec2_new() {
        let v = Vec2::new(1.0, 2.0);
        assert!((v[0] - 1.0).abs() < 1e-10);
        assert!((v[1] - 2.0).abs() < 1e-10);
    }

    #[test]
    fn vec2_dot() {
        let a = Vec2::new(3.0, 4.0);
        let b = Vec2::new(1.0, 2.0);
        assert!((a.dot(b) - 11.0).abs() < 1e-10);
    }

    #[test]
    fn vec2_norm() {
        let v = Vec2::new(3.0, 4.0);
        assert!((v.norm() - 5.0).abs() < 1e-10);
    }

    #[test]
    fn vec2_normalized() {
        let v = Vec2::new(3.0, 4.0);
        let n = v.normalized();
        assert!((n.norm() - 1.0).abs() < 1e-10);
        assert!((n[0] - 0.6).abs() < 1e-10);
        assert!((n[1] - 0.8).abs() < 1e-10);
    }

    #[test]
    fn vec2_normalized_zero() {
        let v = Vec2::new(0.0, 0.0);
        let n = v.normalized();
        assert_eq!(n, Vec2::new(0.0, 0.0));
    }

    #[test]
    fn vec2_add_sub() {
        let a = Vec2::new(1.0, 2.0);
        let b = Vec2::new(3.0, 4.0);
        let sum = a + b;
        assert!((sum[0] - 4.0).abs() < 1e-10);
        assert!((sum[1] - 6.0).abs() < 1e-10);
        let diff = a - b;
        assert!((diff[0] - -2.0).abs() < 1e-10);
        assert!((diff[1] - -2.0).abs() < 1e-10);
    }

    #[test]
    fn vec2_neg() {
        let v = Vec2::new(1.0, -2.0);
        let n = -v;
        assert!((n[0] - -1.0).abs() < 1e-10);
        assert!((n[1] - 2.0).abs() < 1e-10);
    }

    #[test]
    fn vec2_index_mut() {
        let mut v = Vec2::new(1.0, 2.0);
        v[1] = 10.0;
        assert!((v[1] - 10.0).abs() < 1e-10);
    }

    #[test]
    fn vec2_mul_div() {
        let v = Vec2::new(2.0, 4.0);
        let scaled = v * 3.0;
        assert!((scaled[0] - 6.0).abs() < 1e-10);
        assert!((scaled[1] - 12.0).abs() < 1e-10);
        let divided = v / 2.0;
        assert!((divided[0] - 1.0).abs() < 1e-10);
        assert!((divided[1] - 2.0).abs() < 1e-10);
    }

    #[test]
    fn vec2_from_array() {
        let v: Vec2 = [1.0, 2.0].into();
        assert_eq!(v, Vec2::new(1.0, 2.0));
    }

    #[test]
    fn vec2_into_array() {
        let v = Vec2::new(1.0, 2.0);
        let a: [f64; 2] = v.into();
        assert_eq!(a, [1.0, 2.0]);
    }
}
