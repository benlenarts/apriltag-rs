use super::geometry::{forward_eliminate, Mat3, Vec2};

/// A 3x3 homography matrix.
#[derive(Debug, Clone, Copy)]
pub struct Homography {
    pub data: Mat3,
}

impl Homography {
    /// Compute homography from 4 tag-space correspondences to pixel coordinates.
    ///
    /// Tag-space corners: (-1,-1), (1,-1), (1,1), (-1,1)
    /// Pixel corners: the 4 quad corners.
    pub fn from_quad_corners(corners: &[Vec2; 4]) -> Option<Self> {
        // tag coords in order
        let tag_pts = [[-1.0, -1.0], [1.0, -1.0], [1.0, 1.0], [-1.0, 1.0]];

        // Build 8x9 DLT system
        let mut a = [[0.0f64; 9]; 8];
        for i in 0..4 {
            let (tx, ty) = (tag_pts[i][0], tag_pts[i][1]);
            let (px, py) = (corners[i][0], corners[i][1]);

            let row0 = i * 2;
            a[row0][0] = tx;
            a[row0][1] = ty;
            a[row0][2] = 1.0;
            a[row0][6] = -tx * px;
            a[row0][7] = -ty * px;
            a[row0][8] = px;

            let row1 = i * 2 + 1;
            a[row1][3] = tx;
            a[row1][4] = ty;
            a[row1][5] = 1.0;
            a[row1][6] = -tx * py;
            a[row1][7] = -ty * py;
            a[row1][8] = py;
        }

        // Gaussian elimination with partial pivoting (8x9 augmented)
        forward_eliminate::<8, 9>(&mut a, 1e-10)?;

        // Back-substitute (h[8] = 1, solve for h[0..8])
        let mut h = [0.0f64; 9];
        h[8] = 1.0;

        for row in (0..8).rev() {
            let mut sum = a[row][8]; // the "= px" or "= py" column
            for c in (row + 1)..8 {
                sum -= a[row][c] * h[c];
            }
            h[row] = sum / a[row][row];
        }

        Some(Homography {
            data: Mat3([[h[0], h[1], h[2]], [h[3], h[4], h[5]], [h[6], h[7], h[8]]]),
        })
    }

    /// Project a point from tag-space to pixel-space.
    pub fn project(&self, x: f64, y: f64) -> (f64, f64) {
        let h = &self.data.0;
        let xx = h[0][0] * x + h[0][1] * y + h[0][2];
        let yy = h[1][0] * x + h[1][1] * y + h[1][2];
        let zz = h[2][0] * x + h[2][1] * y + h[2][2];
        (xx / zz, yy / zz)
    }

    /// Compute the inverse homography.
    pub fn inverse(&self) -> Option<Self> {
        self.data.inv().map(|data| Homography { data })
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use super::*;

    fn v(corners: [[f64; 2]; 4]) -> [Vec2; 4] {
        corners.map(Vec2::from)
    }

    #[test]
    fn identity_homography_unit_square() {
        let corners = v([[-1.0, -1.0], [1.0, -1.0], [1.0, 1.0], [-1.0, 1.0]]);
        let h = Homography::from_quad_corners(&corners).unwrap();
        let (px, py) = h.project(0.0, 0.0);
        assert!((px - 0.0).abs() < 1e-6);
        assert!((py - 0.0).abs() < 1e-6);

        let (px, py) = h.project(1.0, 1.0);
        assert!((px - 1.0).abs() < 1e-6);
        assert!((py - 1.0).abs() < 1e-6);
    }

    #[test]
    fn scaling_homography() {
        let corners = v([[0.0, 0.0], [100.0, 0.0], [100.0, 100.0], [0.0, 100.0]]);
        let h = Homography::from_quad_corners(&corners).unwrap();
        let (px, py) = h.project(0.0, 0.0);
        assert!((px - 50.0).abs() < 1e-6);
        assert!((py - 50.0).abs() < 1e-6);

        let (px, py) = h.project(-1.0, -1.0);
        assert!((px - 0.0).abs() < 1e-6);
        assert!((py - 0.0).abs() < 1e-6);
    }

    #[test]
    fn project_all_corners_match() {
        let corners = v([[10.0, 20.0], [90.0, 15.0], [95.0, 85.0], [5.0, 90.0]]);
        let h = Homography::from_quad_corners(&corners).unwrap();

        let tag_pts = [[-1.0, -1.0], [1.0, -1.0], [1.0, 1.0], [-1.0, 1.0]];
        for i in 0..4 {
            let (px, py) = h.project(tag_pts[i][0], tag_pts[i][1]);
            assert!((px - corners[i][0]).abs() < 1e-4 && (py - corners[i][1]).abs() < 1e-4);
        }
    }

    #[test]
    fn inverse_roundtrip() {
        let corners = v([[10.0, 20.0], [90.0, 15.0], [95.0, 85.0], [5.0, 90.0]]);
        let h = Homography::from_quad_corners(&corners).unwrap();
        let hinv = h.inverse().unwrap();

        let (px, py) = h.project(0.5, -0.3);
        let (tx, ty) = hinv.project(px, py);
        assert!((tx - 0.5).abs() < 1e-6, "tx={tx}");
        assert!((ty - (-0.3)).abs() < 1e-6, "ty={ty}");
    }

    #[test]
    fn degenerate_returns_none() {
        let corners = v([[5.0, 5.0], [5.0, 5.0], [5.0, 5.0], [5.0, 5.0]]);
        assert!(Homography::from_quad_corners(&corners).is_none());
    }

    #[test]
    fn inverse_singular_matrix() {
        let h = Homography {
            data: Mat3([[1.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]),
        };
        assert!(h.inverse().is_none());
    }
}
