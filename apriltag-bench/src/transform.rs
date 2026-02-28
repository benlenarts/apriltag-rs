/// Geometric transforms for placing tags in a scene.
///
/// All transforms map from **tag-space** coordinates to **image-space** coordinates.
/// In tag-space, the tag occupies [-1, 1] × [-1, 1] with corners at
/// (-1,-1), (1,-1), (1,1), (-1,1) (top-left, top-right, bottom-right, bottom-left).
use serde::{Deserialize, Serialize};

/// A geometric transform mapping tag-space → image-space.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum Transform {
    /// Similarity transform: translate, scale, rotate (no perspective).
    Similarity {
        /// Center of the tag in image coordinates.
        cx: f64,
        cy: f64,
        /// Scale in pixels per tag unit (half the tag size in pixels).
        scale: f64,
        /// In-plane rotation in radians.
        theta: f64,
    },

    /// Full projective transform via a 3×3 homography (row-major).
    Perspective {
        /// Homography matrix entries [h00, h01, h02, h10, h11, h12, h20, h21, h22].
        h: [f64; 9],
    },

    /// Ergonomic pose-based placement: "put a tag here with this tilt."
    FromPose {
        /// Center of the tag in image coordinates.
        center: [f64; 2],
        /// Tag size in pixels (full width of the tag square).
        size: f64,
        /// In-plane rotation in radians.
        roll: f64,
        /// Perspective tilt around the vertical axis (left-right lean), radians.
        tilt_x: f64,
        /// Perspective tilt around the horizontal axis (top-bottom lean), radians.
        tilt_y: f64,
    },
}

impl Transform {
    /// Project a point from tag-space to image-space.
    ///
    /// Tag-space: the tag occupies [-1, 1] × [-1, 1].
    pub fn project(&self, tx: f64, ty: f64) -> (f64, f64) {
        match self {
            Transform::Similarity {
                cx,
                cy,
                scale,
                theta,
            } => {
                let cos = theta.cos();
                let sin = theta.sin();
                let ix = cx + scale * (cos * tx - sin * ty);
                let iy = cy + scale * (sin * tx + cos * ty);
                (ix, iy)
            }
            Transform::Perspective { h } => {
                let w = h[6] * tx + h[7] * ty + h[8];
                let ix = (h[0] * tx + h[1] * ty + h[2]) / w;
                let iy = (h[3] * tx + h[4] * ty + h[5]) / w;
                (ix, iy)
            }
            Transform::FromPose {
                center,
                size,
                roll,
                tilt_x,
                tilt_y,
            } => {
                let h = from_pose_homography(center, *size, *roll, *tilt_x, *tilt_y);
                let w = h[6] * tx + h[7] * ty + h[8];
                let ix = (h[0] * tx + h[1] * ty + h[2]) / w;
                let iy = (h[3] * tx + h[4] * ty + h[5]) / w;
                (ix, iy)
            }
        }
    }

    /// Compute the ground-truth corner positions in image-space.
    ///
    /// Returns corners in order: top-left, top-right, bottom-right, bottom-left,
    /// matching the AprilTag detection convention.
    pub fn ground_truth_corners(&self) -> [[f64; 2]; 4] {
        // Tag-space corners: TL(-1,-1), TR(1,-1), BR(1,1), BL(-1,1)
        let corners_tag = [[-1.0, -1.0], [1.0, -1.0], [1.0, 1.0], [-1.0, 1.0]];
        let mut corners_img = [[0.0; 2]; 4];
        for (i, [tx, ty]) in corners_tag.iter().enumerate() {
            let (ix, iy) = self.project(*tx, *ty);
            corners_img[i] = [ix, iy];
        }
        corners_img
    }
}

/// Build a 3×3 homography from an ergonomic pose specification.
///
/// The homography maps tag-space [-1,1]² to image-space, simulating a camera
/// looking at a planar tag with the given center, size, roll, and tilt angles.
fn from_pose_homography(
    center: &[f64; 2],
    size: f64,
    roll: f64,
    tilt_x: f64,
    tilt_y: f64,
) -> [f64; 9] {
    let half = size / 2.0;

    // Rotation matrices: Rz(roll) * Ry(tilt_x) * Rx(tilt_y)
    let cr = roll.cos();
    let sr = roll.sin();
    let cx = tilt_x.cos();
    let sx = tilt_x.sin();
    let cy = tilt_y.cos();
    let sy = tilt_y.sin();

    // Combined rotation R = Rz * Ry * Rx (only first two columns matter for planar tag)
    // Column 0 of R:
    let r00 = cr * cx;
    let r10 = sr * cx;
    let r20 = -sx;

    // Column 1 of R:
    let r01 = cr * sx * sy - sr * cy;
    let r11 = sr * sx * sy + cr * cy;
    let r21 = cx * sy;

    // The homography maps tag [-1,1]² to image space.
    // H = [half*r0 | half*r1 | center]
    // where r0, r1 are the first two columns of R scaled by half-size.
    //
    // For a fronto-parallel tag (all tilts = 0, roll = 0):
    //   H = [[half, 0, cx], [0, half, cy], [0, 0, 1]]
    //
    // With perspective (non-zero tilt), the third row gets non-zero entries
    // from the rotation, creating foreshortening.
    //
    // We use a virtual focal length proportional to the tag size to control
    // how strongly the tilt manifests as perspective distortion.
    let f = size * 2.0; // focal length in pixels

    let h00 = half * r00;
    let h01 = half * r01;
    let h02 = center[0];

    let h10 = half * r10;
    let h11 = half * r11;
    let h12 = center[1];

    let h20 = half * r20 / f;
    let h21 = half * r21 / f;
    let h22 = 1.0;

    [h00, h01, h02, h10, h11, h12, h20, h21, h22]
}

#[cfg(test)]
mod tests {
    use super::*;

    const EPSILON: f64 = 1e-10;

    fn approx_eq(a: f64, b: f64) -> bool {
        (a - b).abs() < EPSILON
    }

    fn assert_point_approx(actual: (f64, f64), expected: (f64, f64), msg: &str) {
        assert!(
            approx_eq(actual.0, expected.0) && approx_eq(actual.1, expected.1),
            "{msg}: expected ({}, {}), got ({}, {})",
            expected.0,
            expected.1,
            actual.0,
            actual.1,
        );
    }

    #[test]
    fn similarity_identity() {
        // Center at (100, 100), scale=50, no rotation → tag corners at (50,50)..(150,150)
        let t = Transform::Similarity {
            cx: 100.0,
            cy: 100.0,
            scale: 50.0,
            theta: 0.0,
        };

        assert_point_approx(t.project(-1.0, -1.0), (50.0, 50.0), "TL");
        assert_point_approx(t.project(1.0, -1.0), (150.0, 50.0), "TR");
        assert_point_approx(t.project(1.0, 1.0), (150.0, 150.0), "BR");
        assert_point_approx(t.project(-1.0, 1.0), (50.0, 150.0), "BL");
        assert_point_approx(t.project(0.0, 0.0), (100.0, 100.0), "center");
    }

    #[test]
    fn similarity_rotation_90() {
        // 90 degree rotation: (1,0) in tag-space → (0,1)*scale relative to center
        let t = Transform::Similarity {
            cx: 100.0,
            cy: 100.0,
            scale: 50.0,
            theta: std::f64::consts::FRAC_PI_2,
        };

        // After 90° rotation: TL(-1,-1) → center + scale*(cos90*(-1)-sin90*(-1), sin90*(-1)+cos90*(-1))
        //                                = center + scale*(0+1, -1+0) = center + (50, -50) = (150, 50)
        assert_point_approx(t.project(-1.0, -1.0), (150.0, 50.0), "TL rotated 90°");
        // TR(1,-1) → center + scale*(0+1, 1+0) = center + (50, 50) = (150, 150)
        assert_point_approx(t.project(1.0, -1.0), (150.0, 150.0), "TR rotated 90°");
    }

    #[test]
    fn similarity_ground_truth_corners() {
        let t = Transform::Similarity {
            cx: 200.0,
            cy: 200.0,
            scale: 30.0,
            theta: 0.0,
        };
        let corners = t.ground_truth_corners();

        assert!(approx_eq(corners[0][0], 170.0) && approx_eq(corners[0][1], 170.0), "TL");
        assert!(approx_eq(corners[1][0], 230.0) && approx_eq(corners[1][1], 170.0), "TR");
        assert!(approx_eq(corners[2][0], 230.0) && approx_eq(corners[2][1], 230.0), "BR");
        assert!(approx_eq(corners[3][0], 170.0) && approx_eq(corners[3][1], 230.0), "BL");
    }

    #[test]
    fn perspective_identity_homography() {
        // Identity homography scaled and translated: same as similarity with no rotation
        let t = Transform::Perspective {
            h: [50.0, 0.0, 100.0, 0.0, 50.0, 100.0, 0.0, 0.0, 1.0],
        };
        assert_point_approx(t.project(-1.0, -1.0), (50.0, 50.0), "TL");
        assert_point_approx(t.project(1.0, 1.0), (150.0, 150.0), "BR");
        assert_point_approx(t.project(0.0, 0.0), (100.0, 100.0), "center");
    }

    #[test]
    fn from_pose_no_tilt_no_roll() {
        // No tilt, no roll → should behave like a similarity transform
        let t = Transform::FromPose {
            center: [100.0, 100.0],
            size: 100.0,
            roll: 0.0,
            tilt_x: 0.0,
            tilt_y: 0.0,
        };

        let corners = t.ground_truth_corners();
        // half = 50, R = identity, H = [[50, 0, 100], [0, 50, 100], [0, 0, 1]]
        assert!(approx_eq(corners[0][0], 50.0) && approx_eq(corners[0][1], 50.0), "TL");
        assert!(approx_eq(corners[1][0], 150.0) && approx_eq(corners[1][1], 50.0), "TR");
        assert!(approx_eq(corners[2][0], 150.0) && approx_eq(corners[2][1], 150.0), "BR");
        assert!(approx_eq(corners[3][0], 50.0) && approx_eq(corners[3][1], 150.0), "BL");
    }

    #[test]
    fn from_pose_center_preserved() {
        // With any tilt, the center should still map to the specified center
        let t = Transform::FromPose {
            center: [320.0, 240.0],
            size: 80.0,
            roll: 0.3,
            tilt_x: 0.5,
            tilt_y: 0.3,
        };

        let (cx, cy) = t.project(0.0, 0.0);
        assert!(
            approx_eq(cx, 320.0) && approx_eq(cy, 240.0),
            "center should map to ({cx}, {cy}), expected (320, 240)"
        );
    }

    #[test]
    fn from_pose_tilt_creates_perspective() {
        // Tilting around X should make left side shorter and right side taller (or vice versa)
        let t = Transform::FromPose {
            center: [200.0, 200.0],
            size: 100.0,
            roll: 0.0,
            tilt_x: 0.4, // tilt around vertical axis
            tilt_y: 0.0,
        };

        let corners = t.ground_truth_corners();

        // With tilt_x > 0, one side of the tag should appear closer (wider) than the other
        let left_height = (corners[3][1] - corners[0][1]).abs();
        let right_height = (corners[2][1] - corners[1][1]).abs();

        // The two sides should differ (perspective effect)
        assert!(
            (left_height - right_height).abs() > 0.1,
            "tilt should create perspective: left_h={left_height}, right_h={right_height}"
        );
    }

    #[test]
    fn from_pose_roll_rotates_corners() {
        // Roll of π/2 should rotate corners like similarity rotation
        let t_no_roll = Transform::FromPose {
            center: [200.0, 200.0],
            size: 80.0,
            roll: 0.0,
            tilt_x: 0.0,
            tilt_y: 0.0,
        };
        let t_rolled = Transform::FromPose {
            center: [200.0, 200.0],
            size: 80.0,
            roll: std::f64::consts::FRAC_PI_2,
            tilt_x: 0.0,
            tilt_y: 0.0,
        };

        let c0 = t_no_roll.ground_truth_corners();
        let c1 = t_rolled.ground_truth_corners();

        // After 90° roll, TL should go where TR was (approximately)
        // TL(-1,-1) with roll=0: (160,160). With roll=90°: should be (240,160)
        assert!(
            approx_eq(c1[0][0], c0[1][0]) && approx_eq(c1[0][1], c0[1][1]),
            "TL after 90° roll should be at original TR position"
        );
    }

    #[test]
    fn ground_truth_corners_matches_project() {
        let t = Transform::FromPose {
            center: [300.0, 250.0],
            size: 120.0,
            roll: 0.7,
            tilt_x: 0.3,
            tilt_y: -0.2,
        };

        let corners = t.ground_truth_corners();
        let tag_corners = [[-1.0, -1.0], [1.0, -1.0], [1.0, 1.0], [-1.0, 1.0]];

        for (i, [tx, ty]) in tag_corners.iter().enumerate() {
            let (ix, iy) = t.project(*tx, *ty);
            assert!(
                approx_eq(corners[i][0], ix) && approx_eq(corners[i][1], iy),
                "corner {i}: ground_truth_corners() ({}, {}) != project() ({ix}, {iy})",
                corners[i][0],
                corners[i][1]
            );
        }
    }
}
