/// Scene composition: place rendered tags into an image with ground truth.
use apriltag::detect::geometry::{Mat3, Vec3};
use apriltag::detect::pose::PoseParams;
use apriltag::family;
use apriltag::render::RenderedTag;
use apriltag::types::Pixel;
use apriltag::ImageU8;
use serde::{Deserialize, Serialize};

use crate::transform::Transform;

/// A tag placed in a scene with its ground-truth corner positions.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PlacedTag {
    pub family_name: String,
    pub tag_id: u32,
    /// Ground-truth corners in image-space: [TL, TR, BR, BL].
    pub corners: [[f64; 2]; 4],
    /// Ground-truth center in image-space.
    pub center: [f64; 2],
    /// Ground-truth rotation matrix, if known.
    #[serde(default)]
    pub gt_rotation: Option<Mat3>,
    /// Ground-truth translation vector, if known.
    #[serde(default)]
    pub gt_translation: Option<Vec3>,
    /// Camera intrinsics + tag size used to generate the ground truth.
    #[serde(default)]
    pub gt_pose_params: Option<PoseParams>,
}

/// A complete scene: image + ground truth.
#[derive(Debug, Clone)]
pub struct Scene {
    pub image: ImageU8,
    pub ground_truth: Vec<PlacedTag>,
}

/// Background fill for the scene.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum Background {
    /// Uniform gray value.
    Solid(u8),
    /// Vertical gradient from top to bottom.
    Gradient { top: u8, bottom: u8 },
    /// Checkerboard pattern.
    Checkerboard { cell_size: u32, light: u8, dark: u8 },
}

/// A tag to be placed in the scene.
struct TagPlacement {
    family_name: String,
    tag_id: u32,
    transform: Transform,
}

/// Builder for constructing scenes.
pub struct SceneBuilder {
    width: u32,
    height: u32,
    background: Background,
    tags: Vec<TagPlacement>,
}

impl SceneBuilder {
    pub fn new(width: u32, height: u32) -> Self {
        Self {
            width,
            height,
            background: Background::Solid(128),
            tags: Vec::new(),
        }
    }

    pub fn background(mut self, bg: Background) -> Self {
        self.background = bg;
        self
    }

    pub fn add_tag(mut self, family_name: &str, tag_id: u32, transform: Transform) -> Self {
        self.tags.push(TagPlacement {
            family_name: family_name.to_string(),
            tag_id,
            transform,
        });
        self
    }

    /// Build the scene: render tags, composite onto background, compute ground truth.
    pub fn build(self) -> Scene {
        let mut image = fill_background(self.width, self.height, &self.background);
        let mut ground_truth = Vec::new();

        for placement in &self.tags {
            let fam = family::builtin_family(&placement.family_name)
                .unwrap_or_else(|| panic!("unknown tag family: {}", placement.family_name));

            let rendered = fam.tag(placement.tag_id as usize).render();

            composite_tag(
                &mut image,
                &rendered,
                &placement.transform,
                fam.layout.border_start,
                fam.layout.border_width,
            );

            let corners = placement.transform.ground_truth_corners();
            let (cx, cy) = placement.transform.project(0.0, 0.0);

            let (gt_rotation, gt_translation, gt_pose_params) = if let Transform::FromPose {
                center,
                size,
                roll,
                tilt_x,
                tilt_y,
            } = &placement.transform
            {
                let f = size * 2.0;

                // Build rotation R_bench = Rz(roll) * Ry(tilt_x) * Rx(tilt_y)
                let cr = roll.cos();
                let sr = roll.sin();
                let cxx = tilt_x.cos();
                let sxx = tilt_x.sin();
                let cyy = tilt_y.cos();
                let syy = tilt_y.sin();

                // R_bench columns
                let r0 = Vec3::new(cr * cxx, sr * cxx, -sxx);
                let r1_bench = Vec3::new(
                    cr * sxx * syy - sr * cyy,
                    sr * sxx * syy + cr * cyy,
                    cxx * syy,
                );

                // Account for y-axis flip: pose estimator negates homography
                // column 1 (r1 = -c1), so ground-truth R_gt = R_bench * diag(1,-1,1)
                // then r2_gt = r0_gt × r1_gt to ensure proper rotation.
                let r1 = -r1_bench;
                let r2 = r0.cross(r1);

                let rotation = Mat3([
                    [r0[0], r1[0], r2[0]],
                    [r0[1], r1[1], r2[1]],
                    [r0[2], r1[2], r2[2]],
                ]);
                // The pose estimator works in tag-size units: t = c2 * (tagsize/2).
                // Our virtual camera puts the tag at z=f pixels from center.
                // The estimator sees columns scaled by half/f, so t_z = f/(half) * (tagsize/2).
                // With tagsize=2, half=size/2: t_z = f/half = (size*2)/(size/2) = 4.
                let tagsize = 2.0;
                let half = size / 2.0;
                let translation = Vec3::new(0.0, 0.0, f / half * (tagsize / 2.0));

                let pose_params = PoseParams {
                    tagsize: 2.0,
                    fx: f,
                    fy: f,
                    cx: center[0],
                    cy: center[1],
                };

                (Some(rotation), Some(translation), Some(pose_params))
            } else {
                (None, None, None)
            };

            ground_truth.push(PlacedTag {
                family_name: placement.family_name.clone(),
                tag_id: placement.tag_id,
                corners,
                center: [cx, cy],
                gt_rotation,
                gt_translation,
                gt_pose_params,
            });
        }

        Scene {
            image,
            ground_truth,
        }
    }
}

/// Fill an image with the given background pattern.
fn fill_background(width: u32, height: u32, bg: &Background) -> ImageU8 {
    let mut img = ImageU8::new(width, height);
    match bg {
        Background::Solid(v) => {
            for y in 0..height {
                for x in 0..width {
                    img.set(x, y, *v);
                }
            }
        }
        Background::Gradient { top, bottom } => {
            for y in 0..height {
                let t = if height > 1 {
                    y as f64 / (height - 1) as f64
                } else {
                    0.0
                };
                let v = (*top as f64 * (1.0 - t) + *bottom as f64 * t).round() as u8;
                for x in 0..width {
                    img.set(x, y, v);
                }
            }
        }
        Background::Checkerboard {
            cell_size,
            light,
            dark,
        } => {
            for y in 0..height {
                for x in 0..width {
                    let cell_x = x / cell_size;
                    let cell_y = y / cell_size;
                    let v = if (cell_x + cell_y) % 2 == 0 {
                        *light
                    } else {
                        *dark
                    };
                    img.set(x, y, v);
                }
            }
        }
    }
    img
}

/// Composite a rendered tag onto an image using the given transform.
///
/// Uses inverse mapping: for each output pixel, compute the corresponding
/// tag-space coordinate and sample the rendered tag.
///
/// Tag-space convention: [-1, 1] maps to the border region
/// [border_start, grid_size - border_start], matching the detector's homography.
/// The white border extends beyond [-1, 1].
fn composite_tag(
    img: &mut ImageU8,
    tag: &RenderedTag,
    transform: &Transform,
    border_start: usize,
    border_width: usize,
) {
    let grid = tag.grid_size as f64;
    let bs = border_start as f64;
    let bw = border_width as f64;

    // The white border extends beyond tag-space [-1, 1].
    // Grid position 0 → tag-space = 2*(0-bs)/bw - 1 = -(2*bs/bw + 1)
    // Grid position grid_size → tag-space = 2*(grid_size-bs)/bw - 1 = (2*bs/bw + 1)
    let tag_extent = 2.0 * bs / bw + 1.0;

    // Compute bounding box using the extended corners
    let ext_corners = [
        [-tag_extent, -tag_extent],
        [tag_extent, -tag_extent],
        [tag_extent, tag_extent],
        [-tag_extent, tag_extent],
    ];
    let mut min_x = f64::INFINITY;
    let mut max_x = f64::NEG_INFINITY;
    let mut min_y = f64::INFINITY;
    let mut max_y = f64::NEG_INFINITY;
    for [tx, ty] in &ext_corners {
        let (ix, iy) = transform.project(*tx, *ty);
        min_x = min_x.min(ix);
        max_x = max_x.max(ix);
        min_y = min_y.min(iy);
        max_y = max_y.max(iy);
    }

    let x0 = (min_x - 1.0).max(0.0) as u32;
    let x1 = ((max_x + 2.0) as u32).min(img.width);
    let y0 = (min_y - 1.0).max(0.0) as u32;
    let y1 = ((max_y + 2.0) as u32).min(img.height);

    let inv = inverse_homography(transform);

    for iy in y0..y1 {
        for ix in x0..x1 {
            let px = ix as f64 + 0.5;
            let py = iy as f64 + 0.5;

            let w = inv[6] * px + inv[7] * py + inv[8];
            // COVERAGE: degenerate homography pixel — requires near-singular perspective
            if w.abs() < 1e-12 {
                continue;
            }
            let tx = (inv[0] * px + inv[1] * py + inv[2]) / w;
            let ty = (inv[3] * px + inv[4] * py + inv[5]) / w;

            // Tag-space → grid-space:
            // tag-space [-1, 1] maps to grid [border_start, grid_size - border_start]
            let gx = bs + (tx + 1.0) * 0.5 * bw;
            let gy = bs + (ty + 1.0) * 0.5 * bw;

            if gx < 0.0 || gx >= grid || gy < 0.0 || gy >= grid {
                continue;
            }

            let cell_x = gx as usize;
            let cell_y = gy as usize;
            let pixel = tag.pixel(cell_x, cell_y);

            match pixel {
                Pixel::Black => img.set(ix, iy, 0),
                Pixel::White => img.set(ix, iy, 255),
                // COVERAGE: only fires with custom families that have transparent cells
                Pixel::Transparent => {} // leave background
            }
        }
    }
}

/// Compute the 3×3 homography matrix for a transform.
fn transform_to_homography(transform: &Transform) -> [f64; 9] {
    match transform {
        Transform::Similarity {
            cx,
            cy,
            scale,
            theta,
        } => {
            let cos = theta.cos();
            let sin = theta.sin();
            [
                scale * cos,
                -scale * sin,
                *cx,
                scale * sin,
                scale * cos,
                *cy,
                0.0,
                0.0,
                1.0,
            ]
        }
        Transform::Perspective { h } => *h,
        Transform::FromPose {
            center,
            size,
            roll,
            tilt_x,
            tilt_y,
        } => {
            // Replicate the logic from transform.rs::from_pose_homography
            let half = size / 2.0;
            let cr = roll.cos();
            let sr = roll.sin();
            let cxx = tilt_x.cos();
            let sxx = tilt_x.sin();
            let cyy = tilt_y.cos();
            let syy = tilt_y.sin();
            let r00 = cr * cxx;
            let r10 = sr * cxx;
            let r20 = -sxx;
            let r01 = cr * sxx * syy - sr * cyy;
            let r11 = sr * sxx * syy + cr * cyy;
            let r21 = cxx * syy;
            let f = size * 2.0;
            [
                half * r00,
                half * r01,
                center[0],
                half * r10,
                half * r11,
                center[1],
                half * r20 / f,
                half * r21 / f,
                1.0,
            ]
        }
    }
}

/// Compute the inverse of a 3×3 homography matrix.
fn inverse_homography(transform: &Transform) -> [f64; 9] {
    let h = transform_to_homography(transform);
    invert_3x3(&h)
}

/// Invert a 3×3 matrix given as a row-major array.
fn invert_3x3(m: &[f64; 9]) -> [f64; 9] {
    let det = m[0] * (m[4] * m[8] - m[5] * m[7]) - m[1] * (m[3] * m[8] - m[5] * m[6])
        + m[2] * (m[3] * m[7] - m[4] * m[6]);

    let inv_det = 1.0 / det;

    [
        (m[4] * m[8] - m[5] * m[7]) * inv_det,
        (m[2] * m[7] - m[1] * m[8]) * inv_det,
        (m[1] * m[5] - m[2] * m[4]) * inv_det,
        (m[5] * m[6] - m[3] * m[8]) * inv_det,
        (m[0] * m[8] - m[2] * m[6]) * inv_det,
        (m[2] * m[3] - m[0] * m[5]) * inv_det,
        (m[3] * m[7] - m[4] * m[6]) * inv_det,
        (m[1] * m[6] - m[0] * m[7]) * inv_det,
        (m[0] * m[4] - m[1] * m[3]) * inv_det,
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn solid_background_fills_image() {
        let img = fill_background(10, 10, &Background::Solid(200));
        for y in 0..10 {
            for x in 0..10 {
                assert_eq!(img.get(x, y), 200);
            }
        }
    }

    #[test]
    fn gradient_background_top_bottom() {
        let img = fill_background(
            5,
            11,
            &Background::Gradient {
                top: 0,
                bottom: 100,
            },
        );
        assert_eq!(img.get(0, 0), 0);
        assert_eq!(img.get(0, 10), 100);
        assert_eq!(img.get(0, 5), 50);
    }

    #[test]
    fn checkerboard_pattern() {
        let img = fill_background(
            10,
            10,
            &Background::Checkerboard {
                cell_size: 5,
                light: 255,
                dark: 0,
            },
        );
        assert_eq!(img.get(0, 0), 255); // cell (0,0) → light
        assert_eq!(img.get(5, 0), 0); // cell (1,0) → dark
        assert_eq!(img.get(0, 5), 0); // cell (0,1) → dark
        assert_eq!(img.get(5, 5), 255); // cell (1,1) → light
    }

    #[test]
    fn invert_identity() {
        let id = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
        let inv = invert_3x3(&id);
        for i in 0..9 {
            assert!((inv[i] - id[i]).abs() < 1e-10);
        }
    }

    #[test]
    fn invert_scale_translate() {
        let m = [50.0, 0.0, 100.0, 0.0, 50.0, 100.0, 0.0, 0.0, 1.0];
        let inv = invert_3x3(&m);
        // inv should map (100, 100) → (0, 0) and (150, 100) → (1, 0)
        let px = 150.0;
        let py = 100.0;
        let tx = inv[0] * px + inv[1] * py + inv[2];
        let ty = inv[3] * px + inv[4] * py + inv[5];
        // inv should map (150, 100) → tag-space (1, 0)
        assert!((tx - 1.0).abs() < 1e-10);
        assert!((ty - 0.0).abs() < 1e-10);
    }

    #[test]
    fn scene_builder_simple_tag() {
        // Place a tag36h11 tag #0 centered in a 200x200 image
        let scene = SceneBuilder::new(200, 200)
            .background(Background::Solid(128))
            .add_tag(
                "tag36h11",
                0,
                Transform::Similarity {
                    cx: 100.0,
                    cy: 100.0,
                    scale: 40.0,
                    theta: 0.0,
                },
            )
            .build();

        assert_eq!(scene.image.width, 200);
        assert_eq!(scene.image.height, 200);
        assert_eq!(scene.ground_truth.len(), 1);

        let gt = &scene.ground_truth[0];
        assert_eq!(gt.family_name, "tag36h11");
        assert_eq!(gt.tag_id, 0);
        assert!((gt.center[0] - 100.0).abs() < 1e-10);
        assert!((gt.center[1] - 100.0).abs() < 1e-10);
    }

    #[test]
    fn scene_tag_pixels_are_placed() {
        // Place a tag and verify that the image has non-background pixels in the tag area.
        // tag36h11: grid_size=10, border_start=1, border_width=8.
        // Scale=40 means tag-space [-1,1] spans 80px → border region is 80px.
        // Full grid extends further (white border adds 10px on each side).
        let scene = SceneBuilder::new(200, 200)
            .background(Background::Solid(128))
            .add_tag(
                "tag36h11",
                0,
                Transform::Similarity {
                    cx: 100.0,
                    cy: 100.0,
                    scale: 40.0,
                    theta: 0.0,
                },
            )
            .build();

        // The center of the tag should be black or white (part of the tag), not 128
        // The center of the tag should be black or white (part of the tag), not background
        let center_val = scene.image.get(100, 100);
        assert!(center_val == 0 || center_val == 255);

        // A corner far from the tag should still be background
        assert_eq!(scene.image.get(0, 0), 128);
        assert_eq!(scene.image.get(199, 199), 128);
    }

    #[test]
    fn scene_tag_has_white_border() {
        // tag36h11: grid_size=10, border_start=1, border_width=8.
        // Scale=40 → tag-space [-1,1] = 80px (border region).
        // Each grid cell in border region = 80/8 = 10px.
        // White border is 1 grid cell outside the border region = 10px extra on each side.
        // Tag-space -1 maps to center-40 = 60 (inner edge of white border).
        // White border extends from 50 to 60 (grid cell 0).
        let scene = SceneBuilder::new(200, 200)
            .background(Background::Solid(128))
            .add_tag(
                "tag36h11",
                0,
                Transform::Similarity {
                    cx: 100.0,
                    cy: 100.0,
                    scale: 40.0,
                    theta: 0.0,
                },
            )
            .build();

        // White border pixel: should be at ~55, inside the white border region
        // Outer border pixel should be white
        assert_eq!(scene.image.get(55, 55), 255);
        // Inner border pixel should be black
        assert_eq!(scene.image.get(65, 65), 0);
    }

    #[test]
    fn scene_multiple_tags() {
        let scene = SceneBuilder::new(400, 200)
            .background(Background::Solid(128))
            .add_tag(
                "tag36h11",
                0,
                Transform::Similarity {
                    cx: 100.0,
                    cy: 100.0,
                    scale: 30.0,
                    theta: 0.0,
                },
            )
            .add_tag(
                "tag36h11",
                1,
                Transform::Similarity {
                    cx: 300.0,
                    cy: 100.0,
                    scale: 30.0,
                    theta: 0.0,
                },
            )
            .build();

        assert_eq!(scene.ground_truth.len(), 2);
        assert_eq!(scene.ground_truth[0].tag_id, 0);
        assert_eq!(scene.ground_truth[1].tag_id, 1);

        // Both tags should have non-background pixels at their centers
        let c0 = scene.image.get(100, 100);
        let c1 = scene.image.get(300, 100);
        assert!(c0 == 0 || c0 == 255);
        assert!(c1 == 0 || c1 == 255);
    }

    #[test]
    fn gradient_background_height_1() {
        // Edge case: height=1 uses the t=0.0 branch
        let img = fill_background(
            5,
            1,
            &Background::Gradient {
                top: 100,
                bottom: 200,
            },
        );
        // With height=1, t=0.0 so all pixels equal top value
        assert_eq!(img.get(0, 0), 100);
    }

    #[test]
    fn from_pose_has_ground_truth() {
        let scene = SceneBuilder::new(500, 500)
            .background(Background::Solid(128))
            .add_tag(
                "tag36h11",
                0,
                Transform::FromPose {
                    center: [250.0, 250.0],
                    size: 100.0,
                    roll: 0.0,
                    tilt_x: 0.0,
                    tilt_y: 0.0,
                },
            )
            .build();

        let gt = &scene.ground_truth[0];

        // FromPose should provide ground-truth pose data
        assert!(gt.gt_rotation.is_some());
        assert!(gt.gt_translation.is_some());
        assert!(gt.gt_pose_params.is_some());

        let r = gt.gt_rotation.unwrap();
        let t = gt.gt_translation.unwrap();
        let pp = gt.gt_pose_params.as_ref().unwrap();

        // For frontal no-tilt tag: R ≈ diag(1, -1, -1)
        // The bench tag-space has y-down but the pose estimator's tag frame has y-up,
        // so the estimator's c1 negation produces R = diag(1, -1, -1) for a frontal view.
        let expected_r = Mat3([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]]);
        for i in 0..3 {
            for j in 0..3 {
                assert!(
                    (r.0[i][j] - expected_r.0[i][j]).abs() < 0.01,
                    "R[{i}][{j}] = {} expected {}",
                    r.0[i][j],
                    expected_r.0[i][j]
                );
            }
        }

        assert!(t[0].abs() < 0.01, "t_x = {} expected ~0", t[0]);
        assert!(t[1].abs() < 0.01, "t_y = {} expected ~0", t[1]);
        // t_z = f/half * (tagsize/2) = 200/50 * 1 = 4.0
        assert!((t[2] - 4.0).abs() < 0.01, "t_z = {} expected ~4.0", t[2]);

        // tagsize should be 2.0 (tag-space [-1,1])
        assert!((pp.tagsize - 2.0).abs() < 1e-10);
        // fx = fy = size * 2.0 = 200
        assert!((pp.fx - 200.0).abs() < 1e-10);
        assert!((pp.fy - 200.0).abs() < 1e-10);
    }

    #[test]
    fn similarity_has_no_ground_truth_pose() {
        let scene = SceneBuilder::new(200, 200)
            .background(Background::Solid(128))
            .add_tag(
                "tag36h11",
                0,
                Transform::Similarity {
                    cx: 100.0,
                    cy: 100.0,
                    scale: 40.0,
                    theta: 0.0,
                },
            )
            .build();

        let gt = &scene.ground_truth[0];
        assert!(gt.gt_rotation.is_none());
        assert!(gt.gt_translation.is_none());
        assert!(gt.gt_pose_params.is_none());
    }

    #[test]
    fn perspective_homography_accessor() {
        // Exercise the Transform::Perspective arm of transform_to_homography
        let h = [50.0, 0.0, 100.0, 0.0, 50.0, 100.0, 0.0, 0.0, 1.0];
        let t = Transform::Perspective { h };
        let inv = inverse_homography(&t);
        // inv of a scale+translate: (100,100) → (0,0)
        let tx = inv[0] * 100.0 + inv[1] * 100.0 + inv[2];
        let ty = inv[3] * 100.0 + inv[4] * 100.0 + inv[5];
        assert!((tx).abs() < 1e-10);
        assert!((ty).abs() < 1e-10);
    }
}
