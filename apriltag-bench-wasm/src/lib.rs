use wasm_bindgen::prelude::*;

use apriltag_bench::distortion::{self, Distortion};
use apriltag_bench::scene::{Background, SceneBuilder};
use apriltag_bench::transform::Transform;

/// Generate a scene with a single tag and return the image data + ground truth.
///
/// Returns a JS object with:
/// - `image`: Uint8Array of grayscale pixel data
/// - `width`: image width
/// - `height`: image height
/// - `ground_truth`: array of placed tags with corners
#[wasm_bindgen(js_name = "generateScene")]
#[allow(clippy::too_many_arguments)]
pub fn generate_scene(
    width: u32,
    height: u32,
    family: &str,
    tag_id: u32,
    tag_size: f64,
    rotation_deg: f64,
    tilt_x_deg: f64,
    tilt_y_deg: f64,
    noise_sigma: f64,
    blur_sigma: f64,
    contrast: f64,
) -> Result<JsValue, JsError> {
    let cx = width as f64 / 2.0;
    let cy = height as f64 / 2.0;

    let transform = if tilt_x_deg.abs() > 0.01 || tilt_y_deg.abs() > 0.01 {
        Transform::FromPose {
            center: [cx, cy],
            size: tag_size,
            roll: rotation_deg.to_radians(),
            tilt_x: tilt_x_deg.to_radians(),
            tilt_y: tilt_y_deg.to_radians(),
        }
    } else {
        Transform::Similarity {
            cx,
            cy,
            scale: tag_size / 2.0,
            theta: rotation_deg.to_radians(),
        }
    };

    let mut scene = SceneBuilder::new(width, height)
        .background(Background::Solid(128))
        .add_tag(family, tag_id, transform)
        .build();

    // Apply distortions
    let mut distortions = Vec::new();
    if contrast != 1.0 {
        distortions.push(Distortion::ContrastScale { factor: contrast });
    }
    if blur_sigma > 0.0 {
        distortions.push(Distortion::GaussianBlur { sigma: blur_sigma });
    }
    if noise_sigma > 0.0 {
        distortions.push(Distortion::GaussianNoise {
            sigma: noise_sigma,
            seed: 42,
        });
    }
    if !distortions.is_empty() {
        distortion::apply(&mut scene.image, &distortions);
    }

    // Build result object
    let result = SceneResult {
        width: scene.image.width,
        height: scene.image.height,
        stride: scene.image.stride,
        image_data: scene.image.buf,
        ground_truth: scene
            .ground_truth
            .iter()
            .map(|gt| GroundTruthTag {
                family_name: gt.family_name.clone(),
                tag_id: gt.tag_id,
                corners: gt.corners,
                center: gt.center,
            })
            .collect(),
    };

    serde_wasm_bindgen::to_value(&result).map_err(|e| JsError::new(&e.to_string()))
}

#[derive(serde::Serialize)]
struct SceneResult {
    width: u32,
    height: u32,
    stride: u32,
    #[serde(rename = "imageData")]
    image_data: Vec<u8>,
    #[serde(rename = "groundTruth")]
    ground_truth: Vec<GroundTruthTag>,
}

#[derive(serde::Serialize)]
struct GroundTruthTag {
    #[serde(rename = "familyName")]
    family_name: String,
    #[serde(rename = "tagId")]
    tag_id: u32,
    corners: [[f64; 2]; 4],
    center: [f64; 2],
}
