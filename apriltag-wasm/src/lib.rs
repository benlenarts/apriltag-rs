use serde::{Deserialize, Serialize};
use tsify_next::Tsify;
use wasm_bindgen::prelude::*;

use apriltag::detect::detector::{
    Detection as CoreDetection, Detector as CoreDetector, DetectorConfig,
};
use apriltag::detect::image::ImageU8;
use apriltag::detect::pose::{estimate_tag_pose, PoseParams};
use apriltag::family;

// ── Tsify types for TypeScript interface generation ──

/// Detector configuration passed from JavaScript.
#[derive(Tsify, Serialize, Deserialize)]
#[tsify(into_wasm_abi, from_wasm_abi)]
pub struct WasmDetectorConfig {
    /// Tag family names to detect (e.g. ["tag36h11"]).
    pub families: Vec<String>,
    /// Decimation factor (default: 2.0).
    #[serde(default = "default_decimate")]
    pub quad_decimate: Option<f32>,
    /// Gaussian blur sigma (default: 0.0).
    #[serde(default)]
    pub quad_sigma: Option<f32>,
    /// Enable edge refinement (default: true).
    #[serde(default)]
    pub refine_edges: Option<bool>,
    /// Decode sharpening factor (default: 0.25).
    #[serde(default)]
    pub decode_sharpening: Option<f64>,
    /// Maximum Hamming distance for matching (default: 2).
    #[serde(default)]
    pub max_hamming: Option<u32>,
}

fn default_decimate() -> Option<f32> {
    Some(2.0)
}

/// A detected AprilTag returned to JavaScript.
#[derive(Tsify, Serialize, Deserialize)]
#[tsify(into_wasm_abi, from_wasm_abi)]
pub struct WasmDetection {
    pub family: String,
    pub id: i32,
    pub hamming: i32,
    pub decision_margin: f32,
    pub center: [f64; 2],
    pub corners: [[f64; 2]; 4],
}

/// A 3D pose estimate returned to JavaScript.
#[derive(Tsify, Serialize, Deserialize)]
#[tsify(into_wasm_abi)]
pub struct WasmPose {
    /// 3x3 rotation matrix (row-major, 9 elements).
    pub rotation: Vec<f64>,
    /// Translation vector [x, y, z].
    pub translation: [f64; 3],
    /// Reprojection error.
    pub error: f64,
}

// ── Detector wrapper ──

/// AprilTag detector for use from JavaScript/TypeScript.
#[wasm_bindgen]
pub struct Detector {
    inner: CoreDetector,
}

#[wasm_bindgen]
impl Detector {
    /// Create a new detector with the given configuration.
    #[wasm_bindgen(constructor)]
    pub fn new(config: WasmDetectorConfig) -> Result<Detector, JsError> {
        let mut det_config = DetectorConfig::default();

        if let Some(d) = config.quad_decimate {
            det_config.quad_decimate = d;
        }
        if let Some(s) = config.quad_sigma {
            det_config.quad_sigma = s;
        }
        if let Some(r) = config.refine_edges {
            det_config.refine_edges = r;
        }
        if let Some(s) = config.decode_sharpening {
            det_config.decode_sharpening = s;
        }

        let max_hamming = config.max_hamming.unwrap_or(2);
        let mut inner = CoreDetector::new(det_config);

        for family_name in &config.families {
            let fam = family::builtin_family(family_name).ok_or_else(|| {
                JsError::new(&format!("unknown tag family: {family_name}"))
            })?;
            inner.add_family(fam, max_hamming);
        }

        Ok(Detector { inner })
    }

    /// Detect tags in a grayscale image (one byte per pixel).
    pub fn detect(&self, data: &[u8], width: u32, height: u32) -> Result<JsValue, JsError> {
        let expected = (width * height) as usize;
        if data.len() != expected {
            return Err(JsError::new(&format!(
                "data length {} does not match {}x{} = {}",
                data.len(),
                width,
                height,
                expected,
            )));
        }

        let img = ImageU8::from_buf(width, height, width, data.to_vec());
        let detections = self.inner.detect(&img);

        let wasm_dets: Vec<WasmDetection> = detections
            .iter()
            .map(detection_to_wasm)
            .collect();

        serde_wasm_bindgen::to_value(&wasm_dets).map_err(|e| JsError::new(&e.to_string()))
    }

    /// Detect tags in an RGBA image (4 bytes per pixel).
    pub fn detect_rgba(
        &self,
        data: &[u8],
        width: u32,
        height: u32,
    ) -> Result<JsValue, JsError> {
        let expected = (width * height * 4) as usize;
        if data.len() != expected {
            return Err(JsError::new(&format!(
                "RGBA data length {} does not match {}x{}x4 = {}",
                data.len(),
                width,
                height,
                expected,
            )));
        }

        let gray: Vec<u8> = data
            .chunks_exact(4)
            .map(|px| ((77u32 * px[0] as u32 + 150u32 * px[1] as u32 + 29u32 * px[2] as u32) >> 8) as u8)
            .collect();

        self.detect(&gray, width, height)
    }

    /// Estimate the pose of a detected tag.
    ///
    /// Returns the best pose (lowest reprojection error) from up to two candidates.
    pub fn estimate_pose(
        &self,
        detection: WasmDetection,
        tagsize: f64,
        fx: f64,
        fy: f64,
        cx: f64,
        cy: f64,
    ) -> Result<JsValue, JsError> {
        let core_det = CoreDetection {
            family_name: detection.family,
            id: detection.id,
            hamming: detection.hamming,
            decision_margin: detection.decision_margin,
            corners: detection.corners,
            center: detection.center,
        };

        let params = PoseParams {
            tagsize,
            fx,
            fy,
            cx,
            cy,
        };

        let (pose1, err1, pose2, err2) = estimate_tag_pose(&core_det, &params);

        let best_pose = if let Some(p2) = pose2 {
            if err2 < err1 {
                pose_to_wasm(&p2, err2)
            } else {
                pose_to_wasm(&pose1, err1)
            }
        } else {
            pose_to_wasm(&pose1, err1)
        };

        serde_wasm_bindgen::to_value(&best_pose).map_err(|e| JsError::new(&e.to_string()))
    }
}

fn detection_to_wasm(det: &CoreDetection) -> WasmDetection {
    WasmDetection {
        family: det.family_name.clone(),
        id: det.id,
        hamming: det.hamming,
        decision_margin: det.decision_margin,
        center: det.center,
        corners: det.corners,
    }
}

fn pose_to_wasm(pose: &apriltag::detect::pose::Pose, error: f64) -> WasmPose {
    WasmPose {
        rotation: vec![
            pose.r[0][0], pose.r[0][1], pose.r[0][2],
            pose.r[1][0], pose.r[1][1], pose.r[1][2],
            pose.r[2][0], pose.r[2][1], pose.r[2][2],
        ],
        translation: pose.t,
        error,
    }
}
