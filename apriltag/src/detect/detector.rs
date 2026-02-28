use crate::family::TagFamily;

#[cfg(feature = "parallel")]
use rayon::iter::{IntoParallelRefIterator, ParallelIterator};

use super::cluster::gradient_clusters;
use super::connected::connected_components;
use super::decode::{decode_quad, QuickDecode};
use super::dedup::deduplicate;
use super::homography::Homography;
use super::image::ImageU8;
use super::preprocess::{apply_sigma, decimate};
use super::quad::{fit_quads, QuadThreshParams};
use super::refine::refine_edges;
use super::threshold::threshold;

/// A detected AprilTag in an image.
#[derive(Debug, Clone)]
pub struct Detection {
    pub family_name: String,
    pub id: i32,
    pub hamming: i32,
    pub decision_margin: f32,
    pub corners: [[f64; 2]; 4],
    pub center: [f64; 2],
}

/// Detector configuration.
#[derive(Debug, Clone)]
pub struct DetectorConfig {
    pub quad_decimate: f32,
    pub quad_sigma: f32,
    pub refine_edges: bool,
    pub decode_sharpening: f64,
    pub qtp: QuadThreshParams,
}

impl Default for DetectorConfig {
    fn default() -> Self {
        Self {
            quad_decimate: 2.0,
            quad_sigma: 0.0,
            refine_edges: true,
            decode_sharpening: 0.25,
            qtp: QuadThreshParams::default(),
        }
    }
}

/// An AprilTag detector with pre-built lookup tables.
pub struct Detector {
    pub config: DetectorConfig,
    families: Vec<(TagFamily, QuickDecode)>,
}

impl Detector {
    /// Create a new detector with the given configuration.
    pub fn new(config: DetectorConfig) -> Self {
        Self {
            config,
            families: Vec::new(),
        }
    }

    /// Add a tag family to the detector with the given maximum Hamming distance.
    pub fn add_family(&mut self, family: TagFamily, maxhamming: u32) {
        let qd = QuickDecode::new(&family, maxhamming);
        self.families.push((family, qd));
    }

    /// Detect tags in a grayscale image.
    pub fn detect(&self, img: &ImageU8) -> Vec<Detection> {
        let f = self.config.quad_decimate as u32;

        // Stage 1: Preprocess
        let decimated = decimate(img, f);
        let filtered = apply_sigma(&decimated, self.config.quad_sigma);

        // Stage 2: Threshold
        let threshed = threshold(
            &filtered,
            self.config.qtp.min_white_black_diff,
            self.config.qtp.deglitch,
        );

        // Stage 3: Connected components
        let mut uf = connected_components(&threshed);

        // Stage 4: Gradient clustering
        let mut clusters = gradient_clusters(
            &threshed,
            &mut uf,
            self.config.qtp.min_cluster_pixels as u32,
        );

        // Determine border orientations needed
        let has_normal = self.families.iter().any(|(f, _)| !f.layout.reversed_border);
        let has_reversed = self.families.iter().any(|(f, _)| f.layout.reversed_border);

        // Stage 5: Quad fitting
        let mut quads = fit_quads(
            &mut clusters,
            filtered.width,
            filtered.height,
            &self.config.qtp,
            has_normal,
            has_reversed,
        );

        // Scale quad corners back to original image coordinates
        if f > 1 {
            for quad in &mut quads {
                for corner in &mut quad.corners {
                    corner[0] *= f as f64;
                    corner[1] *= f as f64;
                }
            }
        }

        // Stage 6: Edge refinement
        if self.config.refine_edges {
            for quad in &mut quads {
                refine_edges(quad, img, self.config.quad_decimate);
            }
        }

        // Stages 7-8: Homography + Decode
        let decode_one = |quad: &super::quad::Quad| -> Vec<Detection> {
            let h = match Homography::from_quad_corners(&quad.corners) {
                Some(h) => h,
                None => return Vec::new(),
            };

            let mut dets = Vec::new();
            for (family, qd) in &self.families {
                if quad.reversed_border != family.layout.reversed_border {
                    continue;
                }

                if let Some(result) = decode_quad(
                    img,
                    family,
                    qd,
                    &h,
                    quad.reversed_border,
                    self.config.decode_sharpening,
                ) {
                    let (center, corners) =
                        compute_detection_geometry(&h, result.rotation, family);

                    dets.push(Detection {
                        family_name: result.family_name,
                        id: result.id,
                        hamming: result.hamming,
                        decision_margin: result.decision_margin,
                        corners,
                        center,
                    });
                }
            }
            dets
        };

        #[cfg(feature = "parallel")]
        let mut detections: Vec<Detection> = quads
            .par_iter()
            .flat_map(decode_one)
            .collect();

        #[cfg(not(feature = "parallel"))]
        let mut detections: Vec<Detection> = quads
            .iter()
            .flat_map(decode_one)
            .collect();

        // Stage 9: Deduplication
        deduplicate(&mut detections);

        detections
    }
}

/// Compute center and rotation-corrected corner positions.
fn compute_detection_geometry(
    h: &Homography,
    rotation: i32,
    _family: &TagFamily,
) -> ([f64; 2], [[f64; 2]; 4]) {
    let (cx, cy) = h.project(0.0, 0.0);

    // Tag corners in canonical order, rotated by the detected rotation
    let base_corners = [[-1.0, -1.0], [1.0, -1.0], [1.0, 1.0], [-1.0, 1.0]];
    let mut corners = [[0.0f64; 2]; 4];
    for i in 0..4 {
        let src = &base_corners[(i + rotation as usize) % 4];
        let (px, py) = h.project(src[0], src[1]);
        corners[i] = [px, py];
    }

    ([cx, cy], corners)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::detect::image::ImageU8;
    use crate::family;
    use crate::render;

    #[test]
    fn detector_default_config() {
        let config = DetectorConfig::default();
        assert!((config.quad_decimate - 2.0).abs() < 1e-6);
        assert!((config.quad_sigma - 0.0).abs() < 1e-6);
        assert!(config.refine_edges);
        assert!((config.decode_sharpening - 0.25).abs() < 1e-6);
    }

    #[test]
    #[cfg(feature = "family-tag36h11")]
    fn detector_add_family() {
        let mut det = Detector::new(DetectorConfig::default());
        det.add_family(family::tag36h11(), 2);
        assert_eq!(det.families.len(), 1);
    }

    #[test]
    #[cfg(feature = "family-tag16h5")]
    fn detect_empty_image_no_crash() {
        let mut det = Detector::new(DetectorConfig::default());
        det.add_family(family::tag16h5(), 2);
        let img = ImageU8::new(100, 100);
        let dets = det.detect(&img);
        assert!(dets.is_empty());
    }

    #[test]
    #[cfg(feature = "family-tag16h5")]
    fn detect_synthetic_tag() {
        // Render a tag16h5 tag and embed it in a larger image
        let family = family::tag16h5();
        let rendered = render::render(&family.layout, family.codes[0]);

        // Create a 200x200 image with white background
        let mut img = ImageU8::new(200, 200);
        for y in 0..200 {
            for x in 0..200 {
                img.set(x, y, 255);
            }
        }

        // Place the 8x8 tag at center, scaled up by 10x
        let scale = 10;
        let ox = 60u32;
        let oy = 60u32;
        for ty in 0..rendered.grid_size {
            for tx in 0..rendered.grid_size {
                let pixel = rendered.pixel(tx, ty);
                let val = match pixel {
                    crate::types::Pixel::Black => 0u8,
                    crate::types::Pixel::White => 255u8,
                    crate::types::Pixel::Transparent => 255u8,
                };
                for dy in 0..scale {
                    for dx in 0..scale {
                        img.set(
                            ox + tx as u32 * scale + dx,
                            oy + ty as u32 * scale + dy,
                            val,
                        );
                    }
                }
            }
        }

        let mut config = DetectorConfig::default();
        config.quad_decimate = 1.0; // no decimation for synthetic test
        config.quad_sigma = 0.0;
        let mut det = Detector::new(config);
        det.add_family(family, 2);

        let dets = det.detect(&img);

        // We should detect tag ID 0
        assert!(
            !dets.is_empty(),
            "Should detect at least one tag, got none"
        );
        assert_eq!(dets[0].id, 0, "Should detect tag ID 0");
    }

    #[test]
    #[cfg(feature = "family-tag16h5")]
    fn compute_detection_geometry_identity() {
        let corners = [[-1.0, -1.0], [1.0, -1.0], [1.0, 1.0], [-1.0, 1.0]];
        let h = Homography::from_quad_corners(&corners).unwrap();
        let family = family::tag16h5();
        let (center, det_corners) = compute_detection_geometry(&h, 0, &family);
        assert!((center[0] - 0.0).abs() < 1e-6);
        assert!((center[1] - 0.0).abs() < 1e-6);
        for i in 0..4 {
            assert!((det_corners[i][0] - corners[i][0]).abs() < 1e-6);
            assert!((det_corners[i][1] - corners[i][1]).abs() < 1e-6);
        }
    }

    /// Helper to build the synthetic tag image used across tests.
    #[cfg(feature = "family-tag16h5")]
    fn build_synthetic_tag_image() -> (ImageU8, crate::family::TagFamily) {
        let family = family::tag16h5();
        let rendered = render::render(&family.layout, family.codes[0]);

        let mut img = ImageU8::new(200, 200);
        for y in 0..200 {
            for x in 0..200 {
                img.set(x, y, 255);
            }
        }

        let scale = 10u32;
        let ox = 60u32;
        let oy = 60u32;
        for ty in 0..rendered.grid_size {
            for tx in 0..rendered.grid_size {
                let pixel = rendered.pixel(tx, ty);
                let val = match pixel {
                    crate::types::Pixel::Black => 0u8,
                    crate::types::Pixel::White => 255u8,
                    crate::types::Pixel::Transparent => 255u8,
                };
                for dy in 0..scale {
                    for dx in 0..scale {
                        img.set(
                            ox + tx as u32 * scale + dx,
                            oy + ty as u32 * scale + dy,
                            val,
                        );
                    }
                }
            }
        }

        (img, family)
    }

    #[test]
    #[cfg(feature = "family-tag16h5")]
    fn pipeline_stages_diagnostic() {
        use crate::detect::{threshold, connected, cluster, quad};

        let (img, _family) = build_synthetic_tag_image();

        // Stage 2: Threshold
        let threshed = threshold::threshold(&img, 5, false);
        let mut black_count = 0;
        let mut white_count = 0;
        let mut unknown_count = 0;
        for y in 0..200 {
            for x in 0..200 {
                match threshed.get(x, y) {
                    0 => black_count += 1,
                    255 => white_count += 1,
                    _ => unknown_count += 1,
                }
            }
        }
        assert!(black_count > 0, "No black pixels after threshold");
        assert!(white_count > 0, "No white pixels after threshold");

        // Stage 3: Connected components
        let mut uf = connected::connected_components(&threshed);

        // Stage 4: Gradient clustering
        let mut clusters = cluster::gradient_clusters(&threshed, &mut uf, 5);
        assert!(
            !clusters.is_empty(),
            "No clusters found (black={black_count}, white={white_count}, unknown={unknown_count})"
        );

        // Stage 5: Quad fitting
        let quads = quad::fit_quads(
            &mut clusters,
            200,
            200,
            &quad::QuadThreshParams::default(),
            true,
            true,
        );
        assert!(
            !quads.is_empty(),
            "No quads found from {} clusters (largest: {} pts)",
            clusters.len(),
            clusters.iter().map(|c| c.points.len()).max().unwrap_or(0),
        );
    }
}
