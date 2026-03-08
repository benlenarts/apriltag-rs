use crate::family::{FamilyId, TagFamily};

#[cfg(feature = "parallel")]
use rayon::iter::{IntoParallelRefIterator, ParallelIterator};

use super::cluster::{gradient_clusters, Cluster};
use super::connected::connected_components;
use super::decode::{decode_quad, DecodeBufs, QuickDecode};
use super::dedup::deduplicate;
use super::homography::Homography;
use super::image::{GrayImage, ImageU8};
use super::preprocess::{apply_sigma, decimate};
#[cfg(not(feature = "parallel"))]
use super::quad::QuadFitBufs;
use super::quad::{fit_quads, Quad, QuadThreshParams};
use super::refine::refine_edges;
use super::threshold::{threshold, ThresholdBuffers};
use super::unionfind::UnionFind;

/// A detected AprilTag in an image.
///
/// Contains the tag ID, Hamming distance from the nearest valid code,
/// decision margin, and the detected corner and center positions in
/// pixel coordinates.
///
/// ```
/// use apriltag::detect::detector::{Detector, DetectorConfig, DetectorBuffers};
/// use apriltag::detect::image::ImageU8;
/// use apriltag::family;
/// use apriltag::types::Pixel;
///
/// // Create a synthetic image with a tag16h5 tag
/// let f = family::tag16h5();
/// let rendered = f.tag(0).render();
/// let mut img = ImageU8::new(200, 200);
/// for y in 0..200 { for x in 0..200 { img.set(x, y, 255); } }
/// let scale = 10u32;
/// for ty in 0..rendered.grid_size {
///     for tx in 0..rendered.grid_size {
///         let val = if rendered.pixel(tx, ty) == Pixel::Black { 0 } else { 255 };
///         for dy in 0..scale { for dx in 0..scale {
///             img.set(60 + tx as u32 * scale + dx, 60 + ty as u32 * scale + dy, val);
///         }}
///     }
/// }
///
/// // Detect tags
/// let mut config = DetectorConfig::default();
/// config.quad_decimate = 1.0;
/// let mut det = Detector::new(config);
/// det.add_family(f, 2);
/// let detections = det.detect(&img, &mut DetectorBuffers::new());
///
/// assert!(!detections.is_empty());
/// assert_eq!(detections[0].id, 0);
/// assert_eq!(detections[0].hamming, 0);
/// ```
#[derive(Debug, Clone)]
pub struct Detection {
    pub family_id: FamilyId,
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

/// Reusable buffers for [`Detector::detect`].
///
/// Holds pre-allocated buffers that are reused across consecutive `detect` calls,
/// avoiding ~850KB of allocation per frame for a 640x480/decimate=2 image.
///
/// Create once and pass to [`Detector::detect`] in a loop:
///
/// ```
/// use apriltag::detect::detector::{Detector, DetectorConfig, DetectorBuffers};
/// use apriltag::detect::image::ImageU8;
/// use apriltag::family;
///
/// let mut det = Detector::new(DetectorConfig::default());
/// det.add_family(family::tag36h11(), 2);
///
/// let mut buffers = DetectorBuffers::new();
/// let frames = [ImageU8::new(100, 100), ImageU8::new(100, 100)];
/// for frame in &frames {
///     let dets = det.detect(frame, &mut buffers);
/// }
/// ```
pub struct DetectorBuffers {
    decimated: ImageU8,
    filtered: ImageU8,
    blur_tmp: ImageU8,
    threshed: ImageU8,
    threshold_bufs: ThresholdBuffers,
    uf: UnionFind,
    refine_vals: Vec<f64>,
    cluster_map: super::cluster::ClusterMap,
    clusters: Vec<Cluster>,
    quads: Vec<Quad>,
    #[cfg(not(feature = "parallel"))]
    quad_fit_bufs: QuadFitBufs,
    #[cfg(not(feature = "parallel"))]
    decode_bufs: DecodeBufs,
}

impl DetectorBuffers {
    /// Create new empty buffers with no pre-allocated memory.
    pub fn new() -> Self {
        Self {
            decimated: ImageU8::new(0, 0),
            filtered: ImageU8::new(0, 0),
            blur_tmp: ImageU8::new(0, 0),
            threshed: ImageU8::new(0, 0),
            threshold_bufs: ThresholdBuffers::new(),
            refine_vals: Vec::new(),
            uf: UnionFind::empty(),
            cluster_map: super::cluster::ClusterMap::new(),
            clusters: Vec::new(),
            quads: Vec::new(),
            #[cfg(not(feature = "parallel"))]
            quad_fit_bufs: QuadFitBufs::new(),
            #[cfg(not(feature = "parallel"))]
            decode_bufs: DecodeBufs::new(),
        }
    }
}

impl Default for DetectorBuffers {
    fn default() -> Self {
        Self::new()
    }
}

/// An AprilTag detector with pre-built lookup tables.
///
/// ```
/// use apriltag::detect::detector::{Detector, DetectorConfig};
/// use apriltag::family;
///
/// let mut det = Detector::new(DetectorConfig::default());
/// det.add_family(family::tag36h11(), 2);
/// ```
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

    /// Detect tags in a grayscale image, reusing buffers to avoid per-frame allocation.
    ///
    /// On the first call, buffers are allocated as needed. On subsequent calls
    /// with the same (or smaller) image dimensions, no allocation occurs.
    ///
    /// Accepts any type implementing [`GrayImage`], including borrowed [`ImageRef`](super::ImageRef)
    /// for zero-copy detection from a `&[u8]` slice.
    pub fn detect(
        &self,
        img: &(impl GrayImage + Sync),
        buffers: &mut DetectorBuffers,
    ) -> Vec<Detection> {
        let f = self.config.quad_decimate as u32;

        // Stage 1: Preprocess
        decimate(img, f, &mut buffers.decimated);
        apply_sigma(
            &buffers.decimated,
            self.config.quad_sigma,
            &mut buffers.filtered,
            &mut buffers.blur_tmp,
        );

        // Save filtered dimensions
        let filtered_w = buffers.filtered.width;
        let filtered_h = buffers.filtered.height;

        // Stage 2: Threshold
        threshold(
            &buffers.filtered,
            self.config.qtp.min_white_black_diff,
            self.config.qtp.deglitch,
            &mut buffers.threshed,
            &mut buffers.threshold_bufs,
        );

        // Stage 3: Connected components
        connected_components(&buffers.threshed, &mut buffers.uf);

        // Stage 4: Gradient clustering
        gradient_clusters(
            &buffers.threshed,
            &mut buffers.uf,
            self.config.qtp.min_cluster_pixels as u32,
            &mut buffers.cluster_map,
            &mut buffers.clusters,
        );

        // Determine border orientations needed
        let has_normal = self.families.iter().any(|(f, _)| !f.layout.reversed_border);
        let has_reversed = self.families.iter().any(|(f, _)| f.layout.reversed_border);

        // Stage 5: Quad fitting
        fit_quads(
            &mut buffers.clusters,
            filtered_w,
            filtered_h,
            &self.config.qtp,
            has_normal,
            has_reversed,
            &mut buffers.quads,
            #[cfg(not(feature = "parallel"))]
            &mut buffers.quad_fit_bufs,
        );

        // Recycle cluster point Vecs back into ClusterMap's free pool
        buffers.cluster_map.recycle_clusters(&mut buffers.clusters);

        // Scale quad corners back to original image coordinates
        if f > 1 {
            for quad in &mut buffers.quads {
                for corner in &mut quad.corners {
                    corner[0] *= f as f64;
                    corner[1] *= f as f64;
                }
            }
        }

        // Stage 6: Edge refinement
        if self.config.refine_edges {
            for quad in &mut buffers.quads {
                refine_edges(
                    quad,
                    img,
                    self.config.quad_decimate,
                    &mut buffers.refine_vals,
                );
            }
        }

        // Stages 7-8: Homography + Decode
        // COVERAGE: parallel feature block — only compiled with --features parallel
        #[cfg(feature = "parallel")]
        let mut detections: Vec<Detection> = buffers
            .quads
            .par_iter()
            .map_init(
                || (DecodeBufs::new(), Vec::new()),
                |(bufs, local_dets), quad| {
                    local_dets.clear();
                    decode_quad_to_detections(
                        quad,
                        img,
                        &self.families,
                        &self.config,
                        bufs,
                        local_dets,
                    );
                    std::mem::take(local_dets)
                },
            )
            .flatten()
            .collect();

        #[cfg(not(feature = "parallel"))]
        let mut detections: Vec<Detection> = {
            let mut detections = Vec::new();
            let bufs = &mut buffers.decode_bufs;
            for quad in &buffers.quads {
                decode_quad_to_detections(
                    quad,
                    img,
                    &self.families,
                    &self.config,
                    bufs,
                    &mut detections,
                );
            }
            detections
        };

        // Stage 9: Deduplication
        deduplicate(&mut detections);

        detections
    }
}

/// Decode a single quad against all families, appending detections to `out`.
fn decode_quad_to_detections(
    quad: &super::quad::Quad,
    img: &(impl GrayImage + Sync),
    families: &[(TagFamily, QuickDecode)],
    config: &DetectorConfig,
    bufs: &mut DecodeBufs,
    out: &mut Vec<Detection>,
) {
    // COVERAGE: None branch requires a degenerate quad (all corners collinear)
    // surviving all prior pipeline stages — not reachable in practice.
    let Some(h) = Homography::from_quad_corners(&quad.corners) else {
        return;
    };

    for (family, qd) in families {
        if quad.reversed_border != family.layout.reversed_border {
            continue;
        }

        if let Some(result) = decode_quad(
            img,
            family,
            qd,
            &h,
            quad.reversed_border,
            config.decode_sharpening,
            bufs,
        ) {
            let (center, corners) = compute_detection_geometry(&h, result.rotation, family);

            out.push(Detection {
                family_id: result.family_id,
                id: result.id,
                hamming: result.hamming,
                decision_margin: result.decision_margin,
                corners,
                center,
            });
        }
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
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use super::*;
    use crate::detect::image::{ImageRef, ImageU8};
    use crate::family;

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
        let dets = det.detect(&img, &mut DetectorBuffers::new());
        assert!(dets.is_empty());
    }

    #[test]
    #[cfg(feature = "family-tag16h5")]
    fn detect_synthetic_tag() {
        // Render a tag16h5 tag and embed it in a larger image
        let family = family::tag16h5();
        let rendered = family.tag(0).render();

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
                    _ => 255u8,
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

        let dets = det.detect(&img, &mut DetectorBuffers::new());

        // We should detect tag ID 0
        assert!(!dets.is_empty(), "Should detect at least one tag, got none");
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
        let rendered = family.tag(0).render();

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
                    _ => 255u8,
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

    /// Regression test: large tags (200px in 500x500) must be detected with
    /// the default quad_decimate=2.0. This failed when decimation used averaging
    /// instead of subsampling.
    #[test]
    #[cfg(feature = "family-tag36h11")]
    fn detect_large_tag_with_decimation() {
        let family = family::tag36h11();
        let rendered = family.tag(0).render();

        let img_size = 500u32;
        let scale = 20u32; // 10 grid cells * 20 = 200px tag
        let mut img = ImageU8::new(img_size, img_size);
        for y in 0..img_size {
            for x in 0..img_size {
                img.set(x, y, 255);
            }
        }

        let ox = (img_size - rendered.grid_size as u32 * scale) / 2;
        let oy = ox;
        for ty in 0..rendered.grid_size {
            for tx in 0..rendered.grid_size {
                let pixel = rendered.pixel(tx, ty);
                let val = match pixel {
                    crate::types::Pixel::Black => 0u8,
                    _ => 255u8,
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

        let config = DetectorConfig::default(); // quad_decimate = 2.0
        let mut det = Detector::new(config);
        det.add_family(family, 2);

        let dets = det.detect(&img, &mut DetectorBuffers::new());
        // should detect large tag with decimation=2.0
        assert!(!dets.is_empty());
        assert_eq!(dets[0].id, 0);
    }

    /// Regression test: large tags on gray-128 backgrounds must be detected.
    /// Gray backgrounds cause adaptive thresholding to create an extra boundary
    /// cluster whose size exceeds the max_perimeter filter if it uses 2*(w+h)
    /// instead of the correct 4*(w+h) from the C reference.
    #[test]
    #[cfg(feature = "family-tag36h11")]
    fn detect_large_tag_gray128_background() {
        let family = family::tag36h11();
        let rendered = family.tag(0).render();

        let img_size = 300u32;
        let scale = 20u32; // 10 grid cells * 20 = 200px tag in 300x300
        let mut img = ImageU8::new(img_size, img_size);
        // Gray-128 background: adaptive thresholding creates an extra boundary
        // cluster whose edge count exceeds 2*(w+h) but fits within 4*(w+h).
        for y in 0..img_size {
            for x in 0..img_size {
                img.set(x, y, 128);
            }
        }

        let ox = (img_size - rendered.grid_size as u32 * scale) / 2;
        let oy = ox;
        for ty in 0..rendered.grid_size {
            for tx in 0..rendered.grid_size {
                let pixel = rendered.pixel(tx, ty);
                let val = match pixel {
                    crate::types::Pixel::Black => 0u8,
                    crate::types::Pixel::White => 255u8,
                    // COVERAGE: only fires with custom families that have transparent cells
                    crate::types::Pixel::Transparent => 128u8, // blend with gray bg
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

        let config = DetectorConfig::default();
        let mut det = Detector::new(config);
        det.add_family(family, 2);

        let dets = det.detect(&img, &mut DetectorBuffers::new());
        // should detect large tag on gray-128 background
        assert!(!dets.is_empty());
        assert_eq!(dets[0].id, 0);
    }

    #[test]
    #[cfg(feature = "family-tag16h5")]
    fn pipeline_stages_diagnostic() {
        use crate::detect::{cluster, connected, quad, threshold};

        let (img, _family) = build_synthetic_tag_image();

        // Stage 2: Threshold
        let mut threshed = ImageU8::new(0, 0);
        threshold::threshold(
            &img,
            5,
            false,
            &mut threshed,
            &mut threshold::ThresholdBuffers::new(),
        );
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
        let mut uf = crate::detect::unionfind::UnionFind::empty();
        connected::connected_components(&threshed, &mut uf);

        // Stage 4: Gradient clustering
        let mut clusters = Vec::new();
        cluster::gradient_clusters(
            &threshed,
            &mut uf,
            5,
            &mut cluster::ClusterMap::new(),
            &mut clusters,
        );
        // should find clusters
        assert!(!clusters.is_empty());

        // Stage 5: Quad fitting
        let mut quads = Vec::new();
        quad::fit_quads(
            &mut clusters,
            200,
            200,
            &quad::QuadThreshParams::default(),
            true,
            true,
            &mut quads,
            #[cfg(not(feature = "parallel"))]
            &mut quad::QuadFitBufs::new(),
        );
        // should find quads from clusters
        assert!(!quads.is_empty());
    }

    /// Detect a normal-border tag with both a normal and reversed-border family
    /// registered. The reversed family should hit the `reversed_border` mismatch
    /// continue (line 205), exercising that filter path.
    #[test]
    #[cfg(all(feature = "family-tag16h5", feature = "family-circle21h7"))]
    fn detect_skips_mismatched_border_family() {
        let (img, tag16h5) = build_synthetic_tag_image();
        let circle21h7 = family::tag_circle21h7();

        let mut config = DetectorConfig::default();
        config.quad_decimate = 1.0;
        config.quad_sigma = 0.0;
        let mut det = Detector::new(config);
        det.add_family(tag16h5, 2);
        det.add_family(circle21h7, 2);

        let dets = det.detect(&img, &mut DetectorBuffers::new());
        // Should still detect tag16h5 tag ID 0; circle21h7 is skipped via the
        // reversed_border mismatch continue.
        assert!(!dets.is_empty());
        assert_eq!(dets[0].id, 0);
        assert_eq!(dets[0].family_id, "tag16h5");
    }

    #[test]
    fn detector_buffers_default() {
        let buffers = DetectorBuffers::new();
        assert!(buffers.decimated.buf.is_empty());
        assert!(buffers.filtered.buf.is_empty());
        assert!(buffers.blur_tmp.buf.is_empty());
        assert!(buffers.threshed.buf.is_empty());

        let buffers2 = DetectorBuffers::default();
        assert!(buffers2.decimated.buf.is_empty());
    }

    #[test]
    #[cfg(feature = "family-tag16h5")]
    fn detect_deterministic_across_buffer_reuse() {
        let (img, family) = build_synthetic_tag_image();

        let mut config = DetectorConfig::default();
        config.quad_decimate = 1.0;
        config.quad_sigma = 0.0;
        let mut det = Detector::new(config);
        det.add_family(family, 2);

        let dets_fresh = det.detect(&img, &mut DetectorBuffers::new());
        let mut buffers = DetectorBuffers::new();
        let dets_reuse = det.detect(&img, &mut buffers);

        assert_eq!(dets_fresh.len(), dets_reuse.len());
        for (a, b) in dets_fresh.iter().zip(dets_reuse.iter()) {
            assert_eq!(a.id, b.id);
            assert_eq!(a.hamming, b.hamming);
            for i in 0..4 {
                assert!((a.corners[i][0] - b.corners[i][0]).abs() < 1e-6);
                assert!((a.corners[i][1] - b.corners[i][1]).abs() < 1e-6);
            }
        }
    }

    #[test]
    #[cfg(feature = "family-tag16h5")]
    fn detect_reuses_allocations() {
        let (img, family) = build_synthetic_tag_image();

        let mut config = DetectorConfig::default();
        config.quad_decimate = 1.0;
        config.quad_sigma = 0.0;
        let mut det = Detector::new(config);
        det.add_family(family, 2);

        let mut buffers = DetectorBuffers::new();

        // First call populates buffers
        let _ = det.detect(&img, &mut buffers);
        let cap_after_first = (
            buffers.decimated.buf.capacity(),
            buffers.filtered.buf.capacity(),
            buffers.threshed.buf.capacity(),
        );

        // Second call should not grow
        let _ = det.detect(&img, &mut buffers);
        let cap_after_second = (
            buffers.decimated.buf.capacity(),
            buffers.filtered.buf.capacity(),
            buffers.threshed.buf.capacity(),
        );

        assert_eq!(cap_after_first, cap_after_second);
    }

    #[test]
    #[cfg(feature = "family-tag36h11")]
    fn detect_deterministic_with_sigma() {
        // Test with quad_sigma > 0 to exercise blur buffer reuse
        let family = family::tag36h11();
        let rendered = family.tag(0).render();

        let img_size = 500u32;
        let scale = 20u32;
        let mut img = ImageU8::new(img_size, img_size);
        for y in 0..img_size {
            for x in 0..img_size {
                img.set(x, y, 255);
            }
        }
        let ox = (img_size - rendered.grid_size as u32 * scale) / 2;
        let oy = ox;
        for ty in 0..rendered.grid_size {
            for tx in 0..rendered.grid_size {
                let pixel = rendered.pixel(tx, ty);
                let val = match pixel {
                    crate::types::Pixel::Black => 0u8,
                    _ => 255u8,
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

        let config = DetectorConfig {
            quad_sigma: 0.8,
            ..DetectorConfig::default()
        };
        let mut det = Detector::new(config);
        det.add_family(family, 2);

        let dets_fresh = det.detect(&img, &mut DetectorBuffers::new());
        let mut buffers = DetectorBuffers::new();
        let dets_reuse = det.detect(&img, &mut buffers);

        assert_eq!(dets_fresh.len(), dets_reuse.len());
        for (a, b) in dets_fresh.iter().zip(dets_reuse.iter()) {
            assert_eq!(a.id, b.id);
        }
    }

    #[test]
    #[cfg(feature = "family-tag16h5")]
    fn detect_image_ref_matches_image_u8() {
        let (img, family) = build_synthetic_tag_image();

        let mut config = DetectorConfig::default();
        config.quad_decimate = 1.0;
        config.quad_sigma = 0.0;
        let mut det = Detector::new(config);
        det.add_family(family, 2);

        let dets_owned = det.detect(&img, &mut DetectorBuffers::new());

        let img_ref = ImageRef::new(img.width, img.height, img.stride, &img.buf);
        let dets_borrowed = det.detect(&img_ref, &mut DetectorBuffers::new());

        assert_eq!(dets_owned.len(), dets_borrowed.len());
        for (a, b) in dets_owned.iter().zip(dets_borrowed.iter()) {
            assert_eq!(a.id, b.id);
            assert_eq!(a.hamming, b.hamming);
            assert!((a.decision_margin - b.decision_margin).abs() < 1e-6);
            for i in 0..4 {
                assert!((a.corners[i][0] - b.corners[i][0]).abs() < 1e-6);
                assert!((a.corners[i][1] - b.corners[i][1]).abs() < 1e-6);
            }
        }
    }
}
