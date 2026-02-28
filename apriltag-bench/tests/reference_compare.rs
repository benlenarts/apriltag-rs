/// Diagnostic test: compare Rust and C reference detection on gray-128 background
/// at various tag sizes. Prints results (does not assert) to investigate whether
/// the C reference also fails to detect large tags on mid-gray backgrounds.
#[cfg(feature = "reference")]
mod gray_background {
    use apriltag::detect::detector::{Detector, DetectorConfig};
    use apriltag::family;
    use apriltag_bench::reference::{reference_detect, ReferenceConfig};
    use apriltag_bench::scene::{Background, SceneBuilder};
    use apriltag_bench::transform::Transform;

    fn rust_detector() -> Detector {
        let mut det = Detector::new(DetectorConfig::default());
        det.add_family(family::builtin_family("tag36h11").unwrap(), 2);
        det
    }

    #[test]
    fn gray128_tag_sizes() {
        let ref_config = ReferenceConfig {
            quad_decimate: 2.0,
            nthreads: 1,
        };
        let detector = rust_detector();

        for tag_size in [100.0, 200.0, 300.0] {
            let scale = tag_size / 2.0;
            let scene = SceneBuilder::new(500, 500)
                .background(Background::Solid(128))
                .add_tag(
                    "tag36h11",
                    0,
                    Transform::Similarity {
                        cx: 250.0,
                        cy: 250.0,
                        scale,
                        theta: 0.0,
                    },
                )
                .build();

            let rust_dets = detector.detect(&scene.image);
            let ref_dets = reference_detect(&scene.image, "tag36h11", &ref_config);

            println!("\n=== tag_size={tag_size} (scale={scale}) ===");

            println!("Rust: {} detection(s)", rust_dets.len());
            for d in &rust_dets {
                println!(
                    "  id={} hamming={} margin={:.1} corners={:?}",
                    d.id, d.hamming, d.decision_margin, d.corners
                );
            }

            println!("C ref: {} detection(s)", ref_dets.len());
            for d in &ref_dets {
                // C returns corners [BL, BR, TR, TL]; print as-is for raw comparison
                println!(
                    "  id={} hamming={} margin={:.1} corners={:?}",
                    d.id, d.hamming, d.decision_margin, d.corners
                );
            }
        }
    }
}
