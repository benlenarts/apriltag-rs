/// End-to-end integration tests: build scenes → detect → evaluate metrics.
use apriltag::detect::detector::{Detector, DetectorConfig};
use apriltag::family;
use apriltag_bench::distortion::{self, Distortion};
use apriltag_bench::metrics;
use apriltag_bench::scene::{Background, SceneBuilder};
use apriltag_bench::transform::Transform;

fn detector_with_family(family_name: &str) -> Detector {
    let mut det = Detector::new(DetectorConfig::default());
    det.add_family(family::builtin_family(family_name).unwrap(), 2);
    det
}

#[test]
fn detect_single_centered_tag() {
    let scene = SceneBuilder::new(300, 300)
        .background(Background::Solid(128))
        .add_tag(
            "tag36h11",
            0,
            Transform::Similarity {
                cx: 150.0,
                cy: 150.0,
                scale: 50.0,
                theta: 0.0,
            },
        )
        .build();

    let detector = detector_with_family("tag36h11");
    let detections = detector.detect(&scene.image);

    assert_eq!(detections.len(), 1, "should detect exactly one tag");
    assert_eq!(detections[0].id, 0);
    assert_eq!(detections[0].family_name, "tag36h11");

    let result = metrics::evaluate(&scene.ground_truth, &detections, 0);
    assert_eq!(result.detection_rate, 1.0);
    assert!(
        result.corner_rmse < 2.0,
        "corner RMSE should be < 2px, got {}",
        result.corner_rmse
    );
}

#[test]
fn detect_rotated_tag() {
    let scene = SceneBuilder::new(300, 300)
        .background(Background::Solid(128))
        .add_tag(
            "tag36h11",
            5,
            Transform::Similarity {
                cx: 150.0,
                cy: 150.0,
                scale: 50.0,
                theta: std::f64::consts::FRAC_PI_4, // 45 degrees
            },
        )
        .build();

    let detector = detector_with_family("tag36h11");
    let detections = detector.detect(&scene.image);

    assert_eq!(detections.len(), 1, "should detect rotated tag");
    assert_eq!(detections[0].id, 5);

    let result = metrics::evaluate(&scene.ground_truth, &detections, 0);
    assert_eq!(result.detection_rate, 1.0);
    assert!(
        result.corner_rmse < 3.0,
        "rotated tag RMSE should be < 3px, got {}",
        result.corner_rmse
    );
}

#[test]
fn detect_multiple_tags() {
    let scene = SceneBuilder::new(500, 300)
        .background(Background::Solid(128))
        .add_tag(
            "tag36h11",
            0,
            Transform::Similarity {
                cx: 125.0,
                cy: 150.0,
                scale: 40.0,
                theta: 0.0,
            },
        )
        .add_tag(
            "tag36h11",
            1,
            Transform::Similarity {
                cx: 375.0,
                cy: 150.0,
                scale: 40.0,
                theta: 0.0,
            },
        )
        .build();

    let detector = detector_with_family("tag36h11");
    let detections = detector.detect(&scene.image);

    assert_eq!(detections.len(), 2, "should detect both tags");

    let result = metrics::evaluate(&scene.ground_truth, &detections, 0);
    assert_eq!(result.detection_rate, 1.0);
    assert!(result.false_positives.is_empty());
}

#[test]
fn detect_with_noise() {
    let mut scene = SceneBuilder::new(300, 300)
        .background(Background::Solid(128))
        .add_tag(
            "tag36h11",
            0,
            Transform::Similarity {
                cx: 150.0,
                cy: 150.0,
                scale: 50.0,
                theta: 0.0,
            },
        )
        .build();

    distortion::apply(
        &mut scene.image,
        &[Distortion::GaussianNoise {
            sigma: 10.0,
            seed: 42,
        }],
    );

    let detector = detector_with_family("tag36h11");
    let detections = detector.detect(&scene.image);

    assert_eq!(
        detections.len(),
        1,
        "should detect tag through moderate noise"
    );
    assert_eq!(detections[0].id, 0);
}

#[test]
fn detect_with_perspective_tilt() {
    // Moderate tilt in a large image to ensure the tag fits comfortably
    let scene = SceneBuilder::new(500, 500)
        .background(Background::Solid(128))
        .add_tag(
            "tag36h11",
            0,
            Transform::FromPose {
                center: [250.0, 250.0],
                size: 100.0,
                roll: 0.0,
                tilt_x: 0.3,
                tilt_y: 0.0,
            },
        )
        .build();

    let detector = detector_with_family("tag36h11");
    let detections = detector.detect(&scene.image);

    assert!(!detections.is_empty(), "should detect tilted tag");
    assert_eq!(detections[0].id, 0);

    let result = metrics::evaluate(&scene.ground_truth, &detections, 0);
    assert_eq!(result.detection_rate, 1.0);
    assert!(
        result.corner_rmse < 5.0,
        "perspective tag RMSE should be < 5px, got {}",
        result.corner_rmse
    );
}

#[test]
fn detect_with_gradient_lighting() {
    let mut scene = SceneBuilder::new(300, 300)
        .background(Background::Solid(128))
        .add_tag(
            "tag36h11",
            0,
            Transform::Similarity {
                cx: 150.0,
                cy: 150.0,
                scale: 50.0,
                theta: 0.0,
            },
        )
        .build();

    distortion::apply(
        &mut scene.image,
        &[Distortion::GradientLighting {
            direction: 0.0,
            min_factor: 0.5,
            max_factor: 1.5,
        }],
    );

    let detector = detector_with_family("tag36h11");
    let detections = detector.detect(&scene.image);

    assert_eq!(
        detections.len(),
        1,
        "should detect tag under gradient lighting"
    );
}

#[test]
fn metrics_json_round_trip() {
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

    let detector = detector_with_family("tag36h11");
    let detections = detector.detect(&scene.image);
    let result = metrics::evaluate(&scene.ground_truth, &detections, 1234);

    // Serialize to JSON and back
    let json = serde_json::to_string_pretty(&result).unwrap();
    let deserialized: metrics::SceneResult = serde_json::from_str(&json).unwrap();

    assert_eq!(deserialized.detection_rate, result.detection_rate);
    assert_eq!(deserialized.detection_time_us, 1234);
    assert_eq!(deserialized.matches.len(), result.matches.len());
}
