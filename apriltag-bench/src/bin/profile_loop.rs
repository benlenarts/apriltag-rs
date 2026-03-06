//! Tight detection loop for profiling with samply or other profilers.
//!
//! Generates a noisy scene and runs the detector N times to accumulate
//! enough samples for meaningful profiling data.
//!
//! Usage: profile_loop [scenario-name] [iterations]
//! Default: noise-sigma20, 1000 iterations

use apriltag::detect::detector::{Detector, DetectorBuffers, DetectorConfig};
use apriltag::family;
use apriltag_bench::distortion::{self, Distortion};
use apriltag_bench::scene::{Background, SceneBuilder};
use apriltag_bench::transform::Transform;

fn build_noise_scene(sigma: u32) -> apriltag::detect::image::ImageU8 {
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
            sigma: sigma as f64,
            seed: 42,
        }],
    );
    scene.image
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let scenario = args.get(1).map(|s| s.as_str()).unwrap_or("noise-sigma20");
    let iterations: usize = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(1000);

    let sigma: u32 = match scenario {
        "noise-sigma5" => 5,
        "noise-sigma10" => 10,
        "noise-sigma20" => 20,
        "noise-sigma40" => 40,
        other => {
            eprintln!("Unknown scenario: {other}");
            eprintln!("Available: noise-sigma5, noise-sigma10, noise-sigma20, noise-sigma40");
            std::process::exit(1);
        }
    };

    let image = build_noise_scene(sigma);
    eprintln!(
        "Scene: {scenario} ({}x{}), {iterations} iterations",
        image.width, image.height
    );

    let mut detector = Detector::new(DetectorConfig::default());
    detector.add_family(family::tag36h11(), 2);

    let mut buffers = DetectorBuffers::new();

    // Warmup
    let _ = detector.detect(&image, &mut buffers);

    for _ in 0..iterations {
        let dets = detector.detect(&image, &mut buffers);
        // Prevent the compiler from optimizing away the detection
        std::hint::black_box(&dets);
    }

    eprintln!("Done.");
}
