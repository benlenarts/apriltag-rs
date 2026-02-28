use std::time::Instant;

use apriltag::detect::detector::{Detector, DetectorConfig};
use apriltag::family;
use clap::{Parser, Subcommand};

use apriltag_bench::catalog::{self, Category, Scenario};
use apriltag_bench::distortion::{self, Distortion};
use apriltag_bench::metrics;
use apriltag_bench::report::{self, FullReport};
use apriltag_bench::scene::{Background, SceneBuilder};
use apriltag_bench::transform::Transform;

#[derive(Parser)]
#[command(name = "apriltag-bench", about = "AprilTag detection test harness")]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    /// Run test scenarios and output results.
    Run {
        /// Filter by category name.
        #[arg(long)]
        category: Option<String>,
        /// Filter by scenario name pattern (substring match).
        #[arg(long)]
        scenario: Option<String>,
        /// Output format: terminal, json.
        #[arg(long, default_value = "terminal")]
        format: String,
        /// Corner RMSE pass threshold in pixels.
        #[arg(long, default_value_t = 0.0)]
        threshold: f64,
        /// Only show failures.
        #[arg(long)]
        quiet: bool,
    },
    /// List available scenarios.
    List {
        /// Filter by category.
        #[arg(long)]
        category: Option<String>,
    },
    /// Run all scenarios and exit with code 1 on any failure.
    Regression {
        /// Filter by category.
        #[arg(long)]
        category: Option<String>,
    },
    /// Generate and detect a single scene with custom parameters.
    Explore {
        /// Tag family.
        #[arg(long, default_value = "tag36h11")]
        family: String,
        /// Tag ID.
        #[arg(long, default_value_t = 0)]
        tag_id: u32,
        /// Tag size in pixels (border region width).
        #[arg(long, default_value_t = 100.0)]
        tag_size: f64,
        /// In-plane rotation in degrees.
        #[arg(long, default_value_t = 0.0)]
        rotation: f64,
        /// Perspective tilt X in degrees.
        #[arg(long, default_value_t = 0.0)]
        tilt_x: f64,
        /// Perspective tilt Y in degrees.
        #[arg(long, default_value_t = 0.0)]
        tilt_y: f64,
        /// Gaussian noise sigma.
        #[arg(long, default_value_t = 0.0)]
        noise: f64,
        /// Gaussian blur sigma.
        #[arg(long, default_value_t = 0.0)]
        blur: f64,
        /// Contrast scale factor.
        #[arg(long, default_value_t = 1.0)]
        contrast: f64,
        /// Image width.
        #[arg(long, default_value_t = 500)]
        width: u32,
        /// Image height.
        #[arg(long, default_value_t = 500)]
        height: u32,
        /// Output format: terminal, json.
        #[arg(long, default_value = "terminal")]
        format: String,
    },
}

fn main() {
    let cli = Cli::parse();

    match cli.command {
        Command::Run {
            category,
            scenario,
            format,
            threshold,
            quiet,
        } => cmd_run(category, scenario, &format, threshold, quiet),
        Command::List { category } => cmd_list(category),
        Command::Regression { category } => cmd_regression(category),
        Command::Explore {
            family,
            tag_id,
            tag_size,
            rotation,
            tilt_x,
            tilt_y,
            noise,
            blur,
            contrast,
            width,
            height,
            format,
        } => cmd_explore(
            &family, tag_id, tag_size, rotation, tilt_x, tilt_y, noise, blur, contrast, width,
            height, &format,
        ),
    }
}

fn filter_scenarios(category: Option<String>, scenario: Option<String>) -> Vec<Scenario> {
    let mut scenarios = if let Some(cat_name) = &category {
        let cat =
            Category::from_name(cat_name).unwrap_or_else(|| panic!("unknown category: {cat_name}"));
        catalog::scenarios_for_category(cat)
    } else {
        catalog::all_scenarios()
    };

    if let Some(pattern) = &scenario {
        scenarios.retain(|s| s.name.contains(pattern.as_str()));
    }

    scenarios
}

fn run_scenario(scenario: &Scenario) -> (metrics::SceneResult, std::time::Duration) {
    let scene = scenario.build();

    let mut config = DetectorConfig::default();
    if let Some(decimate) = scenario.quad_decimate {
        config.quad_decimate = decimate;
    }

    let mut detector = Detector::new(config);
    for (fam_name, _) in &scenario.expect_ids {
        if let Some(fam) = family::builtin_family(fam_name) {
            detector.add_family(fam, 2);
        }
    }

    let start = Instant::now();
    let detections = detector.detect(&scene.image);
    let elapsed = start.elapsed();

    let result = metrics::evaluate(&scene.ground_truth, &detections, elapsed.as_micros() as u64);
    (result, elapsed)
}

fn cmd_run(
    category: Option<String>,
    scenario: Option<String>,
    format: &str,
    threshold_override: f64,
    quiet: bool,
) {
    let scenarios = filter_scenarios(category, scenario);

    let mut reports = Vec::new();
    for s in &scenarios {
        let threshold = if threshold_override > 0.0 {
            threshold_override
        } else {
            s.max_corner_rmse
        };
        let (result, _) = run_scenario(s);
        let r = report::scenario_report(
            &s.name,
            s.category.name(),
            &result,
            s.expect_ids.len(),
            threshold,
        );
        if !quiet || !r.passed {
            reports.push(r);
        }
    }

    let full = FullReport::from_scenarios(reports);

    match format {
        "json" => println!("{}", report::to_json(&full)),
        _ => report::print_terminal(&full),
    }
}

fn cmd_list(category: Option<String>) {
    let scenarios = filter_scenarios(category, None);
    println!("{:<35} {:<15} Description", "Name", "Category");
    println!("{}", "-".repeat(80));
    for s in &scenarios {
        println!("{:<35} {:<15} {}", s.name, s.category.name(), s.description);
    }
    println!("\nTotal: {} scenarios", scenarios.len());
}

fn cmd_regression(category: Option<String>) {
    let scenarios = filter_scenarios(category, None);

    let mut reports = Vec::new();
    for s in &scenarios {
        let (result, _) = run_scenario(s);
        reports.push(report::scenario_report(
            &s.name,
            s.category.name(),
            &result,
            s.expect_ids.len(),
            s.max_corner_rmse,
        ));
    }

    let full = FullReport::from_scenarios(reports);
    report::print_terminal(&full);

    if !full.all_passed() {
        std::process::exit(1);
    }
}

#[allow(clippy::too_many_arguments)]
fn cmd_explore(
    family_name: &str,
    tag_id: u32,
    tag_size: f64,
    rotation_deg: f64,
    tilt_x_deg: f64,
    tilt_y_deg: f64,
    noise_sigma: f64,
    blur_sigma: f64,
    contrast: f64,
    width: u32,
    height: u32,
    format: &str,
) {
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
        .add_tag(family_name, tag_id, transform)
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

    // Detect
    let mut detector = Detector::new(DetectorConfig::default());
    if let Some(fam) = family::builtin_family(family_name) {
        detector.add_family(fam, 2);
    }

    let start = Instant::now();
    let detections = detector.detect(&scene.image);
    let elapsed = start.elapsed();

    let result = metrics::evaluate(&scene.ground_truth, &detections, elapsed.as_micros() as u64);
    let r = report::scenario_report(
        "explore",
        "explore",
        &result,
        scene.ground_truth.len(),
        f64::INFINITY,
    );

    match format {
        "json" => {
            println!("{}", serde_json::to_string_pretty(&r).unwrap());
        }
        _ => {
            println!(
                "Scene: {}x{}, family={}, id={}",
                width, height, family_name, tag_id
            );
            println!(
                "Transform: size={}, rotation={}°, tilt_x={}°, tilt_y={}°",
                tag_size, rotation_deg, tilt_x_deg, tilt_y_deg
            );
            if noise_sigma > 0.0 || blur_sigma > 0.0 || contrast != 1.0 {
                println!(
                    "Distortions: noise={}, blur={}, contrast={}",
                    noise_sigma, blur_sigma, contrast
                );
            }
            println!();
            println!("Detections: {}", detections.len());
            println!("Detection rate: {:.0}%", result.detection_rate * 100.0);
            println!("Corner RMSE: {:.2} px", result.corner_rmse);
            println!("Max corner error: {:.2} px", result.max_corner_error);
            println!("Detection time: {:.1} ms", elapsed.as_secs_f64() * 1000.0);

            for det in &detections {
                println!(
                    "  Tag {}: hamming={}, margin={:.1}, center=({:.1}, {:.1})",
                    det.id, det.hamming, det.decision_margin, det.center[0], det.center[1]
                );
            }
        }
    }
}
