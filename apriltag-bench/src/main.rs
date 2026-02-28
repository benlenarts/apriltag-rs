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
    /// Benchmark detection performance: Rust vs C reference (requires --features reference).
    Benchmark {
        /// Filter by category name.
        #[arg(long)]
        category: Option<String>,
        /// Filter by scenario name pattern (substring match).
        #[arg(long)]
        scenario: Option<String>,
        /// Number of iterations per scenario.
        #[arg(long, default_value_t = 10)]
        iterations: usize,
        /// Output format: terminal, json.
        #[arg(long, default_value = "terminal")]
        format: String,
    },
    /// Run a comprehensive benchmark sweep: many tags × distortion conditions (requires --features reference).
    BenchmarkSweep {
        /// Number of iterations per scenario.
        #[arg(long, default_value_t = 10)]
        iterations: usize,
        /// Output format: terminal, json.
        #[arg(long, default_value = "terminal")]
        format: String,
    },
    /// Compare Rust detector vs C reference (requires --features reference).
    Compare {
        /// Filter by category name.
        #[arg(long)]
        category: Option<String>,
        /// Filter by scenario name pattern (substring match).
        #[arg(long)]
        scenario: Option<String>,
        /// Output format: terminal, json.
        #[arg(long, default_value = "terminal")]
        format: String,
    },
    /// Generate test images for all scenarios and save to output directory.
    GenerateImages {
        /// Filter by category name.
        #[arg(long)]
        category: Option<String>,
        /// Filter by scenario name pattern (substring match).
        #[arg(long)]
        scenario: Option<String>,
        /// Output directory for generated images.
        #[arg(long, default_value = "output")]
        output: String,
    },
    /// Start a local HTTP server for the web UI.
    Serve {
        /// Port to listen on.
        #[arg(long, default_value_t = 8080)]
        port: u16,
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
        Command::Benchmark {
            category,
            scenario,
            iterations,
            format,
        } => cmd_benchmark(category, scenario, iterations, &format),
        Command::BenchmarkSweep { iterations, format } => cmd_benchmark_sweep(iterations, &format),
        Command::Compare {
            category,
            scenario,
            format,
        } => cmd_compare(category, scenario, &format),
        Command::GenerateImages {
            category,
            scenario,
            output,
        } => cmd_generate_images(category, scenario, &output),
        Command::Serve { port } => cmd_serve(port),
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

fn cmd_generate_images(category: Option<String>, scenario: Option<String>, output_dir: &str) {
    let scenarios = filter_scenarios(category, scenario);
    let out = std::path::Path::new(output_dir);
    std::fs::create_dir_all(out).unwrap_or_else(|e| panic!("cannot create {output_dir}: {e}"));

    for s in &scenarios {
        let scene = s.build();
        let img = &scene.image;

        // Write PGM (Portable GrayMap) — simple, no external deps
        let filename = format!("{}.pgm", s.name);
        let path = out.join(&filename);

        // PGM binary: write header as text, pixel data as raw bytes
        let header = format!("P5\n{} {}\n255\n", img.width, img.height);
        let mut file_data = header.into_bytes();
        for y in 0..img.height {
            let row_start = (y * img.stride) as usize;
            let row_end = row_start + img.width as usize;
            file_data.extend_from_slice(&img.buf[row_start..row_end]);
        }
        std::fs::write(&path, &file_data)
            .unwrap_or_else(|e| panic!("cannot write {}: {e}", path.display()));

        // Also write ground truth as JSON sidecar
        let gt_filename = format!("{}.json", s.name);
        let gt_path = out.join(&gt_filename);
        let gt_json = serde_json::to_string_pretty(&scene.ground_truth)
            .unwrap_or_else(|e| panic!("cannot serialize ground truth: {e}"));
        std::fs::write(&gt_path, gt_json)
            .unwrap_or_else(|e| panic!("cannot write {}: {e}", gt_path.display()));

        println!("  {} ({:>4}x{:<4})", filename, img.width, img.height);
    }

    println!("\nGenerated {} images in {output_dir}/", scenarios.len());
}

fn cmd_serve(port: u16) {
    // Serve the web UI from the project root so that both ui/ and WASM pkg/ dirs are accessible
    let ui_dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR"));
    let project_root = ui_dir.parent().unwrap_or(ui_dir);

    println!();
    println!("  => http://localhost:{port}/apriltag-bench/ui/");
    println!();
    println!("Press Ctrl+C to stop.");

    let status = std::process::Command::new("python3")
        .args([
            "-m",
            "http.server",
            &port.to_string(),
            "--bind",
            "127.0.0.1",
        ])
        .current_dir(project_root)
        .status();

    match status {
        Ok(s) if !s.success() => {
            eprintln!("Server exited with status: {s}");
        }
        Err(_) => {
            eprintln!("Could not start python3 http.server.");
            eprintln!("You can manually serve the project root:");
            eprintln!(
                "  cd {} && python3 -m http.server {port}",
                project_root.display()
            );
        }
        _ => {}
    }
}

fn cmd_benchmark(
    category: Option<String>,
    scenario: Option<String>,
    iterations: usize,
    format: &str,
) {
    #[cfg(not(feature = "reference"))]
    {
        let _ = (category, scenario, iterations, format);
        eprintln!("Error: the 'benchmark' command requires the 'reference' feature.");
        eprintln!("Build with: cargo run -p apriltag-bench --features reference -- benchmark");
        eprintln!("Make sure to run scripts/fetch-references.sh first.");
        std::process::exit(1);
    }

    #[cfg(feature = "reference")]
    {
        use apriltag_bench::reference::{PersistentReferenceDetector, ReferenceConfig};

        let scenarios = filter_scenarios(category, scenario);

        #[derive(serde::Serialize)]
        struct BenchRow {
            name: String,
            image_size: [u32; 2],
            rust_median_us: u64,
            ref_median_us: u64,
            ratio: f64,
        }

        let mut rows = Vec::new();

        if format != "json" {
            println!(
                "{:<35} {:>10} {:>10} {:>10} {:>8}",
                "Scenario", "Rust(ms)", "Ref(ms)", "Ratio", "Size"
            );
            println!("{}", "-".repeat(78));
        }

        for s in &scenarios {
            let scene = s.build();
            let img = &scene.image;
            let size = [img.width, img.height];

            // Collect unique families for this scenario
            let families: Vec<&str> = s
                .expect_ids
                .iter()
                .map(|(f, _)| f.as_str())
                .collect::<std::collections::HashSet<_>>()
                .into_iter()
                .collect();

            // Create Rust detector (persistent across iterations)
            let mut rust_config = DetectorConfig::default();
            if let Some(decimate) = s.quad_decimate {
                rust_config.quad_decimate = decimate;
            }
            let mut rust_detector = Detector::new(rust_config);
            for fam_name in &families {
                if let Some(fam) = family::builtin_family(fam_name) {
                    rust_detector.add_family(fam, 2);
                }
            }

            // Create C reference detector (persistent across iterations)
            let ref_config = ReferenceConfig {
                quad_decimate: s.quad_decimate.unwrap_or(2.0),
                ..Default::default()
            };
            // Use first family for persistent detector (most scenarios use one family)
            let ref_detector = PersistentReferenceDetector::new(families[0], &ref_config);

            // Warmup run
            let _ = rust_detector.detect(&scene.image);
            let _ = ref_detector.detect(&scene.image);

            // Benchmark Rust detector
            let mut rust_times = Vec::with_capacity(iterations);
            for _ in 0..iterations {
                let start = Instant::now();
                let _ = rust_detector.detect(&scene.image);
                rust_times.push(start.elapsed());
            }

            // Benchmark C reference detector
            let mut ref_times = Vec::with_capacity(iterations);
            for _ in 0..iterations {
                let start = Instant::now();
                let _ = ref_detector.detect(&scene.image);
                ref_times.push(start.elapsed());
            }

            rust_times.sort();
            ref_times.sort();

            let rust_median = rust_times[iterations / 2];
            let ref_median = ref_times[iterations / 2];

            let rust_us = rust_median.as_micros() as u64;
            let ref_us = ref_median.as_micros() as u64;
            let ratio = if ref_us > 0 {
                rust_us as f64 / ref_us as f64
            } else {
                0.0
            };

            if format != "json" {
                println!(
                    "{:<35} {:>9.1} {:>9.1} {:>9.2}x {:>4}x{:<4}",
                    &s.name,
                    rust_us as f64 / 1000.0,
                    ref_us as f64 / 1000.0,
                    ratio,
                    size[0],
                    size[1],
                );
            }

            rows.push(BenchRow {
                name: s.name.clone(),
                image_size: size,
                rust_median_us: rust_us,
                ref_median_us: ref_us,
                ratio,
            });
        }

        if format == "json" {
            println!("{}", serde_json::to_string_pretty(&rows).unwrap());
        } else {
            println!("{}", "-".repeat(78));

            // Summary statistics
            let total_rust: u64 = rows.iter().map(|r| r.rust_median_us).sum();
            let total_ref: u64 = rows.iter().map(|r| r.ref_median_us).sum();
            let overall_ratio = if total_ref > 0 {
                total_rust as f64 / total_ref as f64
            } else {
                0.0
            };
            println!(
                "{:<35} {:>9.1} {:>9.1} {:>9.2}x",
                "TOTAL",
                total_rust as f64 / 1000.0,
                total_ref as f64 / 1000.0,
                overall_ratio,
            );
            println!(
                "\n{} scenarios, {} iterations each (median times shown)",
                rows.len(),
                iterations
            );
        }
    }
}

fn cmd_benchmark_sweep(iterations: usize, format: &str) {
    #[cfg(not(feature = "reference"))]
    {
        let _ = (iterations, format);
        eprintln!("Error: the 'benchmark-sweep' command requires the 'reference' feature.");
        eprintln!(
            "Build with: cargo run -p apriltag-bench --features reference -- benchmark-sweep"
        );
        eprintln!("Make sure to run scripts/fetch-references.sh first.");
        std::process::exit(1);
    }

    #[cfg(feature = "reference")]
    {
        use apriltag_bench::reference::{PersistentReferenceDetector, ReferenceConfig};

        #[derive(serde::Serialize)]
        struct BenchRow {
            name: String,
            tags: usize,
            condition: String,
            image_size: [u32; 2],
            rust_median_us: u64,
            ref_median_us: u64,
            ratio: f64,
        }

        struct SweepScene {
            name: String,
            tags: usize,
            condition: String,
            scene: apriltag_bench::scene::Scene,
        }

        // Tag counts and corresponding image sizes / tag scale
        let tag_configs: &[(usize, u32, u32, f64)] = &[
            // (n_tags, width, height, tag_scale)
            (1, 500, 500, 80.0),
            (5, 800, 600, 50.0),
            (10, 1000, 800, 45.0),
            (25, 1280, 960, 35.0),
        ];

        // Distortion conditions
        struct Condition {
            name: &'static str,
            rotation_deg: f64,
            tilt_x_deg: f64,
            distortions: Vec<Distortion>,
        }

        let conditions = vec![
            Condition {
                name: "clean",
                rotation_deg: 0.0,
                tilt_x_deg: 0.0,
                distortions: vec![],
            },
            Condition {
                name: "rotation-30",
                rotation_deg: 30.0,
                tilt_x_deg: 0.0,
                distortions: vec![],
            },
            Condition {
                name: "tilt-20",
                rotation_deg: 0.0,
                tilt_x_deg: 20.0,
                distortions: vec![],
            },
            Condition {
                name: "noise-20",
                rotation_deg: 0.0,
                tilt_x_deg: 0.0,
                distortions: vec![Distortion::GaussianNoise {
                    sigma: 20.0,
                    seed: 42,
                }],
            },
            Condition {
                name: "blur-2",
                rotation_deg: 0.0,
                tilt_x_deg: 0.0,
                distortions: vec![Distortion::GaussianBlur { sigma: 2.0 }],
            },
            Condition {
                name: "contrast-25",
                rotation_deg: 0.0,
                tilt_x_deg: 0.0,
                distortions: vec![Distortion::ContrastScale { factor: 0.25 }],
            },
            Condition {
                name: "combined",
                rotation_deg: 15.0,
                tilt_x_deg: 15.0,
                distortions: vec![
                    Distortion::GaussianNoise {
                        sigma: 10.0,
                        seed: 42,
                    },
                    Distortion::GaussianBlur { sigma: 1.0 },
                ],
            },
        ];

        // Generate all sweep scenes
        let mut sweep_scenes = Vec::new();

        for &(n_tags, width, height, tag_scale) in tag_configs {
            // Compute grid positions for tags
            let positions = grid_positions(n_tags, width, height, tag_scale);

            for cond in &conditions {
                let name = format!("{}tags-{}", n_tags, cond.name);

                let mut builder =
                    SceneBuilder::new(width, height).background(Background::Solid(128));

                for (id, &(cx, cy)) in positions.iter().enumerate() {
                    let transform =
                        if cond.tilt_x_deg.abs() > 0.01 || cond.rotation_deg.abs() > 0.01 {
                            if cond.tilt_x_deg.abs() > 0.01 {
                                Transform::FromPose {
                                    center: [cx, cy],
                                    size: tag_scale,
                                    roll: cond.rotation_deg.to_radians(),
                                    tilt_x: cond.tilt_x_deg.to_radians(),
                                    tilt_y: 0.0,
                                }
                            } else {
                                Transform::Similarity {
                                    cx,
                                    cy,
                                    scale: tag_scale / 2.0,
                                    theta: cond.rotation_deg.to_radians(),
                                }
                            }
                        } else {
                            Transform::Similarity {
                                cx,
                                cy,
                                scale: tag_scale / 2.0,
                                theta: 0.0,
                            }
                        };

                    builder = builder.add_tag("tag36h11", id as u32, transform);
                }

                let mut scene = builder.build();

                if !cond.distortions.is_empty() {
                    distortion::apply(&mut scene.image, &cond.distortions);
                }

                sweep_scenes.push(SweepScene {
                    name,
                    tags: n_tags,
                    condition: cond.name.to_string(),
                    scene,
                });
            }
        }

        // Run benchmarks
        let mut rows = Vec::new();
        let ref_config = ReferenceConfig::default();
        let ref_detector = PersistentReferenceDetector::new("tag36h11", &ref_config);

        let mut rust_detector = Detector::new(DetectorConfig::default());
        if let Some(fam) = family::builtin_family("tag36h11") {
            rust_detector.add_family(fam, 2);
        }

        if format != "json" {
            println!(
                "{:<30} {:>5} {:>10} {:>10} {:>10} {:>10}",
                "Scenario", "Tags", "Rust(ms)", "Ref(ms)", "Ratio", "Size"
            );
            println!("{}", "-".repeat(80));
        }

        for ss in &sweep_scenes {
            let img = &ss.scene.image;
            let size = [img.width, img.height];

            // Warmup
            let _ = rust_detector.detect(img);
            let _ = ref_detector.detect(img);

            // Benchmark Rust
            let mut rust_times = Vec::with_capacity(iterations);
            for _ in 0..iterations {
                let start = Instant::now();
                let _ = rust_detector.detect(img);
                rust_times.push(start.elapsed());
            }

            // Benchmark C reference
            let mut ref_times = Vec::with_capacity(iterations);
            for _ in 0..iterations {
                let start = Instant::now();
                let _ = ref_detector.detect(img);
                ref_times.push(start.elapsed());
            }

            rust_times.sort();
            ref_times.sort();

            let rust_us = rust_times[iterations / 2].as_micros() as u64;
            let ref_us = ref_times[iterations / 2].as_micros() as u64;
            let ratio = if ref_us > 0 {
                rust_us as f64 / ref_us as f64
            } else {
                0.0
            };

            if format != "json" {
                println!(
                    "{:<30} {:>5} {:>9.1} {:>9.1} {:>9.2}x {:>4}x{:<4}",
                    &ss.name,
                    ss.tags,
                    rust_us as f64 / 1000.0,
                    ref_us as f64 / 1000.0,
                    ratio,
                    size[0],
                    size[1],
                );
            }

            rows.push(BenchRow {
                name: ss.name.clone(),
                tags: ss.tags,
                condition: ss.condition.clone(),
                image_size: size,
                rust_median_us: rust_us,
                ref_median_us: ref_us,
                ratio,
            });
        }

        if format == "json" {
            println!("{}", serde_json::to_string_pretty(&rows).unwrap());
        } else {
            println!("{}", "-".repeat(80));

            // Per-condition summary
            println!("\nPer-condition averages:");
            let cond_names: Vec<String> = conditions.iter().map(|c| c.name.to_string()).collect();
            for cond_name in &cond_names {
                let cond_rows: Vec<_> = rows.iter().filter(|r| r.condition == *cond_name).collect();
                let total_rust: u64 = cond_rows.iter().map(|r| r.rust_median_us).sum();
                let total_ref: u64 = cond_rows.iter().map(|r| r.ref_median_us).sum();
                let ratio = if total_ref > 0 {
                    total_rust as f64 / total_ref as f64
                } else {
                    0.0
                };
                println!(
                    "  {:<20} {:>9.1} vs {:>9.1} ms  ({:.2}x)",
                    cond_name,
                    total_rust as f64 / 1000.0,
                    total_ref as f64 / 1000.0,
                    ratio,
                );
            }

            // Per-tag-count summary
            println!("\nPer-tag-count averages:");
            for &(n_tags, _, _, _) in tag_configs {
                let tag_rows: Vec<_> = rows.iter().filter(|r| r.tags == n_tags).collect();
                let total_rust: u64 = tag_rows.iter().map(|r| r.rust_median_us).sum();
                let total_ref: u64 = tag_rows.iter().map(|r| r.ref_median_us).sum();
                let ratio = if total_ref > 0 {
                    total_rust as f64 / total_ref as f64
                } else {
                    0.0
                };
                println!(
                    "  {:>3} tags             {:>9.1} vs {:>9.1} ms  ({:.2}x)",
                    n_tags,
                    total_rust as f64 / 1000.0,
                    total_ref as f64 / 1000.0,
                    ratio,
                );
            }

            // Overall total
            let total_rust: u64 = rows.iter().map(|r| r.rust_median_us).sum();
            let total_ref: u64 = rows.iter().map(|r| r.ref_median_us).sum();
            let overall_ratio = if total_ref > 0 {
                total_rust as f64 / total_ref as f64
            } else {
                0.0
            };
            println!(
                "\nOVERALL: {:.1} vs {:.1} ms ({:.2}x), {} scenarios, {} iterations each",
                total_rust as f64 / 1000.0,
                total_ref as f64 / 1000.0,
                overall_ratio,
                rows.len(),
                iterations,
            );
        }
    }
}

/// Compute grid positions for N tags within an image, with spacing for borders.
#[cfg(feature = "reference")]
fn grid_positions(n: usize, width: u32, height: u32, tag_scale: f64) -> Vec<(f64, f64)> {
    if n == 1 {
        return vec![(width as f64 / 2.0, height as f64 / 2.0)];
    }

    // Compute grid dimensions
    let cols = (n as f64).sqrt().ceil() as usize;
    let rows = n.div_ceil(cols);

    let margin = tag_scale * 1.2;
    let usable_w = width as f64 - 2.0 * margin;
    let usable_h = height as f64 - 2.0 * margin;

    let step_x = if cols > 1 {
        usable_w / (cols - 1) as f64
    } else {
        0.0
    };
    let step_y = if rows > 1 {
        usable_h / (rows - 1) as f64
    } else {
        0.0
    };

    let mut positions = Vec::with_capacity(n);
    for i in 0..n {
        let col = i % cols;
        let row = i / cols;
        let x = margin + col as f64 * step_x;
        let y = margin + row as f64 * step_y;
        positions.push((x, y));
    }
    positions
}

fn cmd_compare(category: Option<String>, scenario: Option<String>, format: &str) {
    #[cfg(not(feature = "reference"))]
    {
        let _ = (category, scenario, format);
        eprintln!("Error: the 'compare' command requires the 'reference' feature.");
        eprintln!("Build with: cargo run -p apriltag-bench --features reference -- compare");
        eprintln!("Make sure to run scripts/fetch-references.sh first.");
        std::process::exit(1);
    }

    #[cfg(feature = "reference")]
    {
        use apriltag_bench::reference::{self, ReferenceConfig};

        let scenarios = filter_scenarios(category, scenario);

        println!(
            "{:<35} {:>8} {:>8} {:>8} {:>8} {:>8}",
            "Scenario", "Rust%", "Ref%", "RustRMS", "RefRMS", "Match"
        );
        println!("{}", "-".repeat(85));

        #[derive(serde::Serialize)]
        struct CompareRow {
            name: String,
            rust_detection_rate: f64,
            ref_detection_rate: f64,
            rust_corner_rmse: f64,
            ref_corner_rmse: f64,
            results_match: bool,
        }

        let mut rows = Vec::new();

        for s in &scenarios {
            let scene = s.build();

            // Run Rust detector
            let (rust_result, _) = run_scenario(s);

            // Run C reference detector
            let families: Vec<&str> = s
                .expect_ids
                .iter()
                .map(|(f, _)| f.as_str())
                .collect::<std::collections::HashSet<_>>()
                .into_iter()
                .collect();

            let ref_config = ReferenceConfig {
                quad_decimate: s.quad_decimate.unwrap_or(2.0),
                ..Default::default()
            };

            let mut all_ref_dets = Vec::new();
            for fam in &families {
                let dets = reference::reference_detect(&scene.image, fam, &ref_config);
                for d in dets {
                    // The C reference returns corners as [BL, BR, TR, TL]
                    // (tag-space (-1,1), (1,1), (1,-1), (-1,-1)) while our
                    // ground truth uses [TL, TR, BR, BL] (tag-space (-1,-1),
                    // (1,-1), (1,1), (-1,1)). Reverse to match our convention.
                    let c = d.corners;
                    let corners = [c[3], c[2], c[1], c[0]];
                    all_ref_dets.push(apriltag::detect::detector::Detection {
                        id: d.id,
                        hamming: d.hamming,
                        decision_margin: d.decision_margin,
                        center: d.center,
                        corners,
                        family_name: fam.to_string(),
                    });
                }
            }

            let ref_result = metrics::evaluate(&scene.ground_truth, &all_ref_dets, 0);

            let results_match =
                (rust_result.detection_rate - ref_result.detection_rate).abs() < 0.01;

            let row = CompareRow {
                name: s.name.clone(),
                rust_detection_rate: rust_result.detection_rate,
                ref_detection_rate: ref_result.detection_rate,
                rust_corner_rmse: rust_result.corner_rmse,
                ref_corner_rmse: ref_result.corner_rmse,
                results_match,
            };

            if format != "json" {
                let match_str = if results_match { "YES" } else { "NO" };
                println!(
                    "{:<35} {:>7.0}% {:>7.0}% {:>8.2} {:>8.2} {:>8}",
                    &s.name,
                    rust_result.detection_rate * 100.0,
                    ref_result.detection_rate * 100.0,
                    rust_result.corner_rmse,
                    ref_result.corner_rmse,
                    match_str,
                );
            }

            rows.push(row);
        }

        if format == "json" {
            println!("{}", serde_json::to_string_pretty(&rows).unwrap());
        } else {
            println!("{}", "-".repeat(85));
            let matching = rows.iter().filter(|r| r.results_match).count();
            println!("Matching: {}/{} scenarios", matching, rows.len());
        }
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
