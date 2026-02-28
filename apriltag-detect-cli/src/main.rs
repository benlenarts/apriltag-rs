use anyhow::{Context, Result};
use clap::Parser;
use serde::Serialize;

use apriltag::detect::detector::{Detector, DetectorConfig};
use apriltag::detect::image::ImageU8;
use apriltag::detect::pose::{estimate_tag_pose, Pose, PoseParams};
use apriltag::detect::quad::QuadThreshParams;
use apriltag::family;

/// AprilTag detection CLI — detect tags in PNG/JPEG images
#[derive(Parser)]
#[command(name = "apriltag-detect", version)]
struct Args {
    /// Input image files (PNG or JPEG)
    #[arg(required = true)]
    images: Vec<String>,

    /// Tag family to detect (comma-separated for multiple)
    #[arg(short, long, default_value = "tag36h11")]
    family: String,

    /// Decimation factor for input image
    #[arg(short = 'd', long, default_value = "2.0")]
    decimate: f32,

    /// Gaussian blur sigma (0 = no blur, negative = sharpen)
    #[arg(short = 'b', long, default_value = "0.0")]
    blur: f32,

    /// Decode sharpening factor
    #[arg(short = 's', long, default_value = "0.25")]
    sharpening: f64,

    /// Maximum Hamming distance for tag matching
    #[arg(long, default_value = "2")]
    max_hamming: u32,

    /// Disable edge refinement
    #[arg(long)]
    no_refine: bool,

    /// Pretty-print JSON output
    #[arg(long)]
    pretty: bool,

    /// Suppress non-JSON output
    #[arg(short, long)]
    quiet: bool,

    /// Enable pose estimation (requires camera parameters)
    #[arg(long)]
    pose: bool,

    /// Tag size in meters (for pose estimation)
    #[arg(long)]
    tag_size: Option<f64>,

    /// Camera focal length x in pixels
    #[arg(long)]
    fx: Option<f64>,

    /// Camera focal length y in pixels
    #[arg(long)]
    fy: Option<f64>,

    /// Camera principal point x in pixels
    #[arg(long)]
    cx: Option<f64>,

    /// Camera principal point y in pixels
    #[arg(long)]
    cy: Option<f64>,
}

#[derive(Serialize)]
struct OutputResult {
    file: String,
    image_width: u32,
    image_height: u32,
    detections: Vec<OutputDetection>,
}

#[derive(Serialize)]
struct OutputDetection {
    family: String,
    id: i32,
    hamming: i32,
    decision_margin: f32,
    center: [f64; 2],
    corners: [[f64; 2]; 4],
    #[serde(skip_serializing_if = "Option::is_none")]
    pose: Option<OutputPose>,
}

#[derive(Serialize)]
struct OutputPose {
    rotation: Vec<f64>,
    translation: [f64; 3],
    error: f64,
}

fn load_image(path: &str) -> Result<ImageU8> {
    let img = image::open(path)
        .with_context(|| format!("failed to open image: {path}"))?
        .into_luma8();

    let width = img.width();
    let height = img.height();
    let pixels = img.into_raw();

    Ok(ImageU8::from_buf(width, height, width, pixels))
}

fn pose_from_result(pose: &Pose, error: f64) -> OutputPose {
    let rotation = vec![
        pose.r[0][0],
        pose.r[0][1],
        pose.r[0][2],
        pose.r[1][0],
        pose.r[1][1],
        pose.r[1][2],
        pose.r[2][0],
        pose.r[2][1],
        pose.r[2][2],
    ];
    OutputPose {
        rotation,
        translation: pose.t,
        error,
    }
}

fn main() -> Result<()> {
    let args = Args::parse();

    // Validate pose parameters
    let pose_params = if args.pose {
        let tag_size = args
            .tag_size
            .context("--tag-size is required when --pose is set")?;
        let fx = args.fx.context("--fx is required when --pose is set")?;
        let fy = args.fy.context("--fy is required when --pose is set")?;
        let cx = args.cx.context("--cx is required when --pose is set")?;
        let cy = args.cy.context("--cy is required when --pose is set")?;
        Some(PoseParams {
            tagsize: tag_size,
            fx,
            fy,
            cx,
            cy,
        })
    } else {
        None
    };

    // Build detector
    let config = DetectorConfig {
        quad_decimate: args.decimate,
        quad_sigma: args.blur,
        refine_edges: !args.no_refine,
        decode_sharpening: args.sharpening,
        qtp: QuadThreshParams::default(),
    };
    let mut detector = Detector::new(config);

    // Add families
    for family_name in args.family.split(',') {
        let family_name = family_name.trim();
        let fam = family::builtin_family(family_name)
            .with_context(|| format!("unknown tag family: {family_name}"))?;
        detector.add_family(fam, args.max_hamming);
    }

    // Process each image
    for image_path in &args.images {
        let img = load_image(image_path)?;

        if !args.quiet {
            eprintln!("detecting in {} ({}x{})", image_path, img.width, img.height);
        }

        let detections = detector.detect(&img);

        let output_detections: Vec<OutputDetection> = detections
            .iter()
            .map(|det| {
                let pose = pose_params.as_ref().map(|params| {
                    let (pose1, err1, pose2, err2) = estimate_tag_pose(det, params);
                    // Pick the better pose
                    if let Some(p2) = pose2 {
                        if err2 < err1 {
                            return pose_from_result(&p2, err2);
                        }
                    }
                    pose_from_result(&pose1, err1)
                });

                OutputDetection {
                    family: det.family_name.clone(),
                    id: det.id,
                    hamming: det.hamming,
                    decision_margin: det.decision_margin,
                    center: det.center,
                    corners: det.corners,
                    pose,
                }
            })
            .collect();

        if !args.quiet {
            eprintln!("  found {} tags", output_detections.len());
        }

        let result = OutputResult {
            file: image_path.clone(),
            image_width: img.width,
            image_height: img.height,
            detections: output_detections,
        };

        let json = if args.pretty {
            serde_json::to_string_pretty(&result)?
        } else {
            serde_json::to_string(&result)?
        };
        println!("{json}");
    }

    Ok(())
}
