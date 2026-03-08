/// Pre-defined test scenarios for detection quality evaluation.
use crate::distortion::Distortion;
use crate::scene::{Background, Scene, SceneBuilder};
use crate::transform::Transform;

/// A category of test scenarios.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Category {
    Baseline,
    Rotation,
    Perspective,
    Scale,
    Noise,
    Contrast,
    Lighting,
    Blur,
    MultiTag,
    Occlusion,
    Decimation,
    Highres,
    Scaling,
}

impl Category {
    pub fn all() -> &'static [Category] {
        &[
            Category::Baseline,
            Category::Rotation,
            Category::Perspective,
            Category::Scale,
            Category::Noise,
            Category::Contrast,
            Category::Lighting,
            Category::Blur,
            Category::MultiTag,
            Category::Occlusion,
            Category::Decimation,
            Category::Highres,
            Category::Scaling,
        ]
    }

    pub fn name(&self) -> &'static str {
        match self {
            Category::Baseline => "baseline",
            Category::Rotation => "rotation",
            Category::Perspective => "perspective",
            Category::Scale => "scale",
            Category::Noise => "noise",
            Category::Contrast => "contrast",
            Category::Lighting => "lighting",
            Category::Blur => "blur",
            Category::MultiTag => "multi-tag",
            Category::Occlusion => "occlusion",
            Category::Decimation => "decimation",
            Category::Highres => "highres",
            Category::Scaling => "scaling",
        }
    }

    pub fn from_name(name: &str) -> Option<Category> {
        Category::all().iter().find(|c| c.name() == name).copied()
    }
}

/// A test scenario that generates a scene and specifies expected results.
pub struct Scenario {
    pub name: String,
    pub description: String,
    pub category: Category,
    /// Expected (family, tag_id) pairs that should be detected.
    pub expect_ids: Vec<(String, u32)>,
    /// Maximum acceptable corner RMSE in pixels.
    pub max_corner_rmse: f64,
    /// Override detector config: quad_decimate value (None = use default).
    pub quad_decimate: Option<f32>,
    /// Build the scene.
    build_fn: Box<dyn Fn() -> Scene + Send + Sync>,
}

impl Scenario {
    pub fn build(&self) -> Scene {
        (self.build_fn)()
    }
}

/// Build the full catalog of test scenarios.
pub fn all_scenarios() -> Vec<Scenario> {
    let mut scenarios = Vec::new();
    scenarios.extend(baseline_scenarios());
    scenarios.extend(rotation_scenarios());
    scenarios.extend(perspective_scenarios());
    scenarios.extend(scale_scenarios());
    scenarios.extend(noise_scenarios());
    scenarios.extend(contrast_scenarios());
    scenarios.extend(lighting_scenarios());
    scenarios.extend(blur_scenarios());
    scenarios.extend(multi_tag_scenarios());
    scenarios.extend(occlusion_scenarios());
    scenarios.extend(decimation_scenarios());
    scenarios.extend(highres_scenarios());
    scenarios.extend(scaling_scenarios());
    scenarios
}

/// Filter scenarios by category.
pub fn scenarios_for_category(category: Category) -> Vec<Scenario> {
    all_scenarios()
        .into_iter()
        .filter(|s| s.category == category)
        .collect()
}

fn baseline_scenarios() -> Vec<Scenario> {
    let families = ["tag36h11", "tag16h5", "tag25h9"];
    families
        .iter()
        .map(|&fam| {
            let fam_owned = fam.to_string();
            Scenario {
                name: format!("baseline-{fam}"),
                description: format!("Single centered {fam} tag, no distortion"),
                category: Category::Baseline,
                expect_ids: vec![(fam.to_string(), 0)],
                max_corner_rmse: 2.0,
                quad_decimate: None,
                build_fn: Box::new(move || {
                    SceneBuilder::new(300, 300)
                        .background(Background::Solid(128))
                        .add_tag(
                            &fam_owned,
                            0,
                            Transform::Similarity {
                                cx: 150.0,
                                cy: 150.0,
                                scale: 50.0,
                                theta: 0.0,
                            },
                        )
                        .build()
                }),
            }
        })
        .collect()
}

fn rotation_scenarios() -> Vec<Scenario> {
    let angles_deg = [0, 10, 15, 20, 25, 30, 45, 60, 70, 75, 80, 90];
    angles_deg
        .iter()
        .map(|&deg| {
            let theta = (deg as f64).to_radians();
            Scenario {
                name: format!("rotation-{deg}deg"),
                description: format!("Tag rotated {deg} degrees"),
                category: Category::Rotation,
                expect_ids: vec![("tag36h11".to_string(), 0)],
                max_corner_rmse: 3.0,
                quad_decimate: None,
                build_fn: Box::new(move || {
                    SceneBuilder::new(500, 500)
                        .background(Background::Solid(128))
                        .add_tag(
                            "tag36h11",
                            0,
                            Transform::Similarity {
                                cx: 250.0,
                                cy: 250.0,
                                scale: 80.0,
                                theta,
                            },
                        )
                        .build()
                }),
            }
        })
        .collect()
}

fn perspective_scenarios() -> Vec<Scenario> {
    let tilts_deg = [10, 20, 30];
    tilts_deg
        .iter()
        .map(|&deg| {
            let tilt = (deg as f64).to_radians();
            Scenario {
                name: format!("perspective-tilt-{deg}deg"),
                description: format!("Tag with {deg}° perspective tilt"),
                category: Category::Perspective,
                expect_ids: vec![("tag36h11".to_string(), 0)],
                max_corner_rmse: 5.0,
                quad_decimate: None,
                build_fn: Box::new(move || {
                    SceneBuilder::new(500, 500)
                        .background(Background::Solid(128))
                        .add_tag(
                            "tag36h11",
                            0,
                            Transform::FromPose {
                                center: [250.0, 250.0],
                                size: 100.0,
                                roll: 0.0,
                                tilt_x: tilt,
                                tilt_y: 0.0,
                            },
                        )
                        .build()
                }),
            }
        })
        .collect()
}

fn scale_scenarios() -> Vec<Scenario> {
    let sizes = [16, 32, 64, 128, 200];
    sizes
        .iter()
        .map(|&size| {
            let scale = size as f64 / 2.0;
            let img_size = (size as u32 * 3).max(200);
            let center = img_size as f64 / 2.0;
            Scenario {
                name: format!("scale-{size}px"),
                description: format!("Tag at {size}px size"),
                category: Category::Scale,
                expect_ids: vec![("tag36h11".to_string(), 0)],
                max_corner_rmse: 3.0,
                quad_decimate: if size <= 32 { Some(1.0) } else { None },
                build_fn: Box::new(move || {
                    SceneBuilder::new(img_size, img_size)
                        .background(Background::Solid(128))
                        .add_tag(
                            "tag36h11",
                            0,
                            Transform::Similarity {
                                cx: center,
                                cy: center,
                                scale,
                                theta: 0.0,
                            },
                        )
                        .build()
                }),
            }
        })
        .collect()
}

fn noise_scenarios() -> Vec<Scenario> {
    let sigmas = [5, 10, 20, 40];
    let mut scenarios: Vec<Scenario> = sigmas
        .iter()
        .map(|&sigma| Scenario {
            name: format!("noise-sigma{sigma}"),
            description: format!("Gaussian noise sigma={sigma}"),
            category: Category::Noise,
            expect_ids: vec![("tag36h11".to_string(), 0)],
            max_corner_rmse: 5.0,
            quad_decimate: None,
            build_fn: Box::new(move || {
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
                crate::distortion::apply(
                    &mut scene.image,
                    &[Distortion::GaussianNoise {
                        sigma: sigma as f64,
                        seed: 42,
                    }],
                );
                scene
            }),
        })
        .collect();

    // Salt-and-pepper noise scenarios
    let densities = [0.05, 0.10];
    for &density in &densities {
        let label = format!("{:.0}pct", density * 100.0);
        scenarios.push(Scenario {
            name: format!("noise-saltpepper-{label}"),
            description: format!("Salt-and-pepper noise density={density:.0}%"),
            category: Category::Noise,
            expect_ids: vec![("tag36h11".to_string(), 0)],
            max_corner_rmse: 5.0,
            quad_decimate: None,
            build_fn: Box::new(move || {
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
                crate::distortion::apply(
                    &mut scene.image,
                    &[Distortion::SaltPepper { density, seed: 42 }],
                );
                scene
            }),
        });
    }

    scenarios
}

fn contrast_scenarios() -> Vec<Scenario> {
    let factors = [0.5, 0.25, 0.1];
    factors
        .iter()
        .map(|&factor| {
            let label = format!("{:.0}pct", factor * 100.0);
            Scenario {
                name: format!("contrast-{label}"),
                description: format!("Contrast scaled to {:.0}%", factor * 100.0),
                category: Category::Contrast,
                expect_ids: vec![("tag36h11".to_string(), 0)],
                max_corner_rmse: 3.0,
                quad_decimate: None,
                build_fn: Box::new(move || {
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
                    crate::distortion::apply(
                        &mut scene.image,
                        &[Distortion::ContrastScale { factor }],
                    );
                    scene
                }),
            }
        })
        .collect()
}

fn lighting_scenarios() -> Vec<Scenario> {
    let mut scenarios = vec![
        Scenario {
            name: "lighting-gradient-lr".to_string(),
            description: "Left-to-right gradient lighting (0.5–1.5×)".to_string(),
            category: Category::Lighting,
            expect_ids: vec![("tag36h11".to_string(), 0)],
            max_corner_rmse: 3.0,
            quad_decimate: None,
            build_fn: Box::new(|| {
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
                crate::distortion::apply(
                    &mut scene.image,
                    &[Distortion::GradientLighting {
                        direction: 0.0,
                        min_factor: 0.5,
                        max_factor: 1.5,
                    }],
                );
                scene
            }),
        },
        Scenario {
            name: "lighting-vignette".to_string(),
            description: "Vignette effect (strength=0.8)".to_string(),
            category: Category::Lighting,
            expect_ids: vec![("tag36h11".to_string(), 0)],
            max_corner_rmse: 3.0,
            quad_decimate: None,
            build_fn: Box::new(|| {
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
                crate::distortion::apply(
                    &mut scene.image,
                    &[Distortion::Vignette { strength: 0.8 }],
                );
                scene
            }),
        },
    ];

    // Brightness shift scenarios
    for &(offset, label) in &[(60i16, "bright+60"), (-80i16, "bright-80")] {
        scenarios.push(Scenario {
            name: format!("lighting-{label}"),
            description: format!("Brightness shift offset={offset}"),
            category: Category::Lighting,
            expect_ids: vec![("tag36h11".to_string(), 0)],
            max_corner_rmse: 3.0,
            quad_decimate: None,
            build_fn: Box::new(move || {
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
                crate::distortion::apply(
                    &mut scene.image,
                    &[Distortion::BrightnessShift { offset }],
                );
                scene
            }),
        });
    }

    scenarios
}

fn blur_scenarios() -> Vec<Scenario> {
    let sigmas = [1.0, 2.0, 4.0];
    sigmas
        .iter()
        .map(|&sigma| {
            let label = format!("{sigma:.0}");
            Scenario {
                name: format!("blur-sigma{label}"),
                description: format!("Gaussian blur sigma={sigma}"),
                category: Category::Blur,
                expect_ids: vec![("tag36h11".to_string(), 0)],
                max_corner_rmse: 5.0,
                quad_decimate: None,
                build_fn: Box::new(move || {
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
                    crate::distortion::apply(
                        &mut scene.image,
                        &[Distortion::GaussianBlur { sigma }],
                    );
                    scene
                }),
            }
        })
        .collect()
}

fn multi_tag_scenarios() -> Vec<Scenario> {
    vec![
        Scenario {
            name: "multi-2tags".to_string(),
            description: "Two tags side by side".to_string(),
            category: Category::MultiTag,
            expect_ids: vec![("tag36h11".to_string(), 0), ("tag36h11".to_string(), 1)],
            max_corner_rmse: 3.0,
            quad_decimate: None,
            build_fn: Box::new(|| {
                SceneBuilder::new(500, 300)
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
                    .build()
            }),
        },
        Scenario {
            name: "multi-5tags".to_string(),
            description: "Five tags in a grid".to_string(),
            category: Category::MultiTag,
            expect_ids: (0..5).map(|i| ("tag36h11".to_string(), i)).collect(),
            max_corner_rmse: 3.0,
            quad_decimate: None,
            build_fn: Box::new(|| {
                let positions = [
                    (100.0, 100.0),
                    (300.0, 100.0),
                    (500.0, 100.0),
                    (200.0, 250.0),
                    (400.0, 250.0),
                ];
                let mut builder = SceneBuilder::new(600, 350).background(Background::Solid(128));
                for (id, (cx, cy)) in positions.iter().enumerate() {
                    builder = builder.add_tag(
                        "tag36h11",
                        id as u32,
                        Transform::Similarity {
                            cx: *cx,
                            cy: *cy,
                            scale: 30.0,
                            theta: 0.0,
                        },
                    );
                }
                builder.build()
            }),
        },
    ]
}

fn highres_scenarios() -> Vec<Scenario> {
    // Place mixed tag36h11/tagStandard52h13 tags in a 4000×3000 grid with varied rotations and tilts
    let n_cols = 10;
    let n_rows = 7;
    let n_tags = n_cols * n_rows; // 70 tags
    let spacing_x = 4000.0 / (n_cols as f64 + 1.0);
    let spacing_y = 3000.0 / (n_rows as f64 + 1.0);
    let tag_size = 120.0; // pixels (full tag width)

    // Deterministic per-tag parameters via simple hash
    let tag_params: Vec<(f64, f64, f64)> = (0..n_tags)
        .map(|i| {
            let seed = (i as u32).wrapping_mul(2654435761);
            let roll_deg = ((seed % 360) as f64 - 180.0) * 10.0 / 180.0; // ±10°
            let roll_rad = roll_deg.to_radians();
            let tilt_x = (((seed >> 8) % 200) as f64 - 100.0) / 100.0 * 0.05; // ±0.05 rad (~3°)
            let tilt_y = (((seed >> 16) % 200) as f64 - 100.0) / 100.0 * 0.05; // ±0.05 rad (~3°)
            (roll_rad, tilt_x, tilt_y)
        })
        .collect();

    let expect_ids: Vec<(String, u32)> = (0..n_tags)
        .map(|i| {
            let family = if i % 2 == 0 {
                "tag36h11"
            } else {
                "tagStandard52h13"
            };
            (family.to_string(), i as u32)
        })
        .collect();

    vec![Scenario {
        name: "highres-4000x3000".to_string(),
        description: format!(
            "{n_tags} mixed tag36h11/tagStandard52h13 tags at 4000×3000 with rotation, perspective, noise, and lighting"
        ),
        category: Category::Highres,
        expect_ids,
        max_corner_rmse: 5.0,
        quad_decimate: None,
        build_fn: Box::new(move || {
            let mut builder = SceneBuilder::new(4000, 3000).background(Background::Solid(128));

            for row in 0..n_rows {
                for col in 0..n_cols {
                    let i = row * n_cols + col;
                    let cx = spacing_x * (col as f64 + 1.0);
                    let cy = spacing_y * (row as f64 + 1.0);
                    let (roll, tilt_x, tilt_y) = tag_params[i];

                    let family = if i % 2 == 0 {
                        "tag36h11"
                    } else {
                        "tagStandard52h13"
                    };
                    builder = builder.add_tag(
                        family,
                        i as u32,
                        Transform::FromPose {
                            center: [cx, cy],
                            size: tag_size,
                            roll,
                            tilt_x,
                            tilt_y,
                        },
                    );
                }
            }

            let mut scene = builder.build();
            crate::distortion::apply(
                &mut scene.image,
                &[
                    Distortion::ContrastScale { factor: 0.7 },
                    Distortion::GradientLighting {
                        direction: 0.3,
                        min_factor: 0.7,
                        max_factor: 1.3,
                    },
                    Distortion::GaussianNoise {
                        sigma: 15.0,
                        seed: 42,
                    },
                    Distortion::GaussianBlur { sigma: 0.8 },
                ],
            );
            scene
        }),
    }]
}

fn occlusion_scenarios() -> Vec<Scenario> {
    vec![Scenario {
        name: "occlusion-10pct".to_string(),
        description: "Tag with ~10% occluded by black rectangle".to_string(),
        category: Category::Occlusion,
        expect_ids: vec![("tag36h11".to_string(), 0)],
        max_corner_rmse: 5.0,
        quad_decimate: None,
        build_fn: Box::new(|| {
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
            // Occlude a small strip at the top of the tag
            crate::distortion::apply(
                &mut scene.image,
                &[Distortion::Occlude {
                    rect: [100, 100, 200, 115],
                }],
            );
            scene
        }),
    }]
}

fn decimation_scenarios() -> Vec<Scenario> {
    let decimations = [1.0_f32, 2.0, 4.0];
    decimations
        .iter()
        .map(|&decimate| Scenario {
            name: format!("decimation-{decimate:.0}x"),
            description: format!("Detection with quad_decimate={decimate}"),
            category: Category::Decimation,
            expect_ids: vec![("tag36h11".to_string(), 0)],
            max_corner_rmse: if decimate >= 4.0 { 5.0 } else { 3.0 },
            quad_decimate: Some(decimate),
            build_fn: Box::new(|| {
                SceneBuilder::new(400, 400)
                    .background(Background::Solid(128))
                    .add_tag(
                        "tag36h11",
                        0,
                        Transform::Similarity {
                            cx: 200.0,
                            cy: 200.0,
                            scale: 60.0,
                            theta: 0.0,
                        },
                    )
                    .build()
            }),
        })
        .collect()
}

fn scaling_scenarios() -> Vec<Scenario> {
    let mut scenarios = Vec::new();

    // (a) Image size sweep — isolates preprocessing, threshold, connected components, clustering
    let sizes: &[(u32, u32)] = &[(500, 500), (1000, 1000), (2000, 1500), (4000, 3000)];
    for &(w, h) in sizes {
        let tag_scale = (w.min(h) as f64) / 6.0; // tag fills ~1/3 of smaller dimension
        let cx = w as f64 / 2.0;
        let cy = h as f64 / 2.0;
        scenarios.push(Scenario {
            name: format!("scaling-size-{w}x{h}"),
            description: format!("Single tag36h11 at {w}×{h}, no noise"),
            category: Category::Scaling,
            expect_ids: vec![("tag36h11".to_string(), 0)],
            max_corner_rmse: 3.0,
            quad_decimate: None,
            build_fn: Box::new(move || {
                SceneBuilder::new(w, h)
                    .background(Background::Solid(128))
                    .add_tag(
                        "tag36h11",
                        0,
                        Transform::Similarity {
                            cx,
                            cy,
                            scale: tag_scale,
                            theta: 0.0,
                        },
                    )
                    .build()
            }),
        });
    }

    // (b) Noise at scale — isolates noise-induced clustering penalty
    let noise_sizes: &[(u32, u32)] = &[(500, 500), (1000, 1000), (2000, 1500)];
    for &(w, h) in noise_sizes {
        let tag_scale = (w.min(h) as f64) / 6.0;
        let cx = w as f64 / 2.0;
        let cy = h as f64 / 2.0;
        scenarios.push(Scenario {
            name: format!("scaling-noise-{w}x{h}"),
            description: format!("Single tag36h11 at {w}×{h}, noise sigma=15"),
            category: Category::Scaling,
            expect_ids: vec![("tag36h11".to_string(), 0)],
            max_corner_rmse: 5.0,
            quad_decimate: None,
            build_fn: Box::new(move || {
                let mut scene = SceneBuilder::new(w, h)
                    .background(Background::Solid(128))
                    .add_tag(
                        "tag36h11",
                        0,
                        Transform::Similarity {
                            cx,
                            cy,
                            scale: tag_scale,
                            theta: 0.0,
                        },
                    )
                    .build();
                crate::distortion::apply(
                    &mut scene.image,
                    &[Distortion::GaussianNoise {
                        sigma: 15.0,
                        seed: 42,
                    }],
                );
                scene
            }),
        });
    }

    // (c) Tag count at fixed size — isolates decode/quad-fitting/dedup scaling
    let tag_counts = [1, 10, 30];
    for &n_tags in &tag_counts {
        let expect_ids: Vec<(String, u32)> =
            (0..n_tags).map(|i| ("tag36h11".to_string(), i)).collect();
        scenarios.push(Scenario {
            name: format!("scaling-tags-{n_tags}"),
            description: format!("{n_tags} tag(s) at 2000×1500, no noise"),
            category: Category::Scaling,
            expect_ids,
            max_corner_rmse: 3.0,
            quad_decimate: None,
            build_fn: Box::new(move || {
                let (w, h) = (2000, 1500);
                let cols = (n_tags as f64).sqrt().ceil() as u32;
                let rows = n_tags.div_ceil(cols);
                let spacing_x = w as f64 / (cols as f64 + 1.0);
                let spacing_y = h as f64 / (rows as f64 + 1.0);
                let tag_scale = (spacing_x.min(spacing_y) * 0.35).min(80.0);

                let mut builder = SceneBuilder::new(w, h).background(Background::Solid(128));
                for i in 0..n_tags {
                    let col = i % cols;
                    let row = i / cols;
                    let cx = spacing_x * (col as f64 + 1.0);
                    let cy = spacing_y * (row as f64 + 1.0);
                    builder = builder.add_tag(
                        "tag36h11",
                        i,
                        Transform::Similarity {
                            cx,
                            cy,
                            scale: tag_scale,
                            theta: 0.0,
                        },
                    );
                }
                builder.build()
            }),
        });
    }

    // (d) Decimation factor at fixed large size — isolates decimation benefit
    let decimations = [1.0_f32, 2.0, 4.0];
    for &decimate in &decimations {
        scenarios.push(Scenario {
            name: format!("scaling-decimate-{decimate:.0}x"),
            description: format!("4000×3000 single tag, decimate={decimate}"),
            category: Category::Scaling,
            expect_ids: vec![("tag36h11".to_string(), 0)],
            max_corner_rmse: if decimate >= 4.0 { 5.0 } else { 3.0 },
            quad_decimate: Some(decimate),
            build_fn: Box::new(|| {
                SceneBuilder::new(4000, 3000)
                    .background(Background::Solid(128))
                    .add_tag(
                        "tag36h11",
                        0,
                        Transform::Similarity {
                            cx: 2000.0,
                            cy: 1500.0,
                            scale: 500.0,
                            theta: 0.0,
                        },
                    )
                    .build()
            }),
        });
    }

    scenarios
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn all_scenarios_non_empty() {
        let scenarios = all_scenarios();
        assert!(!scenarios.is_empty());
    }

    #[test]
    fn each_category_has_scenarios() {
        for cat in Category::all() {
            let scenarios = scenarios_for_category(*cat);
            assert!(!scenarios.is_empty(), "category {:?} has no scenarios", cat);
        }
    }

    #[test]
    fn scenario_builds_produce_valid_scenes() {
        let scenarios = all_scenarios();
        for scenario in scenarios.iter() {
            let scene = scenario.build();
            assert!(scene.image.width > 0);
            assert!(scene.image.height > 0);
            assert!(!scene.ground_truth.is_empty());
        }
    }

    #[test]
    fn category_from_name_roundtrip() {
        for cat in Category::all() {
            assert_eq!(Category::from_name(cat.name()), Some(*cat));
        }
        assert_eq!(Category::from_name("nonexistent"), None);
    }

    #[test]
    fn baseline_scenarios_cover_families() {
        let scenarios = scenarios_for_category(Category::Baseline);
        let families: Vec<_> = scenarios
            .iter()
            .flat_map(|s| s.expect_ids.iter().map(|(f, _)| f.clone()))
            .collect();
        assert!(families.contains(&"tag36h11".to_string()));
        assert!(families.contains(&"tag16h5".to_string()));
        assert!(families.contains(&"tag25h9".to_string()));
    }
}
