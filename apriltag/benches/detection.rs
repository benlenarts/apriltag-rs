//! Criterion benchmarks for individual detection pipeline stages and end-to-end detection.

use criterion::{black_box, criterion_group, criterion_main, Criterion};

use apriltag::detect::cluster::gradient_clusters;
use apriltag::detect::connected::connected_components;
use apriltag::detect::decode::{decode_quad, QuickDecode};
use apriltag::detect::detector::{Detector, DetectorBuffers, DetectorConfig};
use apriltag::detect::homography::Homography;
use apriltag::detect::image::ImageU8;
use apriltag::detect::preprocess::{apply_sigma, decimate};
use apriltag::detect::quad::{fit_quads, QuadThreshParams};
use apriltag::detect::refine::refine_edges;
use apriltag::detect::threshold::{threshold, ThresholdBuffers};
use apriltag::detect::unionfind::UnionFind;
use apriltag::family;
use apriltag::render;
use apriltag::types::Pixel;

/// Build a 640x480 image with a centered tag36h11 tag (scale ~40px per grid cell).
fn build_bench_image() -> ImageU8 {
    let fam = family::tag36h11();
    let rendered = render::render(&fam.layout, fam.codes[0]);

    let (w, h) = (640u32, 480u32);
    let mut img = ImageU8::new(w, h);
    for y in 0..h {
        for x in 0..w {
            img.set(x, y, 255);
        }
    }

    // tag36h11 grid_size = 10, scale 40 => 400px tag, centered in 640x480
    let scale = 40u32;
    let tag_px = rendered.grid_size as u32 * scale;
    let ox = (w - tag_px) / 2;
    let oy = (h - tag_px) / 2;
    for ty in 0..rendered.grid_size {
        for tx in 0..rendered.grid_size {
            let val = match rendered.pixel(tx, ty) {
                Pixel::Black => 0u8,
                Pixel::White | Pixel::Transparent => 255u8,
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

    img
}

fn bench_decimate(c: &mut Criterion) {
    let img = build_bench_image();
    c.bench_function("preprocess/decimate", |b| {
        b.iter(|| decimate(black_box(&img), 2, Vec::new()))
    });
}

fn bench_sigma(c: &mut Criterion) {
    let img = build_bench_image();
    c.bench_function("preprocess/sigma", |b| {
        b.iter(|| apply_sigma(black_box(&img), 0.8, Vec::new(), Vec::new(), Vec::new()))
    });
}

fn bench_threshold(c: &mut Criterion) {
    let img = build_bench_image();
    let decimated = decimate(&img, 2, Vec::new());
    c.bench_function("threshold", |b| {
        let mut tbufs = ThresholdBuffers::new();
        b.iter(|| threshold(black_box(&decimated), 5, false, Vec::new(), &mut tbufs))
    });
}

fn bench_connected_components(c: &mut Criterion) {
    let img = build_bench_image();
    let decimated = decimate(&img, 2, Vec::new());
    let threshed = threshold(
        &decimated,
        5,
        false,
        Vec::new(),
        &mut ThresholdBuffers::new(),
    );
    c.bench_function("connected_components", |b| {
        b.iter(|| {
            let mut uf = UnionFind::empty();
            connected_components(black_box(&threshed), &mut uf);
        })
    });
}

fn bench_gradient_clusters(c: &mut Criterion) {
    let img = build_bench_image();
    let decimated = decimate(&img, 2, Vec::new());
    let threshed = threshold(
        &decimated,
        5,
        false,
        Vec::new(),
        &mut ThresholdBuffers::new(),
    );
    c.bench_function("gradient_clusters", |b| {
        b.iter(|| {
            let mut uf = UnionFind::empty();
            connected_components(&threshed, &mut uf);
            gradient_clusters(
                black_box(&threshed),
                &mut uf,
                5,
                &mut apriltag::detect::cluster::ClusterMap::new(),
            )
        })
    });
}

/// Build a 300x300 noisy image with a centered tag, matching the benchmark
/// harness noise-sigma20 scenario.
fn build_noisy_image() -> ImageU8 {
    let fam = family::tag36h11();
    let rendered = render::render(&fam.layout, fam.codes[0]);

    let (w, h) = (300u32, 300u32);
    let mut img = ImageU8::new(w, h);
    for y in 0..h {
        for x in 0..w {
            img.set(x, y, 255);
        }
    }

    let scale = 20u32;
    let tag_px = rendered.grid_size as u32 * scale;
    let ox = (w - tag_px) / 2;
    let oy = (h - tag_px) / 2;
    for ty in 0..rendered.grid_size {
        for tx in 0..rendered.grid_size {
            let val = match rendered.pixel(tx, ty) {
                Pixel::Black => 0u8,
                Pixel::White | Pixel::Transparent => 255u8,
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

    // Add strong pseudo-random noise (deterministic LCG, sigma ~40)
    // to create many small components and boundary crossings
    let mut rng: u32 = 0xDEAD_BEEF;
    for y in 0..h {
        for x in 0..w {
            let mut sum: i32 = 0;
            for _ in 0..4 {
                rng = rng.wrapping_mul(1103515245).wrapping_add(12345);
                sum += ((rng >> 16) % 256) as i32 - 128;
            }
            let noise = sum / 2; // stronger noise
            let v = img.get(x, y) as i32 + noise;
            img.set(x, y, v.clamp(0, 255) as u8);
        }
    }

    img
}

fn bench_gradient_clusters_noisy(c: &mut Criterion) {
    let img = build_noisy_image();
    let decimated = decimate(&img, 2, Vec::new());
    let threshed = threshold(
        &decimated,
        5,
        false,
        Vec::new(),
        &mut ThresholdBuffers::new(),
    );
    c.bench_function("gradient_clusters_noisy", |b| {
        b.iter(|| {
            let mut uf = UnionFind::empty();
            connected_components(&threshed, &mut uf);
            gradient_clusters(
                black_box(&threshed),
                &mut uf,
                5,
                &mut apriltag::detect::cluster::ClusterMap::new(),
            )
        })
    });
}

fn bench_fit_quads(c: &mut Criterion) {
    let img = build_bench_image();
    let decimated = decimate(&img, 2, Vec::new());
    let threshed = threshold(
        &decimated,
        5,
        false,
        Vec::new(),
        &mut ThresholdBuffers::new(),
    );
    let mut uf = UnionFind::empty();
    connected_components(&threshed, &mut uf);
    let clusters = gradient_clusters(
        &threshed,
        &mut uf,
        5,
        &mut apriltag::detect::cluster::ClusterMap::new(),
    );
    let qtp = QuadThreshParams::default();
    c.bench_function("fit_quads", |b| {
        b.iter(|| {
            let mut clusters = clusters.clone();
            fit_quads(
                &mut clusters,
                decimated.width,
                decimated.height,
                black_box(&qtp),
                true,
                true,
            )
        })
    });
}

fn bench_refine_edges(c: &mut Criterion) {
    use apriltag::detect::quad::Quad;

    let img = build_bench_image();

    // Create many synthetic quads spread across the interior of the image.
    // Each is a 60x60 pixel quad — large enough for meaningful refinement,
    // small enough to tile many across the 640x480 image.
    let mut quads = Vec::new();
    let mut y = 40.0f64;
    while y + 60.0 < 440.0 {
        let mut x = 40.0f64;
        while x + 60.0 < 600.0 {
            quads.push(Quad {
                corners: [[x, y + 60.0], [x + 60.0, y + 60.0], [x + 60.0, y], [x, y]],
                reversed_border: false,
            });
            x += 70.0;
        }
        y += 70.0;
    }
    let n_quads = quads.len();
    assert!(n_quads >= 40, "expected many quads, got {n_quads}");

    let mut group = c.benchmark_group("refine_edges");

    group.bench_function("many_quads", |b| {
        let mut vals = Vec::new();
        b.iter(|| {
            let mut qs = quads.clone();
            for q in &mut qs {
                refine_edges(black_box(q), black_box(&img), 2.0, &mut vals);
            }
        })
    });

    group.bench_function("high_decimate", |b| {
        let mut vals = Vec::new();
        b.iter(|| {
            let mut qs = quads.clone();
            for q in &mut qs {
                refine_edges(black_box(q), black_box(&img), 4.0, &mut vals);
            }
        })
    });

    group.finish();
}

fn bench_decode(c: &mut Criterion) {
    let img = build_bench_image();
    let fam = family::tag36h11();
    let qd = QuickDecode::new(&fam, 2);

    // Run the pipeline to get a real quad + homography
    let decimated = decimate(&img, 2, Vec::new());
    let threshed = threshold(
        &decimated,
        5,
        false,
        Vec::new(),
        &mut ThresholdBuffers::new(),
    );
    let mut uf = UnionFind::empty();
    connected_components(&threshed, &mut uf);
    let mut clusters = gradient_clusters(
        &threshed,
        &mut uf,
        5,
        &mut apriltag::detect::cluster::ClusterMap::new(),
    );
    let qtp = QuadThreshParams::default();
    let quads = fit_quads(
        &mut clusters,
        decimated.width,
        decimated.height,
        &qtp,
        true,
        true,
    );

    // Find a quad that produces a valid homography and successful decode
    let (h, reversed) = quads
        .iter()
        .find_map(|q| {
            // Scale corners back to original coords like the detector does
            let mut corners = q.corners;
            for c in &mut corners {
                c[0] *= 2.0;
                c[1] *= 2.0;
            }
            let h = Homography::from_quad_corners(&corners)?;
            // Verify this quad actually decodes
            decode_quad(&img, &fam, &qd, &h, q.reversed_border, 0.25)?;
            Some((h, q.reversed_border))
        })
        .expect("bench image should produce at least one decodable quad");

    c.bench_function("decode", |b| {
        b.iter(|| decode_quad(black_box(&img), &fam, &qd, black_box(&h), reversed, 0.25))
    });
}

/// Build a 1280x960 image with a grid of tag36h11 tags (scale 10px per cell = 100px tags).
fn build_multi_tag_image() -> ImageU8 {
    let fam = family::tag36h11();
    let tag_px = fam.layout.grid_size as u32 * 10; // 100px per tag
    let spacing = tag_px + 10; // 10px gap between tags
    let (w, h) = (1280u32, 960u32);

    let mut img = ImageU8::new(w, h);
    // Fill white
    for y in 0..h {
        for x in 0..w {
            img.set(x, y, 255);
        }
    }

    let mut code_idx = 0;
    let mut oy = 10u32;
    while oy + tag_px < h {
        let mut ox = 10u32;
        while ox + tag_px < w {
            let rendered = render::render(&fam.layout, fam.codes[code_idx % fam.codes.len()]);
            for ty in 0..rendered.grid_size {
                for tx in 0..rendered.grid_size {
                    let val = match rendered.pixel(tx, ty) {
                        Pixel::Black => 0u8,
                        Pixel::White | Pixel::Transparent => 255u8,
                    };
                    for dy in 0..10u32 {
                        for dx in 0..10u32 {
                            img.set(ox + tx as u32 * 10 + dx, oy + ty as u32 * 10 + dy, val);
                        }
                    }
                }
            }
            code_idx += 1;
            ox += spacing;
        }
        oy += spacing;
    }

    img
}

fn bench_end_to_end_multi(c: &mut Criterion) {
    let img = build_multi_tag_image();
    let config = DetectorConfig {
        quad_sigma: 0.8,
        ..DetectorConfig::default()
    };
    let mut detector = Detector::new(config);
    detector.add_family(family::tag36h11(), 2);

    let mut buffers = DetectorBuffers::new();
    let dets = detector.detect(&img, &mut buffers);
    assert!(
        dets.len() >= 50,
        "multi-tag image should produce many detections, got {}",
        dets.len()
    );

    c.bench_function("end_to_end_multi", |b| {
        b.iter(|| detector.detect(black_box(&img), &mut buffers))
    });
}

fn bench_end_to_end(c: &mut Criterion) {
    let img = build_bench_image();
    let config = DetectorConfig {
        quad_sigma: 0.8,
        ..DetectorConfig::default()
    };
    let mut detector = Detector::new(config);
    detector.add_family(family::tag36h11(), 2);

    let mut buffers = DetectorBuffers::new();
    // Sanity check: the image should produce a detection
    let dets = detector.detect(&img, &mut buffers);
    assert!(!dets.is_empty(), "bench image should produce a detection");

    c.bench_function("end_to_end", |b| {
        b.iter(|| detector.detect(black_box(&img), &mut buffers))
    });
}

fn bench_end_to_end_reuse(c: &mut Criterion) {
    let img = build_bench_image();
    let config = DetectorConfig {
        quad_sigma: 0.8,
        ..DetectorConfig::default()
    };
    let mut detector = Detector::new(config);
    detector.add_family(family::tag36h11(), 2);

    let mut buffers = DetectorBuffers::new();
    // Warm up to populate buffers
    let dets = detector.detect(&img, &mut buffers);
    assert!(!dets.is_empty(), "bench image should produce a detection");

    c.bench_function("end_to_end_reuse", |b| {
        b.iter(|| detector.detect(black_box(&img), &mut buffers))
    });
}

/// Build a 4000x3000 image with ~100 tag36h11 tags, simulating a realistic
/// high-resolution camera scene. Each tag has a unique rotation and mild
/// perspective tilt. Global distortions include noise, a lighting gradient,
/// and slight blur.
fn build_highres_image() -> ImageU8 {
    let fam = family::tag36h11();
    let grid = fam.layout.grid_size as f64; // 10 for tag36h11
    let tag_half = 100.0; // half-size in pixels (200px tags)
    let spacing = 300.0; // center-to-center distance
    let (w, h) = (4000u32, 3000u32);

    let mut img = ImageU8::new(w, h);
    for y in 0..h {
        for x in 0..w {
            img.set(x, y, 128);
        }
    }

    // Deterministic pseudo-random via LCG (used for per-tag variation)
    let mut rng: u32 = 0xBAAD_F00D;
    let next_f32 = |rng: &mut u32| -> f32 {
        *rng = rng.wrapping_mul(1103515245).wrapping_add(12345);
        ((*rng >> 16) as f32) / 65535.0 // 0.0 .. 1.0
    };

    let mut code_idx = 0usize;
    let mut cy = 180.0f64;
    while cy + tag_half < h as f64 - 10.0 {
        let mut cx = 180.0f64;
        while cx + tag_half < w as f64 - 10.0 {
            let rendered = render::render(&fam.layout, fam.codes[code_idx % fam.codes.len()]);

            // Per-tag rotation: -30° to +30°
            let angle = (next_f32(&mut rng) - 0.5) * 60.0_f32.to_radians();
            let cos_a = angle.cos() as f64;
            let sin_a = angle.sin() as f64;

            // Per-tag perspective tilt: foreshorten top or bottom by 0–20%
            let tilt = (next_f32(&mut rng) - 0.5) * 0.4; // -0.2 .. +0.2
            let top_scale = 1.0 - tilt as f64; // shrink/grow top edge
            let bot_scale = 1.0 + tilt as f64; // opposite for bottom

            // Compute the 4 destination corners (in image coords) of the tag quad.
            // Tag corners in tag-local coords: (-1,-1), (1,-1), (1,1), (-1,1)
            let corners: [(f64, f64); 4] = [
                (-top_scale, -1.0),
                (top_scale, -1.0),
                (bot_scale, 1.0),
                (-bot_scale, 1.0),
            ];
            let dst: [(f64, f64); 4] = corners.map(|(lx, ly)| {
                let rx = lx * cos_a - ly * sin_a;
                let ry = lx * sin_a + ly * cos_a;
                (cx + rx * tag_half, cy + ry * tag_half)
            });

            // Bounding box of destination quad
            let min_x = dst.iter().map(|c| c.0).fold(f64::MAX, f64::min);
            let max_x = dst.iter().map(|c| c.0).fold(f64::MIN, f64::max);
            let min_y = dst.iter().map(|c| c.1).fold(f64::MAX, f64::min);
            let max_y = dst.iter().map(|c| c.1).fold(f64::MIN, f64::max);

            let x0 = (min_x as i32).max(0) as u32;
            let x1 = ((max_x as i32) + 1).min(w as i32) as u32;
            let y0 = (min_y as i32).max(0) as u32;
            let y1 = ((max_y as i32) + 1).min(h as i32) as u32;

            // Inverse bilinear: for each pixel in the bounding box, map back
            // to tag-local coords and sample the rendered tag.
            // We use the inverse of the affine part (rotation + perspective corners).
            // For simplicity, use the inverse of the affine transform (ignoring
            // the mild perspective — it's close enough for benchmarking).
            let inv_det = cos_a * cos_a + sin_a * sin_a; // = 1.0
            for py in y0..y1 {
                for px in x0..x1 {
                    // Map pixel back to tag-local [-1, 1] space
                    let dx = px as f64 - cx;
                    let dy = py as f64 - cy;
                    let lx = (dx * cos_a + dy * sin_a) / (tag_half * inv_det);
                    let ly = (-dx * sin_a + dy * cos_a) / (tag_half * inv_det);

                    // Apply inverse perspective (approximate: scale x by row)
                    let row_scale = if ly <= 0.0 {
                        top_scale as f64 + (1.0 + ly) * (1.0 - top_scale as f64)
                    } else {
                        1.0 + ly * (bot_scale as f64 - 1.0)
                    };
                    let lx_corr = lx / row_scale;

                    if lx_corr < -1.0 || lx_corr > 1.0 || ly < -1.0 || ly > 1.0 {
                        continue;
                    }

                    // Map [-1, 1] to grid coords [0, grid)
                    let gx = ((lx_corr + 1.0) / 2.0 * grid).floor() as i32;
                    let gy = ((ly + 1.0) / 2.0 * grid).floor() as i32;
                    if gx < 0 || gx >= grid as i32 || gy < 0 || gy >= grid as i32 {
                        continue;
                    }

                    let val = match rendered.pixel(gx as usize, gy as usize) {
                        Pixel::Black => 30u8,
                        Pixel::White | Pixel::Transparent => 225u8,
                    };
                    img.set(px, py, val);
                }
            }

            code_idx += 1;
            cx += spacing;
        }
        cy += spacing;
    }

    // Lighting gradient: left side +40, right side -40
    for y in 0..h {
        for x in 0..w {
            let gradient = 40.0 - 80.0 * (x as f32 / w as f32);
            let v = img.get(x, y) as f32 + gradient;
            img.set(x, y, v.clamp(0.0, 255.0) as u8);
        }
    }

    // Gaussian noise (sigma ~15, deterministic LCG)
    let mut noise_rng: u32 = 0xCAFE_BABE;
    for y in 0..h {
        for x in 0..w {
            let mut sum: i32 = 0;
            for _ in 0..4 {
                noise_rng = noise_rng.wrapping_mul(1103515245).wrapping_add(12345);
                sum += ((noise_rng >> 16) % 256) as i32 - 128;
            }
            let noise = sum * 15 / (4 * 37); // ~sigma 15
            let v = img.get(x, y) as i32 + noise;
            img.set(x, y, v.clamp(0, 255) as u8);
        }
    }

    // Slight blur: 2x2 box average
    let mut blurred = ImageU8::new(w, h);
    for y in 0..h - 1 {
        for x in 0..w - 1 {
            let avg = (img.get(x, y) as u32
                + img.get(x + 1, y) as u32
                + img.get(x, y + 1) as u32
                + img.get(x + 1, y + 1) as u32)
                / 4;
            blurred.set(x, y, avg as u8);
        }
    }

    blurred
}

fn bench_end_to_end_highres(c: &mut Criterion) {
    let img = build_highres_image();
    let config = DetectorConfig {
        quad_sigma: 0.8,
        ..DetectorConfig::default()
    };
    let mut detector = Detector::new(config);
    detector.add_family(family::tag36h11(), 2);

    let mut buffers = DetectorBuffers::new();
    let dets = detector.detect(&img, &mut buffers);
    eprintln!(
        "highres 4000x3000: detected {} tags (image {}x{})",
        dets.len(),
        img.width,
        img.height
    );
    assert!(
        dets.len() >= 50,
        "highres image should detect many tags, got {}",
        dets.len()
    );

    c.bench_function("end_to_end_highres_4000x3000", |b| {
        b.iter(|| detector.detect(black_box(&img), &mut buffers))
    });
}

criterion_group!(
    benches,
    bench_decimate,
    bench_sigma,
    bench_threshold,
    bench_connected_components,
    bench_gradient_clusters,
    bench_gradient_clusters_noisy,
    bench_fit_quads,
    bench_refine_edges,
    bench_decode,
    bench_end_to_end,
    bench_end_to_end_multi,
    bench_end_to_end_reuse,
    bench_end_to_end_highres,
);
criterion_main!(benches);
