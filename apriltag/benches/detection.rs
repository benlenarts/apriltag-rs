//! Criterion benchmarks for individual detection pipeline stages and end-to-end detection.

use criterion::{black_box, criterion_group, criterion_main, Criterion};

use apriltag::detect::cluster::gradient_clusters;
use apriltag::detect::connected::connected_components;
use apriltag::detect::decode::{decode_quad, QuickDecode};
use apriltag::detect::detector::{Detector, DetectorConfig, DetectorState};
use apriltag::detect::homography::Homography;
use apriltag::detect::image::ImageU8;
use apriltag::detect::preprocess::{apply_sigma, decimate};
use apriltag::detect::quad::{fit_quads, QuadThreshParams};
use apriltag::detect::refine::refine_edges;
use apriltag::detect::threshold::threshold;
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
        b.iter(|| apply_sigma(black_box(&img), 0.8, Vec::new(), Vec::new()))
    });
}

fn bench_threshold(c: &mut Criterion) {
    let img = build_bench_image();
    let decimated = decimate(&img, 2, Vec::new());
    c.bench_function("threshold", |b| {
        b.iter(|| threshold(black_box(&decimated), 5, false, Vec::new()))
    });
}

fn bench_connected_components(c: &mut Criterion) {
    let img = build_bench_image();
    let decimated = decimate(&img, 2, Vec::new());
    let threshed = threshold(&decimated, 5, false, Vec::new());
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
    let threshed = threshold(&decimated, 5, false, Vec::new());
    c.bench_function("gradient_clusters", |b| {
        b.iter(|| {
            let mut uf = UnionFind::empty();
            connected_components(&threshed, &mut uf);
            gradient_clusters(black_box(&threshed), &mut uf, 5)
        })
    });
}

fn bench_fit_quads(c: &mut Criterion) {
    let img = build_bench_image();
    let decimated = decimate(&img, 2, Vec::new());
    let threshed = threshold(&decimated, 5, false, Vec::new());
    let mut uf = UnionFind::empty();
    connected_components(&threshed, &mut uf);
    let clusters = gradient_clusters(&threshed, &mut uf, 5);
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
    let img = build_bench_image();
    let decimated = decimate(&img, 2, Vec::new());
    let threshed = threshold(&decimated, 5, false, Vec::new());
    let mut uf = UnionFind::empty();
    connected_components(&threshed, &mut uf);
    let mut clusters = gradient_clusters(&threshed, &mut uf, 5);
    let qtp = QuadThreshParams::default();
    let quads = fit_quads(
        &mut clusters,
        decimated.width,
        decimated.height,
        &qtp,
        true,
        true,
    );
    assert!(!quads.is_empty(), "should have quads to refine");

    // Scale corners back to original image coords (as detector does after decimation)
    let mut quads_scaled: Vec<_> = quads
        .into_iter()
        .map(|mut q| {
            for c in &mut q.corners {
                c[0] *= 2.0;
                c[1] *= 2.0;
            }
            q
        })
        .collect();

    c.bench_function("refine_edges", |b| {
        b.iter(|| {
            let mut quads = quads_scaled.clone();
            for q in &mut quads {
                refine_edges(black_box(q), black_box(&img), 2.0);
            }
        })
    });
}

fn bench_decode(c: &mut Criterion) {
    let img = build_bench_image();
    let fam = family::tag36h11();
    let qd = QuickDecode::new(&fam, 2);

    // Run the pipeline to get a real quad + homography
    let decimated = decimate(&img, 2, Vec::new());
    let threshed = threshold(&decimated, 5, false, Vec::new());
    let mut uf = UnionFind::empty();
    connected_components(&threshed, &mut uf);
    let mut clusters = gradient_clusters(&threshed, &mut uf, 5);
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

fn bench_end_to_end(c: &mut Criterion) {
    let img = build_bench_image();
    let config = DetectorConfig {
        quad_sigma: 0.8,
        ..DetectorConfig::default()
    };
    let mut detector = Detector::new(config);
    detector.add_family(family::tag36h11(), 2);

    // Sanity check: the image should produce a detection
    let dets = detector.detect(&img);
    assert!(!dets.is_empty(), "bench image should produce a detection");

    c.bench_function("end_to_end", |b| {
        b.iter(|| detector.detect(black_box(&img)))
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

    let mut state = DetectorState::new();
    // Warm up to populate buffers
    let dets = detector.detect_with_state(&img, &mut state);
    assert!(!dets.is_empty(), "bench image should produce a detection");

    c.bench_function("end_to_end_reuse", |b| {
        b.iter(|| detector.detect_with_state(black_box(&img), &mut state))
    });
}

criterion_group!(
    benches,
    bench_decimate,
    bench_sigma,
    bench_threshold,
    bench_connected_components,
    bench_gradient_clusters,
    bench_fit_quads,
    bench_refine_edges,
    bench_decode,
    bench_end_to_end,
    bench_end_to_end_reuse,
);
criterion_main!(benches);
