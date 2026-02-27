# 002 — Core type definitions

## Goal
Define all value types used across the detection pipeline with `Default` implementations matching the spec.

## Preconditions
- 001 complete: `apriltag-detect` crate compiles

## Postconditions
- `cargo test -p apriltag-detect` passes
- `DetectorConfig::default()` matches spec: `quad_decimate=2.0, quad_sigma=0.0, refine_edges=true, decode_sharpening=0.25`
- `QuadThreshParams::default()` matches spec: `min_cluster_pixels=5, max_nmaxima=10, cos_critical_rad≈0.9848, max_line_fit_mse=10.0, min_white_black_diff=5, deglitch=false`
- All types implement `Debug, Clone`

## Description
Create `apriltag-detect/src/types.rs` with:

- **`DetectorConfig`** — quad_decimate, quad_sigma, refine_edges, decode_sharpening, quad_thresh
- **`QuadThreshParams`** — min_cluster_pixels, max_nmaxima, cos_critical_rad, max_line_fit_mse, min_white_black_diff, deglitch
- **`Quad`** — `corners: [[f32; 2]; 4]`, `reversed_border: bool`
- **`Detection`** — family (String), id (u32), hamming (u32), decision_margin (f32), homography ([f64; 9]), center ([f64; 2]), corners ([[f64; 2]; 4])
- **`Pose`** — rotation ([f64; 9]), translation ([f64; 3]), error (f64)
- **`EdgePoint`** — `#[repr(C)]` packed: x (u16), y (u16), gx (i16), gy (i16), slope (f32) = 12 bytes
- **`LineFitPoint`** — mx, my, mxx, mxy, myy, w (all f64) = 48 bytes

Tests verify all default values match the detection spec §1.3.

## References
- `docs/detection-spec.md` §1.3 (detector config defaults)
- `docs/detection-spec.md` §1.4 (Quad)
- `docs/detection-spec.md` §1.5 (Detection)
- `docs/detection-spec.md` §1.6 (Pose)
- `docs/detection-spec.md` §1.7 (EdgePoint, LineFitPoint)
