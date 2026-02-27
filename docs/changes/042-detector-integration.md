# 042 — Detector struct with workspace and full pipeline

## Goal
Wire all pipeline stages into a single `Detector::detect()` call with internal buffer management via the workspace pattern.

## Preconditions
- All pipeline stages complete (009-041)
- `TagFamily` and `builtin_family()` from `apriltag` crate

## Postconditions
- `Detector::new(config, families)` creates detector with workspace
- `detector.detect(&image)` returns `Vec<Detection>`
- Round-trip test: render tag36h11 id=0 → detect → get id=0 with hamming=0
- Round-trip test: render tag16h5 id=5 → detect → get id=5
- No panics on empty image
- Workspace buffers allocated on first call, reused on subsequent calls

## Description
Create `apriltag-detect/src/detector.rs`:

```rust
pub struct Detector {
    config: DetectorConfig,
    families: Vec<(TagFamily, QuickDecode)>,
    workspace: Workspace,
}

struct Workspace {
    decimated: ImageU8,
    thresholded: ImageU8,
    blur_scratch: ImageU8,
    unionfind: UnionFind,
    edges: Vec<(u64, EdgePoint)>,
    line_fits: Vec<LineFitPoint>,
    errors: Vec<f64>,
    sort_scratch: Vec<EdgePoint>,
    quads: Vec<Quad>,
}

impl Detector {
    pub fn new(config: DetectorConfig) -> Self
    pub fn add_family(&mut self, family: TagFamily, max_hamming: u32)
    pub fn detect(&mut self, image: &ImageU8) -> Vec<Detection>
}
```

`detect()` orchestrates the pipeline:
1. Decimate image → `workspace.decimated`
2. Gaussian blur (if sigma > 0)
3. Adaptive threshold → `workspace.thresholded`
4. Build connected components via union-find
5. Extract edge points → `workspace.edges`
6. Build cluster map
7. For each cluster: size filter → sort → prefix sums → corners → quad validation
8. Edge refinement on original image (if enabled)
9. For each quad × family: homography → gray model → polarity check → bit sampling → sharpening → code extraction → quick-decode
10. Deduplication
11. Return detections

## References
- `docs/detection-spec.md` §2 — full pipeline overview
- `apriltag/src/family.rs:133` — `builtin_family(name)` for loading families
