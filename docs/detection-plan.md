# AprilTag Detection Pipeline — Implementation Plan

## Context

The `apriltag2` project has core types, families, and rendering fully implemented (42 tests, 100% coverage). The entire detection pipeline — the main purpose of the library — remains unbuilt. The goal is to implement a **high-performance, WASM-compatible** detection pipeline that matches the reference C implementation's quality, exposed via both a TypeScript-friendly WASM API and a full-featured CLI.

---

## Architecture Decisions

### Crate Layout
```
apriltag/              — core types, families, rendering (EXISTING, unchanged)
apriltag-detect/       — NEW: detection pipeline library (zero external deps besides apriltag)
apriltag-wasm/         — NEW: wasm-bindgen + tsify bindings
apriltag-detect-cli/   — NEW: CLI (PNG/JPG → JSON)
```

### Key Dependencies
| Crate | Where | Why |
|-------|-------|-----|
| `rayon 1.10` | apriltag-detect (optional `parallel` feature) | Parallelism for clustering + quad fitting |
| `image 0.25` | apriltag-detect-cli only | PNG/JPEG loading |
| `serde_json 1` | apriltag-detect-cli only | JSON output |
| `clap 4.5` | apriltag-detect-cli only | CLI argument parsing |
| `wasm-bindgen 0.2` | apriltag-wasm only | Rust↔JS bridge |
| `tsify-next 0.5` | apriltag-wasm only | Auto-generated TypeScript types |
| `serde-wasm-bindgen 0.6` | apriltag-wasm only | Efficient serde↔JsValue |
| No `nalgebra` | — | Hand-roll all math (3x3, 8x9) for small WASM binary + full control |
| No `bumpalo` | — | Workspace buffer reuse is simpler; add later if profiling warrants |

### Allocation Strategy
- **Workspace pattern**: `Detector` holds pre-allocated buffers reused across `detect()` calls
- **Zero steady-state allocation**: After first frame, all buffers warm; subsequent calls do `clear()` + possible `resize()`
- **Custom flat hash map** for gradient clusters (open addressing, Knuth multiplicative hash) — no per-cluster heap allocation
- **Stack allocation** for all small fixed-size data: `[f64; 9]` homography, `[f64; 4]` line params, etc.

### Existing Code to Reuse
| What | Location | Used By |
|------|----------|---------|
| `TagFamily` (codes, bit_locations, layout) | `apriltag/src/family.rs` | Decoding (stage 8) |
| `hamming::rotate90(code, nbits)` | `apriltag/src/hamming.rs:6` | Quick-decode rotation |
| `hamming::hamming_distance(a, b)` | `apriltag/src/hamming.rs:21` | Quick-decode matching |
| `BitLocation { x, y }` | `apriltag/src/bits.rs` | Bit sampling coordinates |
| `Layout` (width_at_border, total_width, reversed_border) | `apriltag/src/layout/mod.rs` | Gray model border sampling |
| `builtin_family(name)` / `BUILTIN_NAMES` | `apriltag/src/family.rs:121-145` | CLI + WASM construction |
| `render()` | `apriltag/src/render.rs` | Test image generation |

---

## Atomic Changes

Each change below will get its own `docs/changes/NNN-short-name.md` with goal, pre/postconditions, description, and references. The plan file here serves as the index.

---

### Phase 1: Crate Scaffold

#### 001 — Create `apriltag-detect` crate skeleton
- **Goal**: Establish the crate with Cargo.toml, empty lib.rs, and workspace membership
- **Preconditions**: Workspace has 3 crates (apriltag, apriltag-gen, apriltag-gen-cli)
- **Postconditions**: `cargo check -p apriltag-detect` passes; workspace has 4 members
- **Description**: Create `apriltag-detect/Cargo.toml` with `apriltag` path dep, `std`/`parallel` features, rayon optional dep. Create `apriltag-detect/src/lib.rs` (empty). Add to workspace `Cargo.toml` members.
- **References**: `Cargo.toml:1-3`, `apriltag/Cargo.toml` (for pattern)

#### 002 — Core type definitions
- **Goal**: Define all value types used across the detection pipeline
- **Preconditions**: 001 complete
- **Postconditions**: `cargo test -p apriltag-detect` passes; types compile with `Default` impls; `DetectorConfig::default()` matches spec defaults
- **Description**: Create `apriltag-detect/src/types.rs` with: `DetectorConfig`, `QuadThreshParams` (with `Default`), `Quad` (4 corners + reversed_border), `Detection` (family, id, hamming, decision_margin, center, corners, homography), `Pose` (rotation, translation, error). Tests verify default values match `docs/detection-spec.md` §1.3.
- **References**: `docs/detection-spec.md` §1.3-1.6

---

### Phase 2: Math Utilities

#### 003 — 3×3 matrix type and basic operations
- **Goal**: `Mat33` with multiply, determinant, transpose, inverse, identity
- **Preconditions**: 001 complete
- **Postconditions**: `mat33_inverse(mat33_identity()) == mat33_identity()`; inverse of known matrix verified; det of singular matrix == 0
- **Description**: Create `apriltag-detect/src/math.rs`. Define `[f64; 9]` row-major representation. Implement `mul`, `det`, `inverse` (adjugate/det), `transpose`, `identity`. All operations are inline for the compiler.
- **References**: `docs/detection-spec.md` §7 (homography projection formula)

#### 004 — 2×2 symmetric eigenvalue decomposition
- **Goal**: Analytic eigenvalue/eigenvector solver for 2×2 covariance matrices
- **Preconditions**: 003 complete
- **Postconditions**: Known covariance matrix `[[2,1],[1,2]]` → eigenvalues `(3, 1)`; degenerate case (identity) → equal eigenvalues
- **Description**: Add `eigenvalues_sym2x2(cxx, cxy, cyy) -> (f64, f64)` and eigenvector extraction to `math.rs`. Uses the analytic formula: `λ = 0.5(tr ± sqrt(tr² - 4·det))`. Critical for line fitting in quad detection.
- **References**: `docs/detection-spec.md` §5 (line fitting eigenanalysis)

#### 005 — 8×9 Gaussian elimination with partial pivoting
- **Goal**: Solve the DLT system for homography computation
- **Preconditions**: 003 complete
- **Postconditions**: Known 4-point correspondence → recovers exact homography; singular system detected (returns None); pivot < 1e-10 rejected
- **Description**: Add `gauss_eliminate_8x9(a: &mut [[f64; 9]; 8]) -> Option<[f64; 9]>` to `math.rs`. Forward elimination with partial pivoting, back-substitution, h[8] = 1.
- **References**: `docs/detection-spec.md` §7 (DLT, Gaussian elimination)

#### 006 — 3×3 SVD
- **Goal**: Singular value decomposition for pose estimation's SO(3) projection
- **Preconditions**: 003 complete
- **Postconditions**: `U * diag(S) * V' = M` for known matrices; U, V orthogonal; handles rank-deficient input
- **Description**: Add `svd_3x3(m: &[f64; 9]) -> (U, S, V)` to `math.rs`. Implement Golub-Kahan bidiagonalization for 3×3 (well-known, ~100 lines). Port logic from reference C `svd22.h` / `homography.c`.
- **References**: `docs/detection-spec.md` §10 (SVD for rotation projection)

---

### Phase 3: Image Buffer

#### 007 — ImageU8 type with stride-aligned storage
- **Goal**: Core image buffer with cache-line-aligned stride
- **Preconditions**: 001 complete
- **Postconditions**: `ImageU8::new(100, 100).stride >= 100`; stride is multiple of 64; `get(x,y)` and `set(x,y,v)` round-trip correctly; out-of-bounds panics in debug
- **Description**: Create `apriltag-detect/src/image.rs`. `ImageU8 { width, height, stride, buf: Vec<u8> }`. Stride = `(width + 63) & !63`. Provide `new()`, `zeros()`, `get(x,y)`, `set(x,y,v)`, `row(y)`, `resize(w,h)` (reuses capacity). Also `ImageRef<'a>` for borrowed data.
- **References**: `docs/detection-spec.md` §1.1 (image_u8 definition)

#### 008 — Bilinear interpolation
- **Goal**: Sub-pixel sampling for edge refinement and bit sampling
- **Preconditions**: 007 complete
- **Postconditions**: Integer coordinates return exact pixel values; midpoint returns average of 4 neighbors; out-of-bounds clamped
- **Description**: Add `ImageU8::interpolate(fx: f64, fy: f64) -> f64`. Uses floor for integer part, fractional for blending. 4 pixel reads, 3 multiplies, 3 adds. Mark `#[inline]`.
- **References**: `docs/detection-spec.md` §6 (bilinear interpolation in edge refinement), §8 (bit sampling)

#### 009 — Image decimation (block averaging)
- **Goal**: Downsample image by factor f, averaging f×f blocks
- **Preconditions**: 007 complete
- **Postconditions**: 100×100 image decimated by 2 → 50×50; pixel values are block averages; factor 1 returns copy; non-integer factor handled (floor)
- **Description**: Add `pub fn decimate(src: &ImageU8, factor: f32) -> ImageU8`. Output dimensions: `floor(width/f) × floor(height/f)`. Each output pixel = mean of f×f input block. Coordinates are scaled back by f before decoding (caller's responsibility). Handles factor > 1 only.
- **References**: `docs/detection-spec.md` §3.1 (decimation)

#### 010 — Separable Gaussian blur
- **Goal**: 1D-separable Gaussian convolution with configurable sigma
- **Preconditions**: 007 complete
- **Postconditions**: Blur of impulse ≈ Gaussian profile; blur of uniform image = unchanged; sigma=0 returns identity; negative sigma = unsharp mask
- **Description**: Add `pub fn gaussian_blur(img: &mut ImageU8, sigma: f32, scratch: &mut Vec<u8>)`. Kernel size = `4 * |sigma|` rounded to odd. Separable: horizontal pass into scratch, vertical pass back. When sigma < 0: unsharp mask = `clamp(2*orig - blurred, 0, 255)`.
- **References**: `docs/detection-spec.md` §3.2 (Gaussian filtering), §3.3 (unsharp mask)

---

### Phase 4: Adaptive Thresholding

#### 011 — Tile-based min/max computation
- **Goal**: Compute per-tile minimum and maximum pixel values
- **Preconditions**: 007 complete
- **Postconditions**: Known 8×8 image with tile_size=4 → correct tile min/max arrays; handles image dimensions not divisible by tile_size
- **Description**: Create `apriltag-detect/src/threshold.rs`. Add function to divide image into `tilesz × tilesz` tiles and compute min/max per tile. Default tilesz=4. Output: two arrays of tile values.
- **References**: `docs/detection-spec.md` §4.1 (tile min/max)

#### 012 — Tile min/max dilation and erosion
- **Goal**: 3×3 neighborhood dilation of max tiles and erosion of min tiles
- **Preconditions**: 011 complete
- **Postconditions**: Single bright tile surrounded by dark → max spreads to 3×3 neighborhood; boundary tiles handled correctly (no out-of-bounds)
- **Description**: Add dilate-max and erode-min passes over tile arrays using 3×3 neighborhood. Prevents discontinuities at tile boundaries.
- **References**: `docs/detection-spec.md` §4.2 (dilation/erosion of tile extrema)

#### 013 — Ternary binarization
- **Goal**: Classify each pixel as black (0), white (255), or unknown (127)
- **Preconditions**: 012 complete
- **Postconditions**: Low-contrast region (max-min < min_white_black_diff) → 127; clear regions → 0 or 255; threshold = (min+max)/2
- **Description**: For each pixel, look up interpolated tile min/max. If contrast < `min_white_black_diff`: output 127. Otherwise: output = pixel > threshold ? 255 : 0.
- **References**: `docs/detection-spec.md` §4.3 (binarization)

#### 014 — Deglitch (morphological close)
- **Goal**: Optional morphological close to remove single-pixel noise
- **Preconditions**: 013 complete
- **Postconditions**: Single-pixel hole in white region → filled; single-pixel white in black region → removed; disabled by default
- **Description**: When `deglitch=true`, apply dilate→erode with 3×3 structuring element on the ternary image. Only affects 0/255 pixels; 127 unchanged.
- **References**: `docs/detection-spec.md` §4.4 (deglitch)

---

### Phase 5: Union-Find

#### 015 — UnionFind data structure
- **Goal**: Flat-array union-find with path halving and weighted union
- **Preconditions**: 001 complete
- **Postconditions**: `find(x) == find(y)` after `union(x,y)`; path halving reduces depth; weighted union keeps trees balanced; `get_size()` returns correct component sizes
- **Description**: Create `apriltag-detect/src/unionfind.rs`. Two `Vec<u32>` (parent, size). Parent initialized to 0xFFFFFFFF. `find()` uses path halving: `parent[id] = parent[parent[id]]`. `union()` by size: smaller tree under larger. `reset(n)` reuses allocation.
- **References**: `docs/detection-spec.md` §5 (union-find)

#### 016 — Connected components on threshold image
- **Goal**: Build connected components from the ternary threshold image using union-find
- **Preconditions**: 013, 015 complete
- **Postconditions**: Uniform white region → single component; black and white separated; unknown (127) pixels not connected to anything; diagonal connectivity for white pixels only
- **Description**: Create `apriltag-detect/src/cluster.rs` (initial version). For each pixel: check left (x-1,y), up (x,y-1), and for white: upper-left (x-1,y-1), upper-right (x+1,y-1). Skip diagonals if redundant path exists (optimization from spec). Skip if left & up & upper-left all same component.
- **References**: `docs/detection-spec.md` §5.1 (connectivity rules), §5.2 (union-find operations)

---

### Phase 6: Gradient Clustering

#### 017 — Edge point extraction
- **Goal**: Extract boundary points between adjacent black/white components
- **Preconditions**: 016 complete
- **Postconditions**: Square black region in white background → edge points along boundary; gradient directions point outward; fixed-point coordinates are 2× actual; components with <25 pixels skipped
- **Description**: In `cluster.rs`, for each non-unknown pixel with component size ≥ 25: check 4 neighbors at offsets (1,0), (0,1), (-1,1), (1,1). If opposite polarity (v0+v1=255): create EdgePoint at midpoint with gradient. Cluster ID = `(min(rep0,rep1) << 32) | max(rep0,rep1)`.
- **References**: `docs/detection-spec.md` §6.1 (edge point extraction)

#### 018 — Custom flat hash map for clusters
- **Goal**: Group edge points by cluster ID using an open-addressed hash map
- **Preconditions**: 017 complete
- **Postconditions**: All edge points for same cluster ID retrievable; iteration order is deterministic (sorted by hash then ID); no per-cluster heap allocation
- **Description**: Implement `ClusterMap` with flat `Vec<EdgePoint>` storage and open-addressed index. Hash function: `(2654435761 * id) >> 32` (Knuth multiplicative, matches reference). Pre-size capacity to ~4× expected cluster count.
- **References**: `docs/detection-spec.md` §6.2 (hash clustering), §6.3 (deterministic merge)

---

### Phase 7: Quad Fitting — Angular Sorting

#### 019 — Fast slope proxy computation
- **Goal**: Compute angular sort key without atan2
- **Preconditions**: 002 (EdgePoint type) complete
- **Postconditions**: Points around a square produce monotonically increasing slope values in CCW order; noise offsets prevent zero-division
- **Description**: In `apriltag-detect/src/quad.rs`, implement slope computation: `cx = (xmin+xmax)/2 + 0.05118, cy = (ymin+ymax)/2 - 0.028581`. For each point: `dx = x - cx, dy = y - cy`. Map to quadrant [0,4) using `quadrant + |dy|/|dx|`.
- **References**: `docs/detection-spec.md` §7.3 (angular sorting)

#### 020 — Merge sort with small-size sorting networks
- **Goal**: Sort edge points by slope with optimal small-case handling
- **Preconditions**: 019 complete
- **Postconditions**: Sorted output matches `slice.sort_by` for random inputs; ≤5 elements use inline comparison networks; deterministic (stable sort)
- **Description**: Implement merge sort that dispatches to inline sorting networks for ≤5 elements. This matches the reference C implementation's approach for optimal performance on typical cluster sizes (24-200 points).
- **References**: `docs/detection-spec.md` §7.3 (merge sort with sorting networks)

---

### Phase 8: Quad Fitting — Line Fitting

#### 021 — Cumulative line-fit moments (prefix sums)
- **Goal**: Build prefix sum array for O(1) range line fitting
- **Preconditions**: 019, 020 (sorted points), 002 (LineFitPoint type) complete
- **Postconditions**: `lfps[i].W` equals sum of weights for points 0..=i; range query via subtraction matches brute-force computation; weights = `sqrt(gx² + gy²) + 1`
- **Description**: In `quad.rs`, compute cumulative `LineFitPoint` array from sorted edge points. Each point's weight = gradient magnitude + 1. Accumulate Mx, My, Mxx, Mxy, Myy, W as running sums.
- **References**: `docs/detection-spec.md` §7.4 (cumulative moments)

#### 022 — Line fitting via eigenanalysis
- **Goal**: Fit a line to a range of points using the smallest eigenvalue of the covariance matrix
- **Preconditions**: 021 (prefix sums), 004 (eigenvalues) complete
- **Postconditions**: Points along y=2x → line direction matches; MSE (smallest eigenvalue) near 0 for collinear points; returns line as `[px, py, nx, ny]`
- **Description**: Given prefix sum range [i0, i1]: compute covariance `Cxx, Cxy, Cyy` via subtraction, find eigenvalues, extract eigenvector of largest eigenvalue as line direction. MSE = smallest eigenvalue. Return `[centroid_x, centroid_y, normal_x, normal_y]`.
- **References**: `docs/detection-spec.md` §7.5 (line fitting from moments)

---

### Phase 9: Quad Fitting — Corner Detection

#### 023 — Per-point line-fit error computation
- **Goal**: Compute line-fit error at every point index in the cluster
- **Preconditions**: 021, 022 complete
- **Postconditions**: Error is low in straight segments, high at corners; `ksz = min(20, sz/12)` neighborhood used; wraps around (circular indexing)
- **Description**: For each point i in sorted cluster, fit line to 2·ksz points centered on i (with wraparound). Store err[i] = smallest eigenvalue (MSE).
- **References**: `docs/detection-spec.md` §7.6 (corner detection — error computation)

#### 024 — Gaussian smoothing of error array
- **Goal**: Low-pass filter the error array to suppress noise
- **Preconditions**: 023 complete
- **Postconditions**: Smoothed errors are non-negative; sharp spikes are attenuated; sigma=1, cutoff=0.05
- **Description**: Apply 1D Gaussian filter (σ=1) to the circular error array. Kernel truncated where weight < 0.05.
- **References**: `docs/detection-spec.md` §7.6 (Gaussian smoothing of errors)

#### 025 — Local maxima detection
- **Goal**: Find candidate corner positions as local maxima in the smoothed error array
- **Preconditions**: 024 complete
- **Postconditions**: Square cluster → exactly 4 maxima; `err[i] > err[i-1] && err[i] > err[i+1]`; if > max_nmaxima, keep top N by magnitude
- **Description**: Scan smoothed error array for local maxima. If count > `max_nmaxima` (default 10), keep only the top `max_nmaxima` by error value.
- **References**: `docs/detection-spec.md` §7.6 (local maxima selection)

#### 026 — Exhaustive 4-combination search
- **Goal**: Find best 4 corner indices from candidate maxima
- **Preconditions**: 022 (line fitting), 025 (maxima) complete
- **Postconditions**: Best quad has minimum total MSE; all 4 segment MSEs ≤ max_line_fit_mse; adjacent line angles pass cos_critical_rad check
- **Description**: Try all C(n,4) combinations of maxima indices (m0<m1<m2<m3). For each: fit 4 line segments between consecutive corners, check MSE and angle constraints. Keep combination with minimum total error.
- **References**: `docs/detection-spec.md` §7.7 (exhaustive combination search)

---

### Phase 10: Quad Fitting — Finalization

#### 027 — Line intersection for corner computation
- **Goal**: Compute quad corners by intersecting adjacent fitted lines
- **Preconditions**: 026 complete
- **Postconditions**: 4 corners computed from 4 lines; near-parallel lines (|det| < 0.001) cause rejection; corners match analytical intersection
- **Description**: For adjacent lines i and (i+1)%4, solve 2×2 system for intersection. `det = A00*A11 - A10*A01`. Reject if `|det| < 0.001`.
- **References**: `docs/detection-spec.md` §7.8 (corner intersection)

#### 028 — Quad validation (area, angles, convexity)
- **Goal**: Reject invalid quads before proceeding to decoding
- **Preconditions**: 027 complete
- **Postconditions**: Area < 0.95·tag_width² → rejected; non-convex → rejected; interior angles outside cos_critical_rad → rejected; valid square passes
- **Description**: Check: (1) area via cross product ≥ threshold, (2) all interior angles within range, (3) CCW winding order (convexity). `tag_width = min_family_width / decimate`, minimum 3.
- **References**: `docs/detection-spec.md` §7.9 (quad validation)

#### 029 — Border orientation check
- **Goal**: Determine if quad has normal or reversed border, reject if no family matches
- **Preconditions**: 017 (edge points with gradients) complete
- **Postconditions**: White-outside-black-inside → `reversed_border = false`; reversed → `true`; if no registered family has matching orientation → rejected
- **Description**: Compute `dot = Σ(dx·gx + dy·gy)` for all points relative to centroid. `reversed_border = (dot < 0)`.
- **References**: `docs/detection-spec.md` §7.2 (border orientation)

#### 030 — Size filtering for clusters
- **Goal**: Early-reject clusters that are too small or too large
- **Preconditions**: 018 (cluster map) complete
- **Postconditions**: < min_cluster_pixels → rejected; < 24 → rejected; > 2(2w+2h) → rejected
- **Description**: Before quad fitting, filter clusters by point count. This is a cheap early exit.
- **References**: `docs/detection-spec.md` §7.1 (size filtering)

---

### Phase 11: Edge Refinement

#### 031 — Edge refinement on original image
- **Goal**: Snap quad edges to strong gradients in the original (undecimated) image for sub-pixel accuracy
- **Preconditions**: 007 (ImageU8), 008 (bilinear interpolation), 027 (quad corners) complete
- **Postconditions**: Corners shift toward true edges; sample count = max(16, edge_length/8); search range = quad_decimate + 1
- **Description**: Create `apriltag-detect/src/refine.rs`. For each of 4 edges: compute normal, sample along edge, search along normal for strongest gradient (weighted by (g2-g1)²), accumulate into line fit, recompute corners.
- **References**: `docs/detection-spec.md` §8 (edge refinement)

---

### Phase 12: Homography

#### 032 — DLT homography from 4 correspondences
- **Goal**: Compute 3×3 homography mapping tag coords [-1,1]² to pixel coords
- **Preconditions**: 005 (Gaussian elimination), 003 (Mat33) complete
- **Postconditions**: Known square → identity-like H; 4 corners map back to tag coords within epsilon; `project()` and `unproject()` are consistent inverses
- **Description**: Create `apriltag-detect/src/homography.rs`. Build 8×9 DLT matrix from 4 corner correspondences (tag: (-1,-1), (1,-1), (1,1), (-1,1) → pixel: quad corners). Solve via Gaussian elimination. Provide `project(x, y) -> (px, py)`.
- **References**: `docs/detection-spec.md` §9 (homography, DLT)

---

### Phase 13: Tag Decoding — Quick Decode Table

#### 033 — QuickDecode table construction
- **Goal**: Build chunk-based lookup table from family codes for fast matching
- **Preconditions**: 001 (crate), existing `TagFamily` in apriltag crate
- **Postconditions**: Table built for tag16h5 (16 bits, 30 codes): chunk_size=4, capacity=16; all codes reachable via lookup; chunk_offsets are valid prefix sums
- **Description**: Create `apriltag-detect/src/decode.rs` (initial). `QuickDecode::build(family, max_hamming)`. Split nbits into 4 chunks. For each chunk: count codes per chunk-value, compute prefix sums, fill code indices.
- **References**: `docs/detection-spec.md` §10.5 (quick decode table)

#### 034 — QuickDecode lookup with rotation
- **Goal**: Match a decoded code word against the family, trying all 4 rotations
- **Preconditions**: 033 complete, `hamming::rotate90` and `hamming::hamming_distance` from apriltag crate
- **Postconditions**: Exact match → hamming=0; 1-bit error → hamming=1; rotated code → correct rotation detected; no match → hamming=255
- **Description**: `QuickDecode::decode(rcode) -> Option<DecodeResult>`. For each rotation (0-3): for each chunk: lookup code candidates, check full Hamming distance ≤ max_hamming. Return best match. Uses `apriltag::hamming::rotate90` and `hamming_distance`.
- **References**: `docs/detection-spec.md` §10.5 (quick decode lookup), `apriltag/src/hamming.rs:6,21`

---

### Phase 14: Tag Decoding — Gray Models

#### 035 — Gray model construction
- **Goal**: Build linear intensity models for white and black border regions
- **Preconditions**: 032 (homography), 008 (bilinear interp) complete
- **Postconditions**: Uniform illumination → flat model (C[0]≈0, C[1]≈0, C[2]≈pixel_value); gradient illumination → correct slope
- **Description**: In `decode.rs`, implement `GrayModel` with `add(x, y, intensity)` and `solve()`. Sample 8 line patterns along tag borders (4 white, 4 black). Convert border coords to tag coords, project to pixels, sample. Build separate white and black models.
- **References**: `docs/detection-spec.md` §10.1 (gray model), §10.2 (border sampling patterns)

#### 036 — Polarity check
- **Goal**: Verify that white model > black model at tag center, accounting for reversed borders
- **Preconditions**: 035 complete
- **Postconditions**: Normal tag → white(0,0) > black(0,0); reversed tag → white(0,0) < black(0,0); wrong polarity → quad rejected
- **Description**: Evaluate both models at (0,0). Check polarity matches `reversed_border` flag from quad. If mismatched and no family has the other polarity → skip.
- **References**: `docs/detection-spec.md` §10.3 (polarity check)

---

### Phase 15: Tag Decoding — Bit Sampling

#### 037 — Bit value sampling via homography
- **Goal**: Sample each data bit's value through the homography
- **Preconditions**: 032 (homography), 008 (bilinear interp), 035 (gray models) complete; `BitLocation` from apriltag crate
- **Postconditions**: Rendered tag → correct bit values extracted; threshold computed from gray models; values stored as float (distance from threshold)
- **Description**: For each bit i: convert `bit_locations[i]` to tag coords `[-1,1]`, project to pixel coords, bilinear interpolate, subtract threshold `(black_model + white_model) / 2`. Store as `values[bit_y][bit_x]`.
- **References**: `docs/detection-spec.md` §10.4 (bit sampling), `apriltag/src/bits.rs` (BitLocation)

#### 038 — Decode sharpening (Laplacian)
- **Goal**: Apply Laplacian sharpening kernel to improve bit margin
- **Preconditions**: 037 complete
- **Postconditions**: Sharpening with factor 0.25 increases contrast between adjacent bits; factor 0 → no change
- **Description**: Apply Laplacian kernel `[0,-1,0; -1,4,-1; 0,-1,0]` to the values grid. `values[y][x] += sharpening * laplacian[y][x]`. Default sharpening = 0.25.
- **References**: `docs/detection-spec.md` §10.4 (decode sharpening)

#### 039 — Code extraction and decision margin
- **Goal**: Extract the binary code word and compute confidence metric
- **Preconditions**: 038 complete
- **Postconditions**: Known bit pattern → correct rcode; decision_margin = min(white_score/white_count, black_score/black_count); margin < 0 → rejected
- **Description**: Iterate bits MSB-first. For each: `rcode = (rcode << 1) | (v > 0)`. Track white_score/count and black_score/count with Laplace smoothing (+1). Compute decision margin.
- **References**: `docs/detection-spec.md` §10.4 (code extraction, decision margin)

---

### Phase 16: Deduplication

#### 040 — Separating axis theorem for quad overlap
- **Goal**: Test whether two 4-point polygons overlap
- **Preconditions**: 002 (Quad type) complete
- **Postconditions**: Overlapping quads → true; disjoint quads → false; touching at edge → false (not overlapping)
- **Description**: Create `apriltag-detect/src/dedup.rs`. Test all 8 edge normals (4 per quad) as separating axes. If any axis separates the projections → no overlap.
- **References**: `docs/detection-spec.md` §11 (deduplication, SAT)

#### 041 — Detection deduplication with preference ordering
- **Goal**: Remove duplicate detections of the same tag, keeping the best
- **Preconditions**: 040 complete
- **Postconditions**: Two overlapping detections of same family+ID → one survives; preference: lower hamming > higher margin > lexicographic corners
- **Description**: For each pair of detections with same family+ID: check overlap via SAT. If overlapping, keep the preferred one (lower Hamming, then higher margin, then lexicographic tiebreaker).
- **References**: `docs/detection-spec.md` §11 (preference ordering)

---

### Phase 17: Detector Integration

#### 042 — Detector struct with workspace and full pipeline
- **Goal**: Wire all 10 stages into a single `detect()` call
- **Preconditions**: All stages (009-041) complete
- **Postconditions**: `Detector::new()` creates workspace; `detect(&image)` returns `Vec<Detection>`; round-trip test: render tag36h11 id=0, detect it, get id=0 back
- **Description**: Create `apriltag-detect/src/detector.rs`. `Detector { config, families, workspace }`. `Workspace` holds decimated/threshold images, union-find, edge/quad buffers. `detect()` chains: decimate → blur → threshold → union-find → cluster → quad fit → refine → homography → decode → dedup.
- **References**: `docs/detection-spec.md` §2 (pipeline overview)

#### 043 — Multi-frame workspace buffer reuse
- **Goal**: Verify zero allocation on second and subsequent detect() calls
- **Preconditions**: 042 complete
- **Postconditions**: Two detect() calls on same-size images → no new allocations; workspace.resize() called only if dimensions change
- **Description**: Add test that calls `detect()` twice, verifying buffer capacities are reused. The workspace `clear()` resets data without deallocating.
- **References**: Architecture decision — workspace pattern

---

### Phase 18: Pose Estimation

#### 044 — Initial pose from homography decomposition
- **Goal**: Extract initial rotation and translation from detection homography
- **Preconditions**: 003 (Mat33), 006 (SVD) complete
- **Postconditions**: Known camera + frontal tag → identity-like rotation, z-translation proportional to distance; translation scaled by tagsize/2
- **Description**: Create `apriltag-detect/src/pose.rs`. Decompose H into R, t. Normalize columns, fix signs (flip Y, Z axes for camera convention). Scale translation by tagsize/2.
- **References**: `docs/detection-spec.md` §12 (homography decomposition)

#### 045 — Orthogonal iteration
- **Goal**: Refine pose via alternating translation/rotation updates
- **Preconditions**: 044, 006 (SVD) complete
- **Postconditions**: Error decreases monotonically over iterations; converges within 50 iterations; final R is orthogonal (det=1)
- **Description**: Implement Lu et al. 2000. Precompute projection operators F[i]. Iterate: update t via M1_inv, update R via SVD projection onto SO(3). Default 50 iterations.
- **References**: `docs/detection-spec.md` §12.3 (orthogonal iteration)

#### 046 — Ambiguity resolution
- **Goal**: Handle the planar pose ambiguity (up to 2 local minima)
- **Preconditions**: 045 complete
- **Postconditions**: Near-perpendicular tag → single solution; oblique tag → two candidates tested, lower error selected; solutions differ by > 0.1 rad
- **Description**: Parameterize error as function of rotation angle. Solve degree-4 polynomial for additional minima. If second minimum found (> 0.1 rad from first), run iteration from it. Return pose with lower error.
- **References**: `docs/detection-spec.md` §12.4 (Schweighofer & Pinz 2006)

---

### Phase 19: WASM Bindings

#### 047 — Create `apriltag-wasm` crate skeleton
- **Goal**: Establish WASM crate with wasm-bindgen, tsify-next, serde-wasm-bindgen
- **Preconditions**: 042 (Detector integration) complete
- **Postconditions**: `cargo build --target wasm32-unknown-unknown -p apriltag-wasm` compiles; `crate-type = ["cdylib"]`
- **Description**: Create `apriltag-wasm/Cargo.toml` and `src/lib.rs`. Wire dependencies. Add to workspace members.
- **References**: Architecture decision — WASM crate

#### 048 — Tsify type definitions
- **Goal**: Auto-generate TypeScript interfaces for Detection, Pose, DetectorConfig
- **Preconditions**: 047 complete
- **Postconditions**: `#[derive(Tsify, Serialize)]` on wrapper types; TypeScript `.d.ts` generated by wasm-pack
- **Description**: Define WASM-specific wrapper types with `#[derive(Tsify)]` that mirror the core types. `WasmDetection { family, id, hamming, decision_margin, center, corners }`, `WasmPose { rotation, translation, error }`, `WasmDetectorConfig { families, quad_decimate, ... }`.
- **References**: `tsify-next` docs

#### 049 — wasm-bindgen detect entry point
- **Goal**: Expose `Detector` class to JavaScript with `detect(data, width, height)`
- **Preconditions**: 048 complete
- **Postconditions**: JS can create detector, pass `Uint8Array` grayscale image, receive `Detection[]`; RGBA convenience method works
- **Description**: `#[wasm_bindgen] struct Detector { inner }`. Constructor takes config JsValue, family names. `detect(&mut self, data: &[u8], w, h) -> JsValue` wraps inner detect(), converts results via `serde_wasm_bindgen`. Add `detect_rgba` that converts RGBA→grayscale first.
- **References**: Architecture decision — WASM API

#### 050 — wasm-bindgen pose estimation
- **Goal**: Expose pose estimation to JavaScript
- **Preconditions**: 046 (pose), 049 complete
- **Postconditions**: JS can estimate pose from detection + camera intrinsics + tag size
- **Description**: Add `estimate_pose(detection: JsValue, tagsize, fx, fy, cx, cy) -> JsValue`. Deserialize detection, call pose estimation, serialize result.
- **References**: Architecture decision — WASM API

---

### Phase 20: Detection CLI

#### 051 — Create `apriltag-detect-cli` crate with clap args
- **Goal**: CLI that accepts image files with configurable detection parameters
- **Preconditions**: 042 (Detector integration) complete
- **Postconditions**: `cargo run -p apriltag-detect-cli -- --help` shows all options; `--family`, `--decimate`, `--blur`, `--sharpening`, `--pretty`, `--pose` flags work
- **Description**: Create `apriltag-detect-cli/Cargo.toml` and `src/main.rs`. Define clap `Args` struct with all detector config options, family selection, pose flags, output format. Add to workspace.
- **References**: Architecture decision — CLI design

#### 052 — Image loading (PNG + JPEG → grayscale)
- **Goal**: Load PNG and JPEG images and convert to grayscale ImageU8
- **Preconditions**: 051 complete
- **Postconditions**: PNG loaded → correct dimensions; JPEG loaded → correct dimensions; color images auto-converted to grayscale; invalid file → clear error
- **Description**: Use `image` crate: `image::open(path)?.into_luma8()`. Convert to `ImageU8` from pixel data. Handle errors with `anyhow`.
- **References**: CLI design, `image` crate docs

#### 053 — JSON output of detections
- **Goal**: Output detection results as JSON to stdout or file
- **Preconditions**: 052, 042 complete
- **Postconditions**: Valid JSON with `file`, `image_width`, `image_height`, `detections[]`; `--pretty` flag enables indentation; multiple files → one JSON object per file
- **Description**: Define serde-serializable output structs. For each input file: load, detect, serialize. `--pretty` uses `serde_json::to_string_pretty`. Support `--output` file and multi-image input.
- **References**: CLI design — output format spec

#### 054 — CLI pose estimation integration
- **Goal**: `--pose` flag triggers pose estimation with camera intrinsics
- **Preconditions**: 053, 046 (pose) complete
- **Postconditions**: `--pose --tag-size 0.1 --fx 800 --fy 800 --cx 320 --cy 240` adds pose data to JSON; missing camera params → error
- **Description**: When `--pose` is set, validate camera intrinsics are provided. For each detection, estimate pose and include in JSON output.
- **References**: CLI design — pose options

---

### Phase 21: Rayon Parallelism

#### 055 — Feature-gated rayon integration
- **Goal**: Parallelize clustering + quad fitting when `parallel` feature enabled
- **Preconditions**: 042 complete
- **Postconditions**: Same results with and without `parallel` feature; speedup on multi-core; WASM build (no parallel) still compiles
- **Description**: Add `#[cfg(feature = "parallel")]` parallel iterators for: (1) gradient clustering over image rows, (2) quad fitting over clusters, (3) decode over quads. Use `rayon::iter::ParallelIterator` with `par_iter()`. Single-threaded path uses normal iterators.
- **References**: Architecture decision — parallelism

---

### Phase 22: WASM Verification

#### 056 — WASM build verification
- **Goal**: Ensure full pipeline compiles to wasm32-unknown-unknown
- **Preconditions**: All changes complete
- **Postconditions**: `cargo build --target wasm32-unknown-unknown -p apriltag -p apriltag-detect` succeeds; `wasm-pack build apriltag-wasm --target web` succeeds
- **Description**: Verify no std-only APIs leaked into core path. Fix any `#[cfg]` issues. Run wasm-pack build.
- **References**: `CLAUDE.md` — WASM compatibility command

---

## Verification

```bash
# Unit tests (each change)
cargo test -p apriltag-detect

# Full workspace
cargo test

# WASM targets
cargo build --target wasm32-unknown-unknown -p apriltag -p apriltag-detect
wasm-pack build apriltag-wasm --target web

# CLI smoke test
cargo run -p apriltag-detect-cli -- test_image.png -f tag36h11 --pretty

# Coverage
cargo llvm-cov --text --ignore-filename-regex 'apriltag-(gen|detect)-cli/'

# Clippy
cargo clippy -- -D warnings
```
