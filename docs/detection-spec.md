# AprilTag Detection Pipeline Specification

Language-agnostic specification for detecting AprilTag fiducial markers in
images, covering every stage from image preprocessing through pose estimation.

**Reference material**: Olson 2011, Wang et al. 2016, Krogius et al. 2019,
Felzenszwalb & Huttenlocher (graph-based segmentation), and the reference C
implementation (`apriltag` repository).

---

## 1. Data Structures

### 1.1 Image

```
image_u8:
  width:  u32        // pixels
  height: u32        // pixels
  stride: u32        // bytes per row (≥ width, for alignment)
  buf:    [u8]       // row-major pixel data
```

**Constraint**: width and height must each be < 32768 (fixed-point coordinate
representation uses 16-bit integers with 1 fractional bit).

### 1.2 Tag Family

See `generation-spec.md` §1 for full definition. The detection-relevant fields
are:

```
tag_family:
  ncodes:           u32
  codes:            [u64]
  nbits:            u32
  bit_x:            [u32]
  bit_y:            [u32]
  width_at_border:  u32
  total_width:      u32
  reversed_border:  bool
  h:                u32
  name:             string
  impl:             QuickDecode?     // lazily initialized by detector
```

### 1.3 Detector Configuration

```
detector:
  nthreads:           i32     = 1
  quad_decimate:      f32     = 2.0
  quad_sigma:         f32     = 0.0
  refine_edges:       bool    = true
  decode_sharpening:  f64     = 0.25
  debug:              bool    = false
  qtp:                QuadThreshParams

quad_thresh_params:
  min_cluster_pixels:    i32    = 5
  max_nmaxima:           i32    = 10
  critical_rad:          f32           // derived: cos stored as cos_critical_rad
  cos_critical_rad:      f32    = cos(10°) ≈ 0.9848
  max_line_fit_mse:      f32    = 10.0
  min_white_black_diff:  i32    = 5
  deglitch:              bool   = false
```

### 1.4 Quad

```
quad:
  p:               [[f32; 2]; 4]    // four corner positions in pixel coords
  reversed_border: bool             // true if black border is inside white
  H:               Mat3x3?          // homography: tag coords → pixel coords
  Hinv:            Mat3x3?          // inverse homography
```

Corner winding order: counter-clockwise in image coordinates.

### 1.5 Detection Result

```
detection:
  family:          &TagFamily
  id:              i32              // decoded tag ID
  hamming:         i32              // number of corrected bit errors
  decision_margin: f32              // average |bit value − threshold|
  H:               Mat3x3           // tag → pixel homography (rotation-corrected)
  c:               [f64; 2]         // center in pixel coordinates
  p:               [[f64; 2]; 4]    // corners in pixel coordinates (CCW)
```

### 1.6 Pose Result

```
pose:
  R: Mat3x3    // rotation matrix (camera ← tag)
  t: Vec3      // translation vector (camera ← tag), in tag-size units

detection_info:
  det:     &Detection
  tagsize: f64    // physical tag side length (meters)
  fx:      f64    // focal length x (pixels)
  fy:      f64    // focal length y (pixels)
  cx:      f64    // principal point x (pixels)
  cy:      f64    // principal point y (pixels)
```

### 1.7 Internal Structures

```
// Edge point (fixed-point: actual coords = x/2, y/2)
pt:
  x:     u16     // 2× actual x
  y:     u16     // 2× actual y
  gx:    i16     // gradient direction x (−255, 0, or 255)
  gy:    i16     // gradient direction y
  slope: f32     // angular sort key

// Cumulative line-fitting moments
line_fit_pt:
  Mx:  f64      // Σ W·x
  My:  f64      // Σ W·y
  Mxx: f64      // Σ W·x²
  Mxy: f64      // Σ W·x·y
  Myy: f64      // Σ W·y²
  W:   f64      // Σ W

// Union-find
unionfind:
  maxid:  u32
  parent: [u32]    // initialized to 0xFFFFFFFF (unset)
  size:   [u32]    // tree size excluding root; initialized to 0

// Quick decode lookup table
quick_decode:
  nbits:          i32
  chunk_size:     i32     // = ceil(nbits / 4)
  capacity:       i32     // = 2^chunk_size
  chunk_mask:     i32     // = capacity − 1
  shifts:         [i32; 4]
  chunk_offsets:  [[u16]; 4]    // maps chunk value → range in chunk_ids
  chunk_ids:      [[u16]; 4]   // indices into codes array
  maxhamming:     i32
  ncodes:         i32

// Gray model (spatially-varying intensity)
graymodel:
  A: [[f64; 3]; 3]    // J'J (upper triangular, symmetric)
  B: [f64; 3]         // J'gray
  C: [f64; 3]         // solved coefficients [Cx, Cy, C0]
  // interpolate(x, y) = C[0]*x + C[1]*y + C[2]
```

---

## 2. Pipeline Overview

```
                        ┌──────────────┐
                        │  Input Image │
                        │  (grayscale) │
                        └──────┬───────┘
                               │
                    ┌──────────▼──────────┐
                    │  1. Decimate + Blur │
                    └──────────┬──────────┘
                               │
                    ┌──────────▼──────────┐
                    │  2. Threshold       │
                    │  (adaptive, tiled)  │
                    └──────────┬──────────┘
                               │
                    ┌──────────▼──────────┐
                    │  3. Union-Find      │
                    │  (connected comps)  │
                    └──────────┬──────────┘
                               │
                    ┌──────────▼──────────┐
                    │  4. Gradient        │
                    │  Clustering         │
                    └──────────┬──────────┘
                               │
                    ┌──────────▼──────────┐
                    │  5. Quad Fitting    │
                    └──────────┬──────────┘
                               │
                    ┌──────────▼──────────┐
                    │  6. Edge Refinement │
                    │  (on original image)│
                    └──────────┬──────────┘
                               │
                    ┌──────────▼──────────┐
                    │  7. Homography      │
                    └──────────┬──────────┘
                               │
                    ┌──────────▼──────────┐
                    │  8. Tag Decoding    │
                    └──────────┬──────────┘
                               │
                    ┌──────────▼──────────┐
                    │  9. Deduplication   │
                    └──────────┬──────────┘
                               │
                    ┌──────────▼──────────┐
                    │  10. Pose Estimation│
                    │  (optional)         │
                    └──────────┴──────────┘
```

---

## 3. Stage 1: Image Preprocessing

### 3.1 Decimation

If `quad_decimate > 1`, reduce image resolution for faster quad detection.
The decimation factor `f` is applied as integer downsampling:

```
out_width  = in_width  / f
out_height = in_height / f
out[y][x]  = average of f×f block at (x*f, y*f)
```

After quads are found on the decimated image, corner coordinates are scaled
back by `f` before decoding (which operates on the original image).

### 3.2 Gaussian Blur / Sharpening

Controlled by `quad_sigma`:

- **`quad_sigma > 0`** → Gaussian blur (noise suppression)
- **`quad_sigma < 0`** → Unsharp mask (edge enhancement)
- **`quad_sigma == 0`** → No filtering

**Kernel construction**:

```
sigma = |quad_sigma|
ksz = 4 * sigma            // 2 std devs in each direction
if ksz is even: ksz += 1   // force odd
if ksz <= 1: skip           // no-op
```

**Blur** (`quad_sigma > 0`): Apply separable Gaussian blur with kernel size
`ksz` and standard deviation `sigma`.

**Sharpen** (`quad_sigma < 0`): Unsharp masking:

```
blurred = gaussian_blur(image, sigma, ksz)
sharpened[y][x] = clamp(2 * image[y][x] − blurred[y][x], 0, 255)
```

---

## 4. Stage 2: Adaptive Thresholding

Produce a ternary image: **black** (0), **white** (255), or **unknown** (127).
The unknown value marks low-contrast regions that are skipped in later stages.

### 4.1 Tile-Based Min/Max

Divide the image into non-overlapping `tilesz × tilesz` tiles (default
`tilesz = 4`).

```
tw = width  / tilesz
th = height / tilesz
```

For each tile `(tx, ty)`, compute the pixel minimum and maximum:

```
tile_min[ty][tx] = min of all pixels in tile
tile_max[ty][tx] = max of all pixels in tile
```

### 4.2 Dilation of Min/Max (3×3 Neighborhood)

To avoid threshold discontinuities at tile boundaries, dilate the max image
and erode the min image using a 3×3 tile neighborhood:

```
dilated_max[ty][tx] = max of tile_max[ty+dy][tx+dx] for dy,dx ∈ {−1,0,1}
eroded_min[ty][tx]  = min of tile_min[ty+dy][tx+dx] for dy,dx ∈ {−1,0,1}
```

### 4.3 Binarization

For each pixel, look up the dilated min/max from its containing tile:

```
if max − min < min_white_black_diff:
  output = 127          // low contrast → unknown
else:
  thresh = min + (max − min) / 2
  output = if pixel > thresh then 255 else 0
```

Partial tiles at the right/bottom edges use the min/max from the nearest full
tile.

### 4.4 Deglitch (Optional)

If `deglitch` is enabled, apply morphological close (dilate then erode) with a
3×3 structuring element on the threshold image. This removes single-pixel
noise but is rarely needed.

---

## 5. Stage 3: Connected Components (Union-Find)

Build a disjoint-set forest over the thresholded image pixels. Two adjacent
pixels are connected if they have the **same threshold value** (both black or
both white). Pixels with value 127 (unknown) are never connected.

### 5.1 Connectivity Pattern

Process pixels left-to-right, top-to-bottom. For each non-unknown pixel at
`(x, y)` with value `v`:

**Always check**:
- `(x−1, y)` — left neighbor (4-connectivity)

**Conditionally check** (to avoid redundant unions when an intermediary path
already exists through a same-valued neighbor):
- `(x, y−1)` — up: skip if `(x−1, y)`, `(x−1, y−1)`, and `(x, y−1)` all
  equal `v` (the path already exists through left)
- `(x−1, y−1)` — upper-left diagonal: only for **white** pixels, skip if
  `(x−1, y)` or `(x, y−1)` equals `(x−1, y−1)`
- `(x+1, y−1)` — upper-right diagonal: only for **white** pixels, skip if
  `(x, y−1)` equals `(x+1, y−1)`

The asymmetric treatment (diagonals only for white) prevents black regions from
connecting through narrow diagonal gaps.

### 5.2 Union-Find Operations

**Find** (path halving):

```
get_representative(id):
  if parent[id] == UNSET:
    parent[id] = id
    return id
  while parent[id] ≠ id:
    parent[id] = parent[parent[id]]    // path halving
    id = parent[id]
  return id
```

**Union** (weighted by size):

```
connect(a, b):
  ra = get_representative(a)
  rb = get_representative(b)
  if ra == rb: return ra
  if size[ra]+1 > size[rb]+1:
    parent[rb] = ra
    size[ra] += size[rb]+1
    return ra
  else:
    parent[ra] = rb
    size[rb] += size[ra]+1
    return rb
```

---

## 6. Stage 4: Gradient Clustering

Extract boundary points between adjacent black and white connected components,
grouping them by the pair of component IDs they border.

### 6.1 Edge Point Extraction

For each non-unknown pixel at `(x, y)` with value `v0`:

1. Skip if its connected component has fewer than 25 pixels
2. Check neighbors at offsets `(1,0)`, `(0,1)`, `(-1,1)`, `(1,1)`
3. For each neighbor at `(x+dx, y+dy)` with value `v1`:
   - If `v0 + v1 == 255` (one black, one white):
     - The neighbor's component must also have ≥ 25 pixels
     - Compute a cluster ID from the ordered pair of component
       representatives: `(min(rep0, rep1), max(rep0, rep1))`
     - Add a point at the midpoint with gradient direction:
       ```
       pt.x  = 2*x + dx        // fixed-point (half-pixel precision)
       pt.y  = 2*y + dy
       pt.gx = dx * (v1 − v0)  // points toward white if positive
       pt.gy = dy * (v1 − v0)
       ```

### 6.2 Cluster Storage

Points are grouped into clusters keyed by their component-pair ID, using a
hash map. The hash function is:

```
hash(id) = (2654435761 × id) >> 32
```

Clusters from parallel threads are merged in sorted order (by hash, then ID)
to ensure deterministic results.

---

## 7. Stage 5: Quad Fitting

For each cluster of boundary points, attempt to fit a quadrilateral.

### 7.1 Size Filtering

Reject clusters with:
- Fewer than `min_cluster_pixels` points (default 5)
- Fewer than 24 points (minimum for quad fitting)
- More than `2 × (2w + 2h)` points (too large to be a single tag boundary)

### 7.2 Border Direction Check

Compute the dot product of each point's position (relative to the cluster
centroid) with its gradient direction. The sign indicates whether the black
border is inside (normal) or outside (reversed) the white border:

```
dot = Σ (dx · gx + dy · gy)    for each point

reversed_border = (dot < 0)
```

Reject if the border orientation doesn't match any registered tag family.

### 7.3 Angular Sorting

Sort points by angle around the cluster centroid. Rather than computing
`atan2` (expensive), use a fast slope proxy:

```
cx = (xmin + xmax) / 2 + 0.05118        // small noise offset
cy = (ymin + ymax) / 2 − 0.028581

For each point:
  dx = x − cx
  dy = y − cy

  // Map to a monotonic value in [0, 4) representing the quadrant + slope
  quadrant = lookup[dy > 0][dx > 0]      // {0, 1, 2, 3}-ish
  // Fold into upper half-plane, compute dy/dx
  slope = quadrant + |dy|/|dx|           // after appropriate sign flips
```

Sort by `slope` using a merge sort with inline sorting networks for small
sizes (≤ 5 elements).

### 7.4 Cumulative Moments for Line Fitting

Compute cumulative weighted moments for efficient range queries:

```
For each point i (in sorted order):
  W_i   = sqrt(grad_x² + grad_y²) + 1     // gradient magnitude weight
  lfps[i].Mx  = Σ_{j≤i} W_j · x_j
  lfps[i].My  = Σ_{j≤i} W_j · y_j
  lfps[i].Mxx = Σ_{j≤i} W_j · x_j²
  lfps[i].Mxy = Σ_{j≤i} W_j · x_j · y_j
  lfps[i].Myy = Σ_{j≤i} W_j · y_j²
  lfps[i].W   = Σ_{j≤i} W_j
```

These allow O(1) line fitting for any contiguous range `[i0, i1]` using
subtraction of prefix sums.

### 7.5 Corner Detection (Maxima Method)

Find the 4 indices that partition the sorted points into 4 line segments.

1. **Compute line-fit error at each point**: For each index `i`, fit a line to
   the `2·ksz` points centered on `i` (wrapping around). The error is the
   smallest eigenvalue of the 2×2 covariance matrix:
   ```
   ksz = min(20, sz/12)
   err[i] = eig_small of Cov([i−ksz, i+ksz])
   ```

2. **Smooth errors**: Apply a Gaussian low-pass filter (σ=1, cutoff=0.05) to
   the error array.

3. **Find local maxima**: Points where `err[i] > err[i−1]` and
   `err[i] > err[i+1]` are corner candidates.

4. **Select best 4**: If more than `max_nmaxima` candidates, keep only the top
   `max_nmaxima` by error magnitude. Then exhaustively try all 4-combinations:
   ```
   For each (m0, m1, m2, m3) with m0 < m1 < m2 < m3:
     For each of the 4 segments:
       fit_line and compute MSE
       reject if MSE > max_line_fit_mse
     Check adjacent-line angle:
       reject if |dot(normal_i, normal_{i+1})| > cos_critical_rad
     total_err = sum of 4 segment errors
     keep combination with minimum total_err
   ```

### 7.6 Line Fitting (Eigenanalysis)

Given cumulative moments for a range of points, compute the best-fit line:

```
Ex  = Mx / W
Ey  = My / W
Cxx = Mxx/W − Ex²
Cxy = Mxy/W − Ex·Ey
Cyy = Myy/W − Ey²

// Eigenvalues of [Cxx Cxy; Cxy Cyy]
eig_large = 0.5 * (Cxx + Cyy + sqrt((Cxx−Cyy)² + 4·Cxy²))
eig_small = 0.5 * (Cxx + Cyy − sqrt((Cxx−Cyy)² + 4·Cxy²))

// Line direction = eigenvector of larger eigenvalue
// Line normal = eigenvector of smaller eigenvalue
// MSE = eig_small
```

The line is parameterized as `(Ex, Ey, nx, ny)` where `(nx, ny)` is the unit
normal.

### 7.7 Corner Computation (Line Intersection)

Corners are the intersections of adjacent fitted lines. For lines `i` and
`(i+1) mod 4`:

```
// Lines stored as [px, py, nx, ny]. Direction = perpendicular to normal.
A00 =  lines[i][3]          A01 = -lines[i+1][3]
A10 = -lines[i][2]          A11 =  lines[i+1][2]
B0  = lines[i+1][0] − lines[i][0]
B1  = lines[i+1][1] − lines[i][1]

det = A00·A11 − A10·A01
if |det| < 0.001: reject quad

lambda = (A11·B0 − A01·B1) / det
corner[i] = (lines[i][0] + lambda·A00, lines[i][1] + lambda·A10)
```

### 7.8 Quad Validation

Reject quads that:
1. Have area < `0.95 × tag_width²` (tag_width = `min_family_width / decimate`,
   minimum 3)
2. Have any interior angle outside the range defined by `cos_critical_rad`
   (too acute or too obtuse)
3. Have non-convex winding (cross product check)

---

## 8. Stage 6: Edge Refinement

If `refine_edges` is true, snap each quad edge to the nearest strong gradient
in the **original** (undecimated) image.

For each of the 4 edges (from corner `a` to corner `b`):

1. **Compute edge normal**: `(nx, ny)` perpendicular to the edge, pointing
   outward (flipped if `reversed_border`).

2. **Sample along the edge**: `nsamples = max(16, edge_length / 8)` points,
   uniformly distributed (excluding corners):
   ```
   alpha = (1 + s) / (nsamples + 1)
   x0 = alpha · p[a].x + (1 − alpha) · p[b].x
   y0 = alpha · p[a].y + (1 − alpha) · p[b].y
   ```

3. **Search along the normal**: For each sample point, search in the normal
   direction over `range = quad_decimate + 1` pixels, with `steps_per_unit = 4`:
   ```
   For n in [-range, +range] at step 0.25:
     Compute gradient at (x0 + n·nx, y0 + n·ny) via bilinear interpolation
     of two points offset by ±grange (=1) along the normal.
     g1 = interpolated_value(x0 + (n+1)·nx, y0 + (n+1)·ny)  // inner
     g2 = interpolated_value(x0 + (n−1)·nx, y0 + (n−1)·ny)  // outer
     if g1 < g2: skip (backwards gradient)
     weight = (g2 − g1)²
     accumulate: Mn += weight · n, Mcount += weight
   ```

4. **Find best edge point**: `n0 = Mn / Mcount`, then
   `best = (x0 + n0·nx, y0 + n0·ny)`.

5. **Re-fit line**: Accumulate `(bestx, besty)` into covariance statistics
   `(Mx, My, Mxx, Mxy, Myy, N)`. After all samples:
   ```
   normal_theta = 0.5 · atan2(−2·Cxy, Cyy − Cxx)
   refined_line = (Ex, Ey, cos(theta), sin(theta))
   ```

6. **Recompute corners**: Intersect adjacent refined lines using the same
   method as §7.7. Skip if `|det| < 0.001`.

---

## 9. Stage 7: Homography Computation

Compute the 3×3 homography mapping from **tag coordinates** `[-1, 1]²` to
**pixel coordinates**.

### 9.1 Correspondences

The four quad corners map to the tag-space corners:

```
tag:    (-1,-1)  (1,-1)  (1,1)  (-1,1)
pixel:  quad.p[0] quad.p[1] quad.p[2] quad.p[3]
```

### 9.2 DLT (Direct Linear Transform)

Set up the 8×9 system `Ah = 0` where `h` are the 9 entries of the homography
(with `h[8] = 1`):

```
For each correspondence (tag_x, tag_y) → (px, py):
  Row 2i:   [tag_x, tag_y, 1,  0, 0, 0,  -tag_x·px, -tag_y·px,  px]
  Row 2i+1: [0, 0, 0,  tag_x, tag_y, 1,  -tag_x·py, -tag_y·py,  py]
```

### 9.3 Gaussian Elimination

Solve the 8×9 augmented system using partial pivoting:

1. For columns 0–7: find the row with maximum absolute value in the column,
   swap to the diagonal position, eliminate below.
2. Back-substitute to find `h[0]..h[7]`. Set `h[8] = 1`.
3. Return `H = [[h0,h1,h2],[h3,h4,h5],[h6,h7,1]]`.

If the matrix is singular (`max_pivot < 1e-10`), reject this quad.

### 9.4 Projection

```
homography_project(H, x, y):
  xx = H[0][0]·x + H[0][1]·y + H[0][2]
  yy = H[1][0]·x + H[1][1]·y + H[1][2]
  zz = H[2][0]·x + H[2][1]·y + H[2][2]
  return (xx/zz, yy/zz)
```

---

## 10. Stage 8: Tag Decoding

### 10.1 Gray Model Construction

Build separate linear intensity models for the white and black border regions
to handle spatially-varying illumination.

**Sampling patterns** (in tag-border coordinates, where `[0, width_at_border]`
spans the data region):

```
8 line patterns: { start_x, start_y, delta_x, delta_y, is_white }

Left white column:   (-0.5, 0.5, 0, 1, WHITE)
Left black column:   (0.5,  0.5, 0, 1, BLACK)
Right white column:  (W+0.5, 0.5, 0, 1, WHITE)
Right black column:  (W−0.5, 0.5, 0, 1, BLACK)
Top white row:       (0.5, -0.5, 1, 0, WHITE)
Top black row:       (0.5,  0.5, 1, 0, BLACK)
Bottom white row:    (0.5, W+0.5, 1, 0, WHITE)
Bottom black row:    (0.5, W−0.5, 1, 0, BLACK)

where W = width_at_border
```

For each sample point:
1. Convert from border coordinates to tag coordinates `[-1, 1]`:
   ```
   tagx = 2 · (border_x / W − 0.5)
   tagy = 2 · (border_y / W − 0.5)
   ```
2. Project to pixel coordinates via `homography_project(H, tagx, tagy)`
3. Add to the appropriate gray model (white or black):
   ```
   graymodel_add(model, tagx, tagy, pixel_value)
   ```

Solve both models via least-squares: `intensity(x,y) = C0·x + C1·y + C2`.

**Polarity check**: Verify that `white_model(0,0) > black_model(0,0)` for
normal-border tags (or `<` for reversed-border). Reject if wrong.

### 10.2 Bit Sampling

For each data bit `i`:

1. Convert bit coordinates to tag coordinates:
   ```
   tagx = 2 · ((bit_x[i] + 0.5) / W − 0.5)
   tagy = 2 · ((bit_y[i] + 0.5) / W − 0.5)
   ```
2. Project to pixel coordinates and read value via **bilinear interpolation**:
   ```
   value_for_pixel(im, px, py):
     x1 = floor(px − 0.5), x2 = ceil(px − 0.5)
     y1 = floor(py − 0.5), y2 = ceil(py − 0.5)
     // bilinear interpolation of 4 surrounding pixels
   ```
3. Compute threshold at this location:
   ```
   thresh = (black_model(tagx, tagy) + white_model(tagx, tagy)) / 2
   ```
4. Store the margin: `values[i] = pixel_value − thresh`

### 10.3 Decode Sharpening

Apply a Laplacian sharpening kernel to the `total_width × total_width` grid of
margin values:

```
kernel = [  0  -1   0 ]
         [ -1   4  -1 ]
         [  0  -1   0 ]

sharpened[y][x] = Σ kernel[dy][dx] · values[y+dy][x+dx]
values[y][x] += decode_sharpening · sharpened[y][x]
```

Default `decode_sharpening = 0.25`.

### 10.4 Code Extraction and Decision Margin

```
rcode = 0
black_score = 0, white_score = 0
black_count = 1, white_count = 1      // start at 1 (Laplace smoothing)

For each bit i (MSB first):
  rcode = (rcode << 1)
  v = sharpened_values[bit_y[i]][bit_x[i]]
  if v > 0:
    rcode |= 1
    white_score += v
    white_count += 1
  else:
    black_score -= v      // v is negative, so this adds |v|
    black_count += 1

decision_margin = min(white_score / white_count, black_score / black_count)
```

Reject if `decision_margin < 0`.

### 10.5 Quick Decode (Code Matching)

The quick decode table enables fast lookup without checking every code in the
family. It splits each `nbits`-bit code into 4 chunks.

**Initialization**:

```
chunk_size = ceil(nbits / 4)
capacity = 2^chunk_size
chunk_mask = capacity − 1
shifts = [0, chunk_size, 2·chunk_size, 3·chunk_size]

For each chunk j (0..3):
  Count frequency of each chunk value across all codes
  Compute prefix sum → chunk_offsets[j]
  Fill chunk_ids[j] with code indices sorted by chunk value
```

**Lookup**:

```
quick_decode_codeword(family, rcode, result):
  For each rotation r in 0..3:
    For each chunk j in 0..3:
      val = (rcode >> shifts[j]) & chunk_mask
      For each candidate in chunk_offsets[j][val]..chunk_offsets[j][val+1]:
        id = chunk_ids[j][candidate]
        hamming = popcount(codes[id] XOR rcode)
        if hamming ≤ maxhamming:
          result = { id, hamming, rotation=r }
          return

    rcode = rotate90(rcode, nbits)

  result.hamming = 255    // no match
```

The rotation function `rotate90` cyclically shifts the 4 bit-quadrants (see
`generation-spec.md` §4.4).

**Default `maxhamming = 2`**. Values ≥ 3 are supported but increase false
positive rate and memory usage significantly.

---

## 11. Stage 9: Deduplication

When the same physical tag is detected multiple times (e.g., from slight
variations in quad fitting), keep only the best detection.

### 11.1 Overlap Test

For each pair of detections with the **same ID and family**:

```
if polygons_overlap(det0.p, det1.p):
  // keep the better one
```

Polygon overlap is tested using the separating axis theorem on the two
4-point polygons.

### 11.2 Preference Ordering

When two overlapping detections are found, prefer (in order):

1. **Lower Hamming distance** (fewer corrected errors)
2. **Higher decision margin** (more confident decode)
3. **Deterministic tiebreaker**: compare corner coordinates lexicographically

---

## 12. Stage 10: Pose Estimation

Given a detection and camera intrinsics, estimate the 3D pose of the tag.

### 12.1 Coordinate Systems

- **Tag frame**: Origin at tag center. X-right, Y-up. The tag lies in the
  `z=0` plane. Corner coordinates at `(±tagsize/2, ±tagsize/2, 0)`.
- **Camera frame**: Origin at camera optical center. Z-forward (into the
  scene), X-right, Y-down (standard computer vision convention).

### 12.2 Inputs

```
detection_info:
  det:     detection with H (homography)
  tagsize: physical tag side length in world units
  fx, fy:  focal lengths in pixels
  cx, cy:  principal point in pixels
```

Object points (tag corners in tag frame):

```
p[0] = (-tagsize/2, +tagsize/2, 0)
p[1] = (+tagsize/2, +tagsize/2, 0)
p[2] = (+tagsize/2, -tagsize/2, 0)
p[3] = (-tagsize/2, -tagsize/2, 0)
```

Image rays (normalized image coordinates):

```
v[i] = ((det.p[i].x − cx) / fx,  (det.p[i].y − cy) / fy,  1)
```

### 12.3 Initial Estimate (Homography Decomposition)

Extract R and t from the detection homography:

```
M = homography_to_pose(H, -fx, fy, cx, cy)
Scale translation by tagsize/2
Apply sign fix: flip Y and Z axes
```

This gives an initial `R, t` estimate that may not be perfectly orthogonal.

### 12.4 Orthogonal Iteration (Lu et al., 2000)

Refine the pose by alternating between solving for translation and rotation:

**Precompute**:
- Projection operators `F[i] = v[i] · v[i]' / (v[i]' · v[i])` for each
  corner
- Mean of object points `p_mean`, residuals `p_res[i] = p[i] − p_mean`
- `M1_inv = (I − mean(F))^{-1}`

**Iterate** (default 50 iterations):

1. **Update translation**:
   ```
   M2 = (1/n) Σ (F[i] − I) · R · p[i]
   t = M1_inv · M2
   ```

2. **Update rotation** (project onto SO(3)):
   ```
   q[i] = F[i] · (R · p[i] + t)
   M3 = Σ (q[i] − q_mean) · p_res[i]'
   SVD: M3 = U · S · V'
   R = U · V'
   if det(R) < 0: negate third column of R
   ```

3. **Compute object-space error**:
   ```
   error = Σ ||(I − F[i]) · (R · p[i] + t)||²
   ```

### 12.5 Ambiguity Resolution (Schweighofer & Pinz, 2006)

Planar pose estimation has up to two local minima. After finding the first
solution, search for a second:

1. Parameterize the error as a function of one rotation angle
2. Find the polynomial coefficients of the error surface
3. Solve the derivative polynomial (degree 4) for additional minima
4. If a second minimum exists (and differs by > 0.1 rad from the first),
   run orthogonal iteration from that starting point

**Output**: Return the pose with the lower object-space error. If both are
close, the caller may want to consider both.

### 12.6 Convenience API

```
estimate_tag_pose(info) → (pose, error):
  Run orthogonal_iteration from homography-based initial estimate (50 iters)
  Search for second local minimum
  If second exists, run orthogonal_iteration on it too
  Return pose with lower error
```
