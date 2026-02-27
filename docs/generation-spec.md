# AprilTag Family Generation Specification

Language-agnostic specification for generating AprilTag families — the sets of
binary codes and their physical layouts used by the detection pipeline.

**Reference material**: Krogius et al. 2019 ("Flexible Layouts for Fiducial
Tags"), the `apriltag-generation` Java implementation, and the tag family C
headers shipped with the reference detector.

---

## 1. Tag Family Structure

A **tag family** is a set of binary codewords that share a common physical
layout and a guaranteed minimum Hamming distance between any pair (including
rotational variants).

### Fields

| Field | Type | Description |
|-------|------|-------------|
| `name` | string | Human-readable identifier, e.g. `"tag36h11"` |
| `ncodes` | u32 | Number of valid codes in the family |
| `codes` | \[u64\] | Array of valid codewords, indexed by tag ID |
| `nbits` | u32 | Number of data bits per code |
| `h` | u32 | Minimum Hamming distance (accounting for rotations) |
| `bit_x` | \[u32\] | X coordinate of each bit in the tag grid |
| `bit_y` | \[u32\] | Y coordinate of each bit in the tag grid |
| `width_at_border` | u32 | Width of the data region at the inner border edge |
| `total_width` | u32 | Total tag width in cells (including borders) |
| `reversed_border` | bool | `true` if border is black-inside/white-outside (Standard/Circle); `false` for Classic (white-outside/black-inside with white outermost) |

### Naming Convention

Format: `tag<nbits>h<min_hamming>` with an optional layout prefix.

| Family | Layout | Bits | Min Hamming | Codes | Grid |
|--------|--------|------|-------------|-------|------|
| `tag36h11` | Classic | 36 | 11 | 587 | 10×10 |
| `tag25h9` | Classic | 25 | 9 | 35 | 9×9 |
| `tag16h5` | Classic | 16 | 5 | 30 | 8×8 |
| `tagStandard41h12` | Standard | 41 | 12 | 2115 | 9×9 |
| `tagStandard52h13` | Standard | 52 | 13 | 48714 | 10×10 |
| `tagCircle21h7` | Circle | 21 | 7 | 38 | 9×9 |
| `tagCircle49h12` | Circle | 49 | 12 | 65535 | 11×11 |
| `tagCustom48h12` | Custom | 48 | 12 | 42211 | 10×10 |

---

## 2. Layout Types

Every layout maps data bits, border pixels, and ignored pixels onto a square
grid. The grid must be **4-way rotationally symmetric** (invariant under 90°
rotation). Each cell is one of:

| Cell Type | Symbol | Rendered As |
|-----------|--------|-------------|
| Data | `d` | Black (0) or white (1) per codeword bit |
| Black border | `b` | Always black |
| White border | `w` | Always white |
| Ignored | `x` | Transparent / not part of tag |

### 2.1 Classic Layout

Uses **L∞ (Chebyshev) distance** from the grid boundary to assign cell types.

```
Layer 0 (outermost):  white border (w)
Layer 1:              black border (b)
Layer 2+:             data bits (d)
```

Example — Classic 10×10 (`tag36h11`):

```
w w w w w w w w w w
w b b b b b b b b w
w b d d d d d d b w
w b d d d d d d b w
w b d d d d d d b w
w b d d d d d d b w
w b d d d d d d b w
w b d d d d d d b w
w b b b b b b b b w
w w w w w w w w w w
```

Properties:
- `reversed_border = false`
- `width_at_border` = grid size − 4
- `total_width` = grid size
- 6×6 = 36 data cells in a 10×10 grid

### 2.2 Standard Layout

Uses **L∞ distance** but inverts the border order — data bits extend *outside*
the white border, gaining more data cells per grid size.

```
Layer 0 (outermost):  data bits (d)
Layer 1:              white border (w)
Layer 2:              black border (b)
Layer 3+:             data bits (d)
```

Example — Standard 9×9 (`tagStandard41h12`):

```
d d d d d d d d d
d w w w w w w w d
d w b b b b b w d
d w b d d d b w d
d w b d d d b w d
d w b d d d b w d
d w b b b b b w d
d w w w w w w w d
d d d d d d d d d
```

Properties:
- `reversed_border = true`
- `width_at_border` = grid size − 4
- `total_width` = grid size
- 41 data cells in a 9×9 grid (vs 25 for Classic 9×9)

### 2.3 Circle Layout

Uses **L2 (Euclidean) distance** from the grid center to define a circular
data region.

```
cutoff = grid_size / 2.0 − 0.25

For each cell (x, y):
  dist = sqrt((x − cx)² + (y − cy)²)     where cx = cy = (grid_size−1)/2
  if dist > cutoff:    ignored (x)
  elif in border ring: black (b) or white (w)
  else:                data (d)
```

The border ring is computed similarly to Standard layout but with circular
distance. Cells outside the circle are marked ignored (`x`).

Properties:
- `reversed_border = true`
- Fewer data bits than square layouts of the same grid size
- Circular boundary gives more uniform detection at oblique angles

### 2.4 Custom Layout

Defined by a flat character string of length `grid_size²`. Each character is
one of `d`, `b`, `w`, or `x`. The string is read row-major, top-to-bottom,
left-to-right.

Validation rules:
- String length must be a perfect square
- Layout must be invariant under 90° rotation
- Must contain a detectable border (at least one `w↔b` transition layer)

---

## 3. Bit Coordinate System

The `bit_x` / `bit_y` arrays map each bit index to a physical position on the
tag grid, **relative to the inner border edge**.

### 3.1 Enumeration Order

Bits are enumerated by scanning quadrants in rotational order:

1. **Scan the first quadrant** (top-left quarter, along the diagonal to the
   edge):
   ```
   for y in 0..grid_size/2:
     for x in y..(grid_size − 1 − y):
       if cell(x, y) == DATA: record (x, y) as next bit
   ```
   This yields `nbits / 4` bits (one rotational quarter).

2. **Generate remaining three quadrants** by 90° rotation:
   ```
   For each subsequent quarter i (1, 2, 3):
     location[idx] = rotate90(location[idx − nbits/4])
   ```
   where `rotate90(x, y) = (grid_size − 1 − y, x)`.

3. **Center pixel** (odd-sized grids only): If a data cell exists at the
   center `(grid_size/2, grid_size/2)`, it becomes the final bit.

### 3.2 Coordinate Shift

Raw coordinates are shifted so that (0, 0) aligns with the inner border
corner:

```
border_start = {
  Classic:  2   (skip white + black border layers)
  Standard: 3   (skip outer data + white + black layers)
  Circle:   border_distance + 1
}

bit_x[i] = raw_x[i] − border_start
bit_y[i] = raw_y[i] − border_start
```

This means coordinates can be **negative** for Standard/Circle layouts (data
bits outside the inner border).

### 3.3 Example: tag36h11

Classic 10×10, `border_start = 2`:
- First-quadrant raw positions: (2,2), (3,2), (4,2), (5,2), (6,2), (3,3),
  (4,3), (5,3), (4,4) — 9 bits
- After shift: (0,0), (1,0), (2,0), (3,0), (4,0), (1,1), (2,1), (3,1),
  (2,2)
- Remaining 27 bits generated by rotation; `bit_x`/`bit_y` range: [−1, 5]

### 3.4 Example: tagStandard41h12

Standard 9×9, `border_start = 3`:
- `bit_x`/`bit_y` range: [−2, 6]
- Negative coordinates correspond to the outer data ring between the white
  border and the grid edge

---

## 4. Code Generation Algorithm

The goal is to find the **largest set of `nbits`-bit codewords** such that the
minimum Hamming distance between any pair — including all four 90° rotational
variants — meets or exceeds the target `h`.

### 4.1 Overview

The algorithm is a **greedy lexicode search** over the code space:

1. Iterate through all 2^nbits candidate codes in a pseudo-random order
2. For each candidate, check it against all previously accepted codes
3. Accept if minimum Hamming distance ≥ `h` (considering rotations)
4. Reject otherwise

### 4.2 Iteration Order

Candidates are visited in a deterministic pseudo-random order to avoid bias
toward small code values:

```
PRIME = 982_451_653

v0 = deterministic_seed(nbits, min_hamming)
    // = Random(nbits * 10000 + min_hamming * 100 + 7).next_long()

For iteration i:
  candidate = (v0 + PRIME × i) mod 2^nbits
```

The prime multiplier ensures all 2^nbits values are visited exactly once
(since gcd(PRIME, 2^nbits) = 1 when PRIME is odd).

### 4.3 Candidate Validation

Each candidate undergoes three checks:

#### 4.3.1 Complexity Check

Reject codes that are too visually uniform (all-black, all-white, or simple
stripes). Render the code onto the tag layout and count black↔white edge
transitions ("energy"):

```
energy = count of adjacent cell pairs (4-connected) with different bit values
max_energy = 2 × (data cell count)
Require: energy ≥ max_energy / 3
```

#### 4.3.2 Self-Rotation Distance

Compute all four 90° rotations of the candidate. Verify that every pair among
the four rotational variants has Hamming distance ≥ `h`:

```
rotations = [v, rot90(v), rot180(v), rot270(v)]
For all (i, j) where i < j:
  require hamming(rotations[i], rotations[j]) ≥ h
```

This ensures the tag is unambiguously decodable regardless of orientation.

#### 4.3.3 Distance from Accepted Codes

Check the candidate (and all its rotations) against every previously accepted
code (and all their rotations):

```
For each accepted code c:
  For each rotation r of c:
    require hamming(candidate, r) ≥ h
```

Early-exit: reject as soon as any pair falls below `h`.

### 4.4 Rotation Function

Rotating a codeword by 90° permutes its bits according to the quadrant
structure:

```
rotate90(w, nbits):
  if nbits % 4 == 1:
    p = nbits − 1      // bits in the four quadrants
    center = w & 1      // center bit stays fixed
    w >>= 1
  else:
    p = nbits
    center = 0

  quarter = p / 4
  // Cycle the four quadrants: Q0←Q3, Q1←Q0, Q2←Q1, Q3←Q2
  result = ((w << quarter) | (w >> (3 * quarter))) & ((1 << p) − 1)

  return (result << (nbits % 4 == 1 ? 1 : 0)) | center
```

### 4.5 Hamming Distance

```
hamming_distance(a, b) = popcount(a XOR b)

// Early-exit variant for threshold checking:
hamming_distance_at_least(a, b, threshold):
  w = a XOR b
  count = 0
  while w ≠ 0 and count < threshold:
    count += popcount(w & 0xFFFF)   // 16-bit chunk lookup
    w >>= 16
  return count ≥ threshold
```

### 4.6 Parallelization

The reference implementation parallelizes validation in two phases:

1. **Partial approval** (multi-threaded): Check candidate against codes
   accepted before the current chunk. Multiple chunks run in parallel.
2. **Full approval** (single-threaded, ordered): Check candidate against codes
   accepted in concurrent chunks. Process chunks in order to maintain
   deterministic results.

Chunk size: ~50,000 iterations.

---

## 5. Tag Rendering

Converting a codeword into a visual tag image.

### 5.1 Rendering Algorithm

```
render(code, layout) → pixel_grid[grid_size][grid_size]:

  For each quadrant (0..3):
    Scan cells in the quadrant's enumeration order:
      For each cell:
        match cell_type:
          DATA  → extract MSB of code; 1 = white, 0 = black; shift code left
          BLACK → black
          WHITE → white
          IGNORED → transparent

  If grid_size is odd:
    Render center cell similarly

  Apply one final 90° rotation to correct orientation
```

Bits are consumed MSB-first in the same quadrant-scan order used to build
`bit_x`/`bit_y`.

### 5.2 Output Formats

The reference implementation supports:

| Format | Description |
|--------|-------------|
| Individual PNGs | One file per tag: `<family>_<id:05>.png` |
| Mosaic PNG | All tags tiled in a grid |
| PostScript | Vector output, one tag per page, 1-inch size |

Each pixel is one grid cell. Upscaling and adding a quiet zone (white margin)
is left to the consumer.

### 5.3 Color Mapping

| Bit / Cell | Pixel Value |
|------------|-------------|
| Data bit = 0 | Black (`0x000000`) |
| Data bit = 1 | White (`0xFFFFFF`) |
| Black border | Black (`0x000000`) |
| White border | White (`0xFFFFFF`) |
| Ignored | Transparent (`0x00000000`) |

---

## 6. Quality Metrics

### 6.1 False Positive Rate

For a family with `ncodes` codes of `nbits` bits, the probability of a random
bit pattern matching *some* code within `t` bit corrections:

```
FPR(t) = ncodes × Σ(i=0..t) C(nbits, i) / 2^nbits
```

Lower `nbits` and higher `ncodes` increase false-positive risk. Higher minimum
Hamming distance allows more bits to be corrected while keeping FPR low.

### 6.2 Hamming Distance Distribution

After generation, the pairwise Hamming distance distribution (including
rotational variants) should show all distances ≥ `h`. The distribution is
typically concentrated near `h` with a long tail toward `nbits`.

### 6.3 Complexity Energy

```
energy_ratio = edge_transitions / (2 × data_cell_count)
Minimum accepted: 1/3
```

This prevents degenerate codes that would be hard to distinguish from noise or
solid regions.
