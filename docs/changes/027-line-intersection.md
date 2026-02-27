# 027 — Line intersection for corner computation

## Goal
Compute quad corners by intersecting adjacent fitted lines (2×2 linear system solve).

## Preconditions
- 026 complete: 4 fitted lines available as `[centroid_x, centroid_y, normal_x, normal_y]`

## Postconditions
- Perpendicular lines → exact intersection at expected point
- Near-parallel lines (|det| < 0.001) → rejection (return None)
- 4 corners computed from 4 line pairs: (line0, line1), (line1, line2), (line2, line3), (line3, line0)
- Output corners in CCW order matching the line order

## Description
Add to `quad.rs`:

```rust
pub fn intersect_lines(
    l0: &[f64; 4],  // [px, py, nx, ny]
    l1: &[f64; 4],
) -> Option<[f32; 2]>
```

Given lines defined by `n · (p - centroid) = 0`:
- Line 0: `n0x * (x - p0x) + n0y * (y - p0y) = 0`
- Line 1: `n1x * (x - p1x) + n1y * (y - p1y) = 0`

Solve 2×2 system:
```
| n0x  n0y | | x |   | n0x*p0x + n0y*p0y |
| n1x  n1y | | y | = | n1x*p1x + n1y*p1y |
```

`det = n0x * n1y - n0y * n1x`. If `|det| < 0.001`: return None (parallel lines).

```rust
pub fn compute_corners(lines: &[[f64; 4]; 4]) -> Option<[[f32; 2]; 4]>
```

Intersect (lines[0], lines[1]), (lines[1], lines[2]), (lines[2], lines[3]), (lines[3], lines[0]).

## References
- `docs/detection-spec.md` §7.8 — "intersect adjacent lines, reject if |det| < 0.001"
