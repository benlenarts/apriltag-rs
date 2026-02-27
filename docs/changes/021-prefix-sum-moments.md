# 021 — Cumulative line-fit moments (prefix sums)

## Goal
Build a prefix sum array of weighted moments for O(1) line fitting on any contiguous range of sorted edge points.

## Preconditions
- 019 complete: sorted edge points available
- 002 complete: `LineFitPoint` type defined

## Postconditions
- `lfps[i].W` = sum of weights for points 0..=i
- `lfps[sz].W - lfps[0].W` = total weight of all points
- Range query via subtraction: `lfps[j] - lfps[i]` gives moments for points i..j
- Weight per point = gradient magnitude: `sqrt(gx² + gy²) + 1`
- Moments: Mx = Σ(W·x), My = Σ(W·y), Mxx = Σ(W·x²), Mxy = Σ(W·xy), Myy = Σ(W·y²)

## Description
Add to `quad.rs`:

```rust
pub fn compute_line_fit_points(
    points: &[EdgePoint],
    lfps: &mut Vec<LineFitPoint>,
)
```

Build prefix sums:
```
lfps[0] = zero
for i in 0..sz:
    w = sqrt(points[i].gx² + points[i].gy²) as f64 + 1.0
    x = points[i].x as f64 / 2.0   // convert from fixed-point
    y = points[i].y as f64 / 2.0
    lfps[i+1].W   = lfps[i].W  + w
    lfps[i+1].Mx  = lfps[i].Mx + w * x
    lfps[i+1].My  = lfps[i].My + w * y
    lfps[i+1].Mxx = lfps[i].Mxx + w * x * x
    lfps[i+1].Mxy = lfps[i].Mxy + w * x * y
    lfps[i+1].Myy = lfps[i].Myy + w * y * y
```

This enables O(1) line fitting for any segment [i, j]: just subtract `lfps[j+1] - lfps[i]`.

**Critical for performance**: The exhaustive 4-corner search (change 026) calls line fitting O(C(n,4) × 4) times. Without prefix sums this would be O(n) per call; with them it's O(1).

## References
- `docs/detection-spec.md` §7.4 — "cumulative moments, prefix sum, weight = gradient magnitude"
