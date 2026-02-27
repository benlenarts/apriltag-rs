# 040 — Separating axis theorem for quad overlap

## Goal
Test whether two 4-point convex polygons (quads) overlap using the separating axis theorem.

## Preconditions
- 002 complete: `Quad` type with `corners: [[f32; 2]; 4]`

## Postconditions
- Two overlapping quads → returns true
- Two disjoint quads → returns false
- Quads touching at a single edge or point → returns false (not overlapping)
- Tests all 8 candidate axes (4 edge normals per quad)

## Description
Create `apriltag-detect/src/dedup.rs`:

```rust
pub fn quads_overlap(a: &[[f64; 2]; 4], b: &[[f64; 2]; 4]) -> bool
```

Separating Axis Theorem (SAT):
1. For each edge of quad A and quad B (8 edges total):
   - Compute edge normal: `n = (-(y1-y0), x1-x0)`
   - Project all 4 vertices of A onto normal → get `[min_a, max_a]`
   - Project all 4 vertices of B onto normal → get `[min_b, max_b]`
   - If `max_a < min_b || max_b < min_a`: → separated on this axis → return false
2. If no separating axis found → quads overlap → return true

This is an exact test for convex polygons.

## References
- `docs/detection-spec.md` §11 — "deduplication: separating axis theorem on quad corners"
