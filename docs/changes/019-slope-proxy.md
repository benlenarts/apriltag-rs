# 019 — Fast slope proxy computation

## Goal
Compute an angular sort key for each edge point without `atan2`, using a quadrant-based slope proxy.

## Preconditions
- 002 complete: `EdgePoint` type with slope field

## Postconditions
- Points arranged CCW around a square center produce monotonically increasing slope values
- Slope values span [0, 4) — one unit per quadrant
- Noise offsets prevent zero-division: `cx += 0.05118`, `cy -= 0.028581`
- Points directly on axes sort correctly (no ambiguity)

## Description
Create `apriltag-detect/src/quad.rs` (initial):

```rust
pub fn compute_slopes(points: &mut [EdgePoint]) {
    // compute centroid with small offset to avoid degeneracies
    let cx = mean_x + 0.05118;
    let cy = mean_y - 0.028581;

    for p in points.iter_mut() {
        let dx = p.x as f64 / 2.0 - cx;
        let dy = p.y as f64 / 2.0 - cy;

        p.slope = if dy.abs() > dx.abs() {
            // quadrant 1 or 3 (vertical-ish)
            let base = if dy > 0.0 { 1.0 } else { 3.0 };
            base + dx / dy  // slope ∈ [base-1, base+1]
        } else {
            // quadrant 0 or 2 (horizontal-ish)
            let base = if dx > 0.0 { 0.0 } else { 2.0 };
            base - dy / dx
        } as f32;
    }
}
```

The slope proxy maps the full 360° range to [0, 4) monotonically (CCW). This avoids the cost of `atan2` while maintaining correct angular ordering. The irrational noise offsets ensure no point falls exactly on center.

## References
- `docs/detection-spec.md` §7.3 — angular sorting: "quadrant + |dy|/|dx| proxy, noise offsets 0.05118 and −0.028581"
