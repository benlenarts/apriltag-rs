# 028 — Quad validation (area, angles, convexity)

## Goal
Reject invalid quads before the expensive decoding stage.

## Preconditions
- 027 complete: 4 corner positions computed

## Postconditions
- Area < 0.95 × (tag_width)² → rejected
- Non-convex polygon → rejected
- Winding order must be CCW; CW → reverse corners to make CCW
- Interior angles violating `cos_critical_rad` → rejected
- Valid square passes all checks

## Description
Add to `quad.rs`:

```rust
pub fn validate_quad(
    corners: &[[f32; 2]; 4],
    min_tag_width: f32,
    cos_critical_rad: f32,
) -> Option<Quad>
```

Checks:
1. **Area**: Compute via cross product: `area = 0.5 * |Σ (p_i × p_{i+1})|`. If `area < 0.95 * min_tag_width²`: reject.
2. **Winding**: Cross product of edge vectors at each corner. If CW (negative cross products), reverse corner order.
3. **Convexity**: All cross products must have the same sign. If not: reject.
4. **Interior angles**: For each corner, check angle between incoming/outgoing edges: `cos(angle) ≤ cos_critical_rad` → too sharp → reject.

`min_tag_width = smallest_family_total_width / quad_decimate`, clamped to minimum 3.

## References
- `docs/detection-spec.md` §7.9 — "validate area ≥ 0.95 × tag_width², check convexity and angles"
