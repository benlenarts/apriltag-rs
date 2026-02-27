# 029 — Border orientation check

## Goal
Determine whether a quad has normal (white-outside-black-inside) or reversed border orientation, and reject quads that don't match any registered family.

## Preconditions
- 017 complete: edge points have gradient directions (gx, gy)

## Postconditions
- White-outside, black-inside quad → `reversed_border = false`
- Black-outside, white-inside quad → `reversed_border = true`
- If no registered family has matching `reversed_border` → quad rejected early
- Gradient dot product determines orientation: `Σ(dx·gx + dy·gy)`, where (dx,dy) = point - centroid

## Description
Add to `quad.rs`:

```rust
pub fn check_border_orientation(
    points: &[EdgePoint],
    family_reversed: &[bool],  // which orientations are registered
) -> Option<bool>  // None = rejected, Some(reversed)
```

1. Compute centroid of edge points: `cx = mean(x), cy = mean(y)`
2. For each point: `dot += (x - cx) * gx + (y - cy) * gy`
3. `reversed = dot < 0`
4. If no family has `layout.reversed_border == reversed`: return None (early reject)
5. Return `Some(reversed)`

This is a cheap check that avoids running the expensive homography+decode pipeline on quads that can't possibly match any registered family.

## References
- `docs/detection-spec.md` §7.2 — "dot product of gradient with outward direction determines border orientation"
