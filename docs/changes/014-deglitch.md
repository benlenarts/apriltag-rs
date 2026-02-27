# 014 — Deglitch (morphological close)

## Goal
Optional morphological close (dilate→erode) to remove single-pixel noise in the ternary threshold image.

## Preconditions
- 013 complete: ternary threshold image available

## Postconditions
- Single-pixel hole (0) in white (255) region → filled to 255
- Single-pixel white (255) island in black (0) region → removed to 0
- Unknown pixels (127) are unaffected
- Disabled by default (`deglitch = false`)
- Operation is idempotent on clean regions

## Description
Add to `threshold.rs`:

```rust
pub fn deglitch(img: &mut ImageU8)
```

Two-pass morphological close on binary channels (0 and 255 only, skip 127):
1. **Dilate**: for each pixel, output = max of 3×3 neighborhood (among 0/255 values only)
2. **Erode**: for each pixel of dilated result, output = min of 3×3 neighborhood

This fills single-pixel gaps and removes single-pixel islands. The 3×3 structuring element is the standard morphological cross or square.

Only called when `QuadThreshParams::deglitch == true`.

## References
- `docs/detection-spec.md` §4.4 — deglitch: "morphological close (dilate then erode)"
