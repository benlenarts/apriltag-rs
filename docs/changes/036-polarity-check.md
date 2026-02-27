# 036 — Polarity check

## Goal
Verify that the gray model polarity (white > black at center) matches the quad's border orientation.

## Preconditions
- 035 complete: white and black GrayModels solved

## Postconditions
- Normal tag (white outside, black inside): `white_model.interpolate(0,0) > black_model.interpolate(0,0)`
- Reversed tag: `white_model.interpolate(0,0) < black_model.interpolate(0,0)`
- Polarity mismatch with quad's `reversed_border` → quad rejected
- If no family has the opposite orientation → quad fully rejected

## Description
Add to `decode.rs`:

```rust
pub fn check_polarity(
    white_model: &GrayModel,
    black_model: &GrayModel,
    reversed_border: bool,
) -> bool  // true = polarity OK
```

1. `white_at_center = white_model.interpolate(0.0, 0.0)`
2. `black_at_center = black_model.interpolate(0.0, 0.0)`
3. `expected = !reversed_border` → white should be brighter; `reversed_border` → black should be brighter
4. If `reversed_border`: check `black_at_center > white_at_center`
5. If `!reversed_border`: check `white_at_center > black_at_center`
6. Return false if mismatch

This is a cheap check that catches illumination inversion or incorrectly oriented quads before the expensive bit-sampling step.

## References
- `docs/detection-spec.md` §10.3 — "verify polarity: white model > black model at center for normal orientation"
