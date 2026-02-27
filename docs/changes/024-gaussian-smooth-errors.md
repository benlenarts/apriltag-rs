# 024 — Gaussian smoothing of error array

## Goal
Apply 1D Gaussian low-pass filter to the circular error array to suppress noise before local maxima detection.

## Preconditions
- 023 complete: per-point error array computed

## Postconditions
- Smoothed errors are non-negative
- Sharp single-point spikes are attenuated
- σ=1, kernel truncated where weight < 0.05
- Smoothing wraps around (circular boundary)
- Output length = input length

## Description
Add to `quad.rs`:

```rust
pub fn smooth_errors(errors: &mut [f64], sigma: f64)
```

1. Compute Gaussian kernel with σ=1: weights at offsets 0, ±1, ±2 (truncate where `exp(-0.5 * (d/σ)²) < 0.05` → d > 2 for σ=1)
2. Normalize kernel weights to sum to 1.0
3. For each index i: `smoothed[i] = Σ_k kernel[k] * errors[(i + k) % sz]`
4. Write smoothed values back to errors array

Use a temporary buffer for the smoothed output (or two-pass with modular arithmetic).

This smoothing prevents false corner detections from individual noisy error values.

## References
- `docs/detection-spec.md` §7.6 — "Gaussian smoothing with σ=1, cutoff 0.05"
