# 039 — Code extraction and decision margin

## Goal
Extract the binary code word from sampled bit values and compute the decision margin (confidence metric).

## Preconditions
- 038 complete: sharpened bit values available

## Postconditions
- Known bit pattern → correct 64-bit code word extracted
- Bits read MSB-first matching `BitLocation` ordering (bit 0 = MSB)
- `decision_margin = 100 * min(white_score/white_count, black_score/black_count)`
- Laplace smoothing (+1) on white_score and black_score to avoid division by zero
- Negative decision margin → quad rejected (unreliable decoding)

## Description
Add to `decode.rs`:

```rust
pub struct DecodeCandidate {
    pub rcode: u64,
    pub decision_margin: f32,
}

pub fn extract_code(values: &[f64], nbits: u32) -> DecodeCandidate
```

Algorithm:
```
rcode = 0
white_score = 0.0
white_count = 0.0 + 1.0   // Laplace smoothing
black_score = 0.0
black_count = 0.0 + 1.0   // Laplace smoothing

for i in 0..nbits:
    if values[i] > 0:
        rcode |= 1 << (nbits - 1 - i)   // MSB-first
        white_score += values[i]
        white_count += 1
    else:
        black_score -= values[i]   // make positive
        black_count += 1

decision_margin = 100.0 * min(white_score / white_count, black_score / black_count)
```

The decision margin measures the average confidence per bit. Higher is better. Typical good detections have margins > 50.

## References
- `docs/detection-spec.md` §10.4 — "MSB-first, rcode |= (v>0) << (nbits-1-i), decision_margin with Laplace smoothing"
