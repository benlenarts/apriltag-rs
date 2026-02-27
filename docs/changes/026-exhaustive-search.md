# 026 — Exhaustive 4-combination search

## Goal
Find the best 4 corner indices from candidate maxima by trying all C(n,4) combinations and selecting the one with minimum total line-fit error.

## Preconditions
- 022 complete: `fit_line()` available
- 025 complete: candidate maxima indices available

## Postconditions
- Best combination has minimum total MSE across 4 segments
- Each segment MSE ≤ `max_line_fit_mse` (default 10.0)
- Adjacent line directions satisfy `cos_critical_rad` angle constraint
- Returns None if no valid combination found
- Returns `[m0, m1, m2, m3]` indices (in order) and 4 fitted lines

## Description
Add to `quad.rs`:

```rust
pub fn find_best_quad(
    lfps: &[LineFitPoint],
    maxima: &[usize],
    sz: usize,
    max_line_fit_mse: f32,
    cos_critical_rad: f32,
) -> Option<([usize; 4], [[f64; 4]; 4])>  // (corner indices, lines)
```

Algorithm:
```
best_err = f64::MAX
for m0 in 0..n:
  for m1 in (m0+1)..n:
    for m2 in (m1+1)..n:
      for m3 in (m2+1)..n:
        lines[0] = fit_line(lfps, maxima[m0], maxima[m1], sz)
        lines[1] = fit_line(lfps, maxima[m1], maxima[m2], sz)
        lines[2] = fit_line(lfps, maxima[m2], maxima[m3], sz)
        lines[3] = fit_line(lfps, maxima[m3], maxima[m0], sz)
        if any line MSE > max_line_fit_mse: continue
        if any adjacent line angle violates cos_critical_rad: continue
        total_err = sum of 4 MSEs
        if total_err < best_err: update best
```

**Angle check**: `cos(angle) = dot(normal_i, normal_{i+1}) ≤ cos_critical_rad` → lines are too parallel → skip.

With max_nmaxima=10: C(10,4) = 210 combinations × 4 line fits each = 840 line fits. With O(1) prefix-sum line fitting, this is fast.

## References
- `docs/detection-spec.md` §7.7 — "exhaustive C(n,4) search, check MSE and angle constraints"
