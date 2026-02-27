# 025 — Local maxima detection

## Goal
Find candidate corner positions as local maxima in the smoothed error array.

## Preconditions
- 024 complete: smoothed error array available

## Postconditions
- Perfect square cluster → exactly 4 maxima
- `errors[i] > errors[(i-1) % sz] && errors[i] > errors[(i+1) % sz]` for each maximum
- If count > `max_nmaxima` (default 10): keep only top N by magnitude
- Maxima indices returned in sorted order
- Minimum 4 maxima required; fewer → cluster rejected (no quad possible)

## Description
Add to `quad.rs`:

```rust
pub fn find_maxima(
    errors: &[f64],
    max_nmaxima: usize,
) -> Option<Vec<usize>>  // None if < 4 maxima
```

1. Scan errors circularly: `errors[i] > errors[prev] && errors[i] > errors[next]` → candidate
2. Collect all candidates
3. If count < 4: return None (can't form a quad)
4. If count > max_nmaxima: sort candidates by error value (descending), keep top max_nmaxima, re-sort by index
5. Return indices

## References
- `docs/detection-spec.md` §7.6 — "local maxima: err[i] > err[i-1] and err[i] > err[i+1], capped at max_nmaxima"
