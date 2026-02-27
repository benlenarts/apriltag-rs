# 020 — Merge sort with small-size sorting networks

## Goal
Sort edge points by slope using merge sort with inline comparison networks for small sizes (≤5 elements).

## Preconditions
- 019 complete: edge points have computed slope values

## Postconditions
- Sorted output matches `slice.sort_by(|a, b| a.slope.total_cmp(&b.slope))` for random inputs
- Sizes 0, 1 → no-op
- Sizes 2-5 → inline sorting network (optimal compare-swap sequences)
- Sizes > 5 → merge sort with recursive halving
- Stable sort (equal elements preserve original order)

## Description
Add to `quad.rs`:

```rust
pub fn sort_by_slope(points: &mut [EdgePoint], scratch: &mut Vec<EdgePoint>)
```

Dispatch by size:
- **0-1**: return
- **2**: one compare-swap
- **3**: 3 compare-swaps (optimal network)
- **4**: 5 compare-swaps (optimal network)
- **5**: 9 compare-swaps (optimal network)
- **≥6**: bottom-up merge sort using scratch buffer

The small-size sorting networks avoid recursion overhead for the common case (many clusters have 24-200 points, but the recursive base case hits these sizes frequently).

The scratch buffer is passed as a workspace argument for reuse across clusters.

## References
- `docs/detection-spec.md` §7.3 — "merge sort with sorting networks for ≤5 elements"
- Reference C implementation uses the same approach in quad detection
