# 030 — Size filtering for clusters

## Goal
Early-reject clusters that are too small or too large to be valid tag edges.

## Preconditions
- 018 complete: cluster map with point counts

## Postconditions
- Clusters with < `min_cluster_pixels` points → rejected
- Clusters with < 24 points → rejected (hardcoded minimum from reference)
- Clusters with > `2 × perimeter` points → rejected (rough upper bound)
- Remaining clusters passed to quad fitting

## Description
Add to `quad.rs`:

```rust
pub fn filter_clusters(
    cluster_count: u32,
    min_cluster_pixels: u32,
    max_perimeter: u32,
) -> bool  // true = keep
```

Filter criteria:
1. `count < max(24, min_cluster_pixels)` → reject (too few points for a reliable quad fit)
2. `count > max_perimeter` → reject (too many points, likely a large blob boundary)

`max_perimeter = 2 × (2 × image_width + 2 × image_height)` is a conservative upper bound.

This filter runs before any quad fitting computation, so it's a cheap way to eliminate the vast majority of clusters.

## References
- `docs/detection-spec.md` §7.1 — "reject clusters with < min_cluster_pixels, < 24, or > 2×perimeter"
