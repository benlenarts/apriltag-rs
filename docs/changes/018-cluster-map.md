# 018 — Custom flat hash map for clusters

## Goal
Group edge points by cluster ID using an open-addressed hash map with flat storage for cache-friendly iteration.

## Preconditions
- 017 complete: edge points with cluster IDs extracted

## Postconditions
- All edge points for the same cluster ID are grouped together
- Iteration order is deterministic (sorted by hash slot, then by insertion order within slot)
- Lookup is O(1) amortized
- No per-cluster heap allocation (all points stored in one flat Vec)
- Hash function matches reference: Knuth multiplicative `(2654435761 * id) >> shift`

## Description
Add to `cluster.rs`:

```rust
pub struct ClusterMap {
    /// Flat storage of all edge points, grouped by cluster
    points: Vec<EdgePoint>,
    /// Index entries: (cluster_id, start, len) in points array
    entries: Vec<ClusterEntry>,
    /// Hash table: slot → index into entries (or EMPTY)
    table: Vec<u32>,
    table_mask: u32,
}

struct ClusterEntry {
    cluster_id: u64,
    start: u32,     // offset into points
    count: u32,     // number of points
}
```

**Two-pass construction:**
1. **Pass 1**: Count points per cluster using the hash table (open addressing, linear probing). Record each cluster's count.
2. **Prefix sum**: Compute start offsets from counts.
3. **Pass 2**: Place each edge point at its cluster's offset (using running cursor).

This avoids per-cluster `Vec` allocations. The entire point set lives in one contiguous allocation.

**Hash function**: `slot = ((id as u32).wrapping_mul(2654435761)) >> (32 - table_bits)`. Load factor ≤ 0.5 (table capacity = 2× cluster count, rounded to power of 2).

## References
- `docs/detection-spec.md` §6.2 — hash clustering: "Knuth multiplicative hash, open addressing"
- `docs/detection-spec.md` §6.3 — deterministic iteration: "sorted by hash slot"
