# 015 — UnionFind data structure

## Goal
Implement a flat-array union-find with path halving and weighted union for connected component labeling.

## Preconditions
- 001 complete: `apriltag-detect` crate compiles

## Postconditions
- `find(x) == find(y)` after `union(x, y)`
- `find(x) == x` for fresh (unset) elements after `init(x)`
- Path halving reduces tree depth (verifiable via parent chain length)
- Weighted union: smaller tree goes under larger → `get_set_size(find(x)) == merged_size`
- `reset(n)` reuses allocation: capacity ≥ n after reset, parent all 0xFFFFFFFF
- 10,000 random unions: all in same component have same representative

## Description
Create `apriltag-detect/src/unionfind.rs`:

```rust
const UNSET: u32 = 0xFFFF_FFFF;

pub struct UnionFind {
    parent: Vec<u32>,
    size: Vec<u32>,    // size excluding root; root's size = tree size - 1
}
```

Operations:
- **`new(capacity)`** — allocate with all parent = UNSET, size = 0
- **`init(id)`** — `parent[id] = id, size[id] = 0` (make id its own root)
- **`find(id)`** — path halving: `while parent[id] != id { parent[id] = parent[parent[id]]; id = parent[id]; }`
- **`union(a, b)`** — find roots, merge smaller under larger, update sizes
- **`get_set_size(id)`** — `size[find(id)] + 1`
- **`reset(n)`** — resize if needed, fill parent with UNSET, size with 0

**Two flat arrays** instead of struct-of-arrays provides cache-friendly access since `find()` only touches `parent[]` and `union()` touches both.

## References
- `docs/detection-spec.md` §5 — union-find: "parent initialized to 0xFFFFFFFF, size initialized to 0"
- `docs/detection-spec.md` §5 — path halving (not full compression), union by size
