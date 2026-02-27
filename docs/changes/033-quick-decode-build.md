# 033 — QuickDecode table construction

## Goal
Build the chunk-based lookup table from a tag family's codes for fast code matching.

## Preconditions
- 001 complete: `apriltag-detect` crate
- `TagFamily` from `apriltag` crate: `codes`, `bit_locations`, `layout.nbits`
- `hamming::rotate90` from `apriltag` crate

## Postconditions
- Table built for tag16h5 (nbits=16, 30 codes): `chunk_size = ceil(16/4) = 4`, `capacity = 2^4 = 16`
- All 30 × 4 rotations of all codes are indexed
- `chunk_offsets[k]` are valid prefix sums (monotonically non-decreasing)
- `chunk_ids[k]` contains valid indices into the codes array
- max_hamming stored for lookup filtering

## Description
Create initial version of `apriltag-detect/src/decode.rs`:

```rust
pub struct QuickDecode {
    pub nbits: u32,
    pub chunk_size: u32,
    pub capacity: u32,     // 2^chunk_size
    pub chunk_mask: u32,   // capacity - 1
    pub shifts: [u32; 4],
    pub chunk_offsets: [Vec<u16>; 4],  // prefix sums
    pub chunk_ids: [Vec<u16>; 4],     // code indices
    pub max_hamming: u32,
}

impl QuickDecode {
    pub fn build(family: &TagFamily, max_hamming: u32) -> QuickDecode
}
```

Algorithm:
1. `chunk_size = ceil(nbits / 4)`, `capacity = 1 << chunk_size`
2. `shifts = [0, chunk_size, 2*chunk_size, 3*chunk_size]`
3. For each code × 4 rotations: extract each chunk value, count occurrences per chunk per value
4. Prefix sum → `chunk_offsets[k][v]` = start index for value v in chunk k
5. Second pass: fill `chunk_ids[k]` with code indices at correct offsets

## References
- `docs/detection-spec.md` §10.5 — "QuickDecode: split code into 4 chunks, build prefix-sum index per chunk"
- `apriltag/src/hamming.rs:6` — `rotate90(code, nbits)` for generating all 4 rotations
