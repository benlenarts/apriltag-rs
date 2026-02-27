# 034 — QuickDecode lookup with rotation

## Goal
Match a decoded code word against the family using the chunk-based lookup table, trying all 4 rotations.

## Preconditions
- 033 complete: `QuickDecode` table built
- `hamming::rotate90` and `hamming::hamming_distance` from `apriltag` crate

## Postconditions
- Exact match (code in family, rotation 0) → `hamming=0, rotation=0, id=correct_index`
- 1-bit error → `hamming=1`
- Rotated code → correct rotation detected (0-3)
- No match within max_hamming → returns None
- Best match = minimum Hamming distance; ties broken by lowest rotation
- Performance: O(candidate_count) per query, where candidate_count is typically small

## Description
Add to `decode.rs`:

```rust
pub struct DecodeResult {
    pub id: u32,         // index into family codes
    pub hamming: u32,    // number of corrected bits
    pub rotation: u32,   // 0-3 rotation that best matches
}

impl QuickDecode {
    pub fn decode(&self, rcode: u64) -> Option<DecodeResult>
}
```

Algorithm:
```
best = None
for rotation in 0..4:
    code = rotate90^rotation(rcode, nbits)
    for chunk_k in 0..4:
        chunk_val = (code >> shifts[k]) & chunk_mask
        for idx in chunk_offsets[k][chunk_val] .. chunk_offsets[k][chunk_val+1]:
            candidate_id = chunk_ids[k][idx]
            dist = hamming_distance(code, family.codes[candidate_id])
            if dist <= max_hamming && (best.is_none() || dist < best.hamming):
                best = Some(DecodeResult { id: candidate_id, hamming: dist, rotation })
```

Note: all 4 chunks are checked for each rotation — a code matches if it appears in ANY chunk's lookup (the table is a union of 4 independent indices for redundancy).

## References
- `docs/detection-spec.md` §10.5 — "for each rotation: for each chunk: lookup candidates, check Hamming"
- `apriltag/src/hamming.rs:6` — `rotate90(code, nbits)`
- `apriltag/src/hamming.rs:21` — `hamming_distance(a, b)`
