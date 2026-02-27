# 041 — Detection deduplication with preference ordering

## Goal
Remove duplicate detections of the same tag, keeping the best-quality detection.

## Preconditions
- 040 complete: `quads_overlap()` available

## Postconditions
- Two overlapping detections with same family+ID → only one survives
- Preference: lower Hamming distance > higher decision margin > lexicographic corner comparison
- Non-overlapping detections of same ID → both kept (different physical tags)
- Different family+ID pairs → never deduplicated

## Description
Add to `dedup.rs`:

```rust
pub fn deduplicate(detections: &mut Vec<Detection>)
```

Algorithm:
1. Group detections by `(family, id)`
2. For each group with > 1 detection:
   - For each pair (i, j): check `quads_overlap(det[i].corners, det[j].corners)`
   - If overlapping: mark the inferior one for removal
   - Preference ordering (keep the one with):
     a. Lower Hamming distance (fewer corrected bits = more confident)
     b. Higher decision margin (if Hamming equal)
     c. Lexicographic comparison of corner coordinates (tiebreaker)
3. Remove all marked detections

**Note**: O(n²) within each group, but groups are almost always size 1-2 in practice.

## References
- `docs/detection-spec.md` §11 — "prefer: lower hamming > higher margin > lexicographic corners"
