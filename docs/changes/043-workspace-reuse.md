# 043 — Multi-frame workspace buffer reuse

## Goal
Verify that the workspace pattern achieves zero allocation on the second and subsequent `detect()` calls.

## Preconditions
- 042 complete: detector with workspace

## Postconditions
- Two `detect()` calls on same-size images → all workspace buffers have same capacity
- `workspace.decimated` reused (same allocation address) across calls
- Same detection results for same input image on consecutive calls

## Description
Add tests to `detector.rs`:

```rust
#[test]
fn workspace_reused_across_frames() {
    let mut detector = Detector::new(DetectorConfig::default());
    detector.add_family(tag36h11(), 2);

    let image = create_test_image_with_tag(200, 200, 0);

    // First call — allocates workspace
    let det1 = detector.detect(&image);

    // Second call — reuses workspace
    let det2 = detector.detect(&image);

    assert_eq!(det1.len(), det2.len());
    // Verify same detection results
    for (d1, d2) in det1.iter().zip(&det2) {
        assert_eq!(d1.id, d2.id);
        assert_eq!(d1.hamming, d2.hamming);
    }
}
```

The workspace `clear()` methods reset data without deallocating:
- `Vec::clear()` keeps capacity
- `UnionFind::reset(n)` reuses arrays
- `ImageU8::resize(w, h)` reuses buffer if capacity suffices

## References
- Architecture decision — workspace pattern for zero steady-state allocation
