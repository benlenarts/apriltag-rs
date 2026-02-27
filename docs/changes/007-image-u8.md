# 007 — ImageU8 type with stride-aligned storage

## Goal
Implement the core image buffer type with cache-line-aligned stride for optimal memory access patterns.

## Preconditions
- 001 complete: `apriltag-detect` crate compiles

## Postconditions
- `ImageU8::new(100, 100).stride` ≥ 100 and is a multiple of 64
- `ImageU8::zeros(w, h)` creates an all-zero image of correct dimensions
- `get(x, y)` / `set(x, y, v)` round-trip correctly for all valid coordinates
- Out-of-bounds access panics in debug builds
- `row(y)` returns a slice of exactly `width` bytes
- `resize(w, h)` reuses existing allocation when capacity suffices
- `ImageRef` (borrowed variant) works with external `&[u8]` buffers

## Description
Create `apriltag-detect/src/image.rs`:

```rust
pub struct ImageU8 {
    pub width: u32,
    pub height: u32,
    pub stride: u32,
    buf: Vec<u8>,
}

pub struct ImageRef<'a> {
    pub width: u32,
    pub height: u32,
    pub stride: u32,
    buf: &'a [u8],
}
```

- **Stride alignment**: `stride = (width as u32 + 63) & !63` — 64-byte cache line alignment
- **`new(width, height)`**: allocates `stride * height` bytes, uninitialized
- **`zeros(width, height)`**: allocates and fills with 0
- **`get(x, y)`**: `buf[y as usize * stride as usize + x as usize]`
- **`set(x, y, v)`**: same index, mutable
- **`row(y)`**: `&buf[y*stride .. y*stride + width]`
- **`resize(w, h)`**: recompute stride, reuse `Vec` capacity if possible
- **`ImageRef::from_buffer(data, width, height, stride)`**: borrow external data

The 64-byte alignment ensures each image row starts on a cache line boundary, eliminating false sharing in parallel code and enabling future SIMD operations.

## References
- `docs/detection-spec.md` §1.1 — `image_u8: width, height, stride, buf`
- `docs/detection-spec.md` §1.1 — constraint: width, height < 32768
