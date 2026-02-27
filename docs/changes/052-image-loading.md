# 052 — Image loading (PNG + JPEG → grayscale)

## Goal
Load PNG and JPEG images from disk and convert to grayscale `ImageU8` for detection.

## Preconditions
- 051 complete: CLI args parsed, image paths available

## Postconditions
- PNG file → loads correctly, dimensions match file header
- JPEG file → loads correctly
- Color images auto-converted to grayscale (luma)
- Already-grayscale images pass through unchanged
- Invalid/missing file → clear error message (not panic)
- Unsupported format → error listing supported formats

## Description
Add to `apriltag-detect-cli/src/main.rs`:

```rust
use image::io::Reader as ImageReader;

fn load_image(path: &Path) -> anyhow::Result<ImageU8> {
    let img = ImageReader::open(path)
        .with_context(|| format!("failed to open {}", path.display()))?
        .decode()
        .with_context(|| format!("failed to decode {}", path.display()))?
        .into_luma8();

    let (width, height) = img.dimensions();
    let mut result = ImageU8::zeros(width, height);
    for y in 0..height {
        for x in 0..width {
            result.set(x, y, img.get_pixel(x, y).0[0]);
        }
    }
    Ok(result)
}
```

The `image` crate handles format detection, decompression, and color space conversion. We only enable `png` and `jpeg` features to minimize binary size.

## References
- Architecture decision — CLI uses `image` crate for I/O
- `image` crate documentation
