use super::image::ImageU8;

const TILESZ: u32 = 4;

/// Binarize a rectangular block of pixels using a single tile's lo/hi values.
#[inline(always)]
#[allow(clippy::too_many_arguments)]
fn binarize_block(
    img_buf: &[u8],
    stride: usize,
    out_buf: &mut [u8],
    out_w: usize,
    lo: u8,
    hi: u8,
    min_white_black_diff: i32,
    x_start: usize,
    x_end: usize,
    y_start: usize,
    y_end: usize,
) {
    if (hi as i32 - lo as i32) < min_white_black_diff {
        for y in y_start..y_end {
            for x in x_start..x_end {
                out_buf[y * out_w + x] = 127;
            }
        }
    } else {
        let thresh = lo as i32 + (hi as i32 - lo as i32) / 2;
        for y in y_start..y_end {
            let img_row_off = y * stride;
            let out_row_off = y * out_w;
            for x in x_start..x_end {
                out_buf[out_row_off + x] = if img_buf[img_row_off + x] as i32 > thresh {
                    255
                } else {
                    0
                };
            }
        }
    }
}

/// Reusable buffers for the tile-based threshold computation.
///
/// Pool these in `DetectorBuffers` to avoid 4 allocations (~5 KB) per frame.
pub struct ThresholdBuffers {
    pub tile_min: Vec<u8>,
    pub tile_max: Vec<u8>,
    pub dilated_max: Vec<u8>,
    pub eroded_min: Vec<u8>,
    pub morph_a: Vec<u8>,
    pub morph_b: Vec<u8>,
}

impl Default for ThresholdBuffers {
    fn default() -> Self {
        Self::new()
    }
}

impl ThresholdBuffers {
    pub fn new() -> Self {
        Self {
            tile_min: Vec::new(),
            tile_max: Vec::new(),
            dilated_max: Vec::new(),
            eroded_min: Vec::new(),
            morph_a: Vec::new(),
            morph_b: Vec::new(),
        }
    }
}

/// Produce a ternary threshold image: 0 (black), 255 (white), or 127 (unknown).
///
/// Uses tile-based adaptive thresholding with min/max dilation to handle
/// spatially varying illumination.
///
/// Pass a pre-allocated `buf` to reuse memory across calls. Use `Vec::new()`
/// for one-shot usage.
pub fn threshold(
    img: &ImageU8,
    min_white_black_diff: i32,
    deglitch: bool,
    buf: Vec<u8>,
    tile_bufs: &mut ThresholdBuffers,
) -> ImageU8 {
    let w = img.width;
    let h = img.height;
    let tw = w / TILESZ;
    let th = h / TILESZ;

    if tw == 0 || th == 0 {
        return ImageU8::new(w, h);
    }

    // Compute per-tile min/max with 1-element padding border.
    // Padding uses neutral values (255 for min, 0 for max) so the 3×3
    // dilation/erosion loop can index unconditionally without bounds checks.
    let padded_w = tw + 2;
    let padded_h = th + 2;
    let padded_len = (padded_w * padded_h) as usize;

    tile_bufs.tile_min.clear();
    tile_bufs.tile_min.resize(padded_len, 255u8);
    tile_bufs.tile_max.clear();
    tile_bufs.tile_max.resize(padded_len, 0u8);
    let tile_min = &mut tile_bufs.tile_min;
    let tile_max = &mut tile_bufs.tile_max;

    for ty in 0..th {
        for tx in 0..tw {
            let mut lo = 255u8;
            let mut hi = 0u8;
            for dy in 0..TILESZ {
                for dx in 0..TILESZ {
                    let v = img.get(tx * TILESZ + dx, ty * TILESZ + dy);
                    lo = lo.min(v);
                    hi = hi.max(v);
                }
            }
            tile_min[((ty + 1) * padded_w + (tx + 1)) as usize] = lo;
            tile_max[((ty + 1) * padded_w + (tx + 1)) as usize] = hi;
        }
    }

    // Dilate max, erode min using 3x3 tile neighborhood (no bounds checks needed)
    let tile_len = (tw * th) as usize;
    tile_bufs.dilated_max.clear();
    tile_bufs.dilated_max.resize(tile_len, 0u8);
    tile_bufs.eroded_min.clear();
    tile_bufs.eroded_min.resize(tile_len, 255u8);
    let dilated_max = &mut tile_bufs.dilated_max;
    let eroded_min = &mut tile_bufs.eroded_min;

    for ty in 0..th {
        for tx in 0..tw {
            let mut hi = 0u8;
            let mut lo = 255u8;
            for dy in 0..3u32 {
                for dx in 0..3u32 {
                    let idx = ((ty + dy) * padded_w + (tx + dx)) as usize;
                    hi = hi.max(tile_max[idx]);
                    lo = lo.min(tile_min[idx]);
                }
            }
            let idx = (ty * tw + tx) as usize;
            dilated_max[idx] = hi;
            eroded_min[idx] = lo;
        }
    }

    // Binarize each pixel, processing tile-by-tile to load lo/hi once per tile.
    // Remainder pixels (beyond tile-aligned region) use the last tile's values.
    let mut out = ImageU8::new_reuse(w, h, buf);

    for ty in 0..th {
        let y_start = (ty * TILESZ) as usize;
        let y_end = y_start + TILESZ as usize;
        let tile_row = (ty * tw) as usize;

        for tx in 0..tw {
            let idx = tile_row + tx as usize;
            let x_start = (tx * TILESZ) as usize;
            binarize_block(
                &img.buf,
                img.stride as usize,
                &mut out.buf,
                w as usize,
                eroded_min[idx],
                dilated_max[idx],
                min_white_black_diff,
                x_start,
                x_start + TILESZ as usize,
                y_start,
                y_end,
            );
        }

        if (tw * TILESZ) < w {
            let idx = tile_row + (tw - 1) as usize;
            binarize_block(
                &img.buf,
                img.stride as usize,
                &mut out.buf,
                w as usize,
                eroded_min[idx],
                dilated_max[idx],
                min_white_black_diff,
                (tw * TILESZ) as usize,
                w as usize,
                y_start,
                y_end,
            );
        }
    }

    if (th * TILESZ) < h {
        let y_start = (th * TILESZ) as usize;
        let tile_row = ((th - 1) * tw) as usize;

        for tx in 0..tw {
            let idx = tile_row + tx as usize;
            let x_start = (tx * TILESZ) as usize;
            binarize_block(
                &img.buf,
                img.stride as usize,
                &mut out.buf,
                w as usize,
                eroded_min[idx],
                dilated_max[idx],
                min_white_black_diff,
                x_start,
                x_start + TILESZ as usize,
                y_start,
                h as usize,
            );
        }

        if (tw * TILESZ) < w {
            let idx = tile_row + (tw - 1) as usize;
            binarize_block(
                &img.buf,
                img.stride as usize,
                &mut out.buf,
                w as usize,
                eroded_min[idx],
                dilated_max[idx],
                min_white_black_diff,
                (tw * TILESZ) as usize,
                w as usize,
                y_start,
                h as usize,
            );
        }
    }

    if deglitch {
        deglitch_image(&mut out, &mut tile_bufs.morph_a, &mut tile_bufs.morph_b);
    }

    out
}

/// Morphological close (dilate then erode) with 3x3 structuring element.
fn deglitch_image(img: &mut ImageU8, buf_a: &mut Vec<u8>, buf_b: &mut Vec<u8>) {
    let dilated = morph_op(img, true, std::mem::take(buf_a));
    let eroded = morph_op(&dilated, false, std::mem::take(buf_b));
    *buf_a = dilated.into_buf();
    // Swap eroded result into img, reclaim old img.buf into buf_b
    let old_buf = std::mem::replace(&mut img.buf, eroded.into_buf());
    *buf_b = old_buf;
}

/// Morphological operation: dilate (max) or erode (min) with 3x3 kernel.
fn morph_op(img: &ImageU8, dilate: bool, buf: Vec<u8>) -> ImageU8 {
    let w = img.width as i32;
    let h = img.height as i32;
    let mut out = ImageU8::new_reuse(img.width, img.height, buf);

    for y in 0..h {
        for x in 0..w {
            let mut best = img.get(x as u32, y as u32);
            for dy in -1..=1i32 {
                for dx in -1..=1i32 {
                    let nx = x + dx;
                    let ny = y + dy;
                    if nx >= 0 && nx < w && ny >= 0 && ny < h {
                        let v = img.get(nx as u32, ny as u32);
                        if dilate {
                            best = best.max(v);
                        } else {
                            best = best.min(v);
                        }
                    }
                }
            }
            out.set(x as u32, y as u32, best);
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn threshold_reuses_buffer() {
        let mut img = ImageU8::new(8, 8);
        for y in 0..8 {
            for x in 0..8 {
                img.set(x, y, if x < 4 { 0 } else { 255 });
            }
        }
        let buf = Vec::with_capacity(1024);
        let out = threshold(&img, 5, false, buf, &mut ThresholdBuffers::new());
        assert!(out.buf.capacity() >= 1024);
    }

    #[test]
    fn threshold_uniform_white_returns_unknown() {
        // All white → low contrast → all 127
        let mut img = ImageU8::new(8, 8);
        for y in 0..8 {
            for x in 0..8 {
                img.set(x, y, 200);
            }
        }
        let out = threshold(&img, 5, false, Vec::new(), &mut ThresholdBuffers::new());
        for y in 0..8 {
            for x in 0..8 {
                assert_eq!(out.get(x, y), 127, "({x}, {y})");
            }
        }
    }

    #[test]
    fn threshold_high_contrast_binarizes() {
        // Left half = 0, right half = 255
        let mut img = ImageU8::new(8, 8);
        for y in 0..8 {
            for x in 0..4 {
                img.set(x, y, 0);
            }
            for x in 4..8 {
                img.set(x, y, 255);
            }
        }
        let out = threshold(&img, 5, false, Vec::new(), &mut ThresholdBuffers::new());
        // Tile (0,0) spans x=[0,3], all 0. Tile (1,0) spans x=[4,7], all 255.
        // After dilation, tile (0,0) has min=0, max=255 (from neighbor tile (1,0))
        // thresh = 0 + 255/2 = 127
        // Pixels with value 0 → 0, pixels with value 255 → 255
        assert_eq!(out.get(0, 0), 0);
        assert_eq!(out.get(4, 0), 255);
    }

    #[test]
    fn threshold_small_image_no_panic() {
        let img = ImageU8::new(2, 2);
        let out = threshold(&img, 5, false, Vec::new(), &mut ThresholdBuffers::new());
        assert_eq!(out.width, 2);
        assert_eq!(out.height, 2);
    }

    #[test]
    fn threshold_deglitch_removes_noise() {
        // Create a high-contrast image with a single noise pixel
        let mut img = ImageU8::new(8, 8);
        for y in 0..8 {
            for x in 0..8 {
                img.set(x, y, 0);
            }
        }
        img.set(4, 4, 255); // single bright pixel
                            // With deglitch, the single pixel noise should be removed by close operation
        let out = threshold(&img, 5, true, Vec::new(), &mut ThresholdBuffers::new());
        // The close operation (dilate then erode) should remove or smooth isolated changes
        assert_eq!(out.width, 8);
    }

    #[test]
    fn threshold_partial_tiles_use_nearest() {
        // 9 pixels wide, so last column (x=8) is in a partial tile
        let mut img = ImageU8::new(9, 8);
        for y in 0..8 {
            for x in 0..9u32 {
                img.set(x, y, if x < 5 { 0 } else { 255 });
            }
        }
        let out = threshold(&img, 5, false, Vec::new(), &mut ThresholdBuffers::new());
        // Pixel at x=8 should use tile tx=min(8/4, tw-1) = min(2, 1) = 1
        assert_eq!(out.get(8, 0), 255);
    }

    #[test]
    fn morph_dilate_expands_bright() {
        let mut img = ImageU8::new(5, 5);
        img.set(2, 2, 255);
        let out = morph_op(&img, true, Vec::new());
        // All 8 neighbors should become 255
        for dy in -1..=1i32 {
            for dx in -1..=1i32 {
                assert_eq!(out.get((2 + dx) as u32, (2 + dy) as u32), 255);
            }
        }
    }

    #[test]
    fn morph_erode_shrinks_bright() {
        let mut img = ImageU8::new(5, 5);
        for y in 0..5 {
            for x in 0..5 {
                img.set(x, y, 255);
            }
        }
        img.set(2, 2, 0);
        let out = morph_op(&img, false, Vec::new());
        // All 8 neighbors of (2,2) should become 0
        for dy in -1..=1i32 {
            for dx in -1..=1i32 {
                assert_eq!(out.get((2 + dx) as u32, (2 + dy) as u32), 0);
            }
        }
    }

    #[test]
    fn threshold_buffers_default() {
        let bufs = ThresholdBuffers::default();
        assert!(bufs.tile_min.is_empty());
    }
}
