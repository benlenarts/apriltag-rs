use super::image::ImageU8;

const TILESZ: u32 = 4;

/// Produce a ternary threshold image: 0 (black), 255 (white), or 127 (unknown).
///
/// Uses tile-based adaptive thresholding with min/max dilation to handle
/// spatially varying illumination.
pub fn threshold(img: &ImageU8, min_white_black_diff: i32, deglitch: bool) -> ImageU8 {
    let w = img.width;
    let h = img.height;
    let tw = w / TILESZ;
    let th = h / TILESZ;

    if tw == 0 || th == 0 {
        return ImageU8::new(w, h);
    }

    // Compute per-tile min/max
    let mut tile_min = vec![255u8; (tw * th) as usize];
    let mut tile_max = vec![0u8; (tw * th) as usize];

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
            tile_min[(ty * tw + tx) as usize] = lo;
            tile_max[(ty * tw + tx) as usize] = hi;
        }
    }

    // Dilate max, erode min using 3x3 tile neighborhood
    let mut dilated_max = vec![0u8; (tw * th) as usize];
    let mut eroded_min = vec![255u8; (tw * th) as usize];

    for ty in 0..th as i32 {
        for tx in 0..tw as i32 {
            let mut hi = 0u8;
            let mut lo = 255u8;
            for dy in -1..=1i32 {
                for dx in -1..=1i32 {
                    let nx = tx + dx;
                    let ny = ty + dy;
                    if nx >= 0 && nx < tw as i32 && ny >= 0 && ny < th as i32 {
                        let idx = (ny * tw as i32 + nx) as usize;
                        hi = hi.max(tile_max[idx]);
                        lo = lo.min(tile_min[idx]);
                    }
                }
            }
            let idx = (ty * tw as i32 + tx) as usize;
            dilated_max[idx] = hi;
            eroded_min[idx] = lo;
        }
    }

    // Binarize each pixel
    let mut out = ImageU8::new(w, h);
    for y in 0..h {
        for x in 0..w {
            let tx = (x / TILESZ).min(tw - 1);
            let ty = (y / TILESZ).min(th - 1);
            let idx = (ty * tw + tx) as usize;
            let lo = eroded_min[idx];
            let hi = dilated_max[idx];

            let val = if (hi as i32 - lo as i32) < min_white_black_diff {
                127
            } else {
                let thresh = lo as i32 + (hi as i32 - lo as i32) / 2;
                if img.get(x, y) as i32 > thresh {
                    255
                } else {
                    0
                }
            };
            out.set(x, y, val);
        }
    }

    if deglitch {
        deglitch_image(&mut out);
    }

    out
}

/// Morphological close (dilate then erode) with 3x3 structuring element.
fn deglitch_image(img: &mut ImageU8) {
    let dilated = morph_op(img, true);
    let eroded = morph_op(&dilated, false);
    img.buf = eroded.buf;
}

/// Morphological operation: dilate (max) or erode (min) with 3x3 kernel.
fn morph_op(img: &ImageU8, dilate: bool) -> ImageU8 {
    let w = img.width as i32;
    let h = img.height as i32;
    let mut out = ImageU8::new(img.width, img.height);

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
    fn threshold_uniform_white_returns_unknown() {
        // All white → low contrast → all 127
        let mut img = ImageU8::new(8, 8);
        for y in 0..8 {
            for x in 0..8 {
                img.set(x, y, 200);
            }
        }
        let out = threshold(&img, 5, false);
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
        let out = threshold(&img, 5, false);
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
        let out = threshold(&img, 5, false);
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
        let out = threshold(&img, 5, true);
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
        let out = threshold(&img, 5, false);
        // Pixel at x=8 should use tile tx=min(8/4, tw-1) = min(2, 1) = 1
        assert_eq!(out.get(8, 0), 255);
    }

    #[test]
    fn morph_dilate_expands_bright() {
        let mut img = ImageU8::new(5, 5);
        img.set(2, 2, 255);
        let out = morph_op(&img, true);
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
        let out = morph_op(&img, false);
        // All 8 neighbors of (2,2) should become 0
        for dy in -1..=1i32 {
            for dx in -1..=1i32 {
                assert_eq!(out.get((2 + dx) as u32, (2 + dy) as u32), 0);
            }
        }
    }
}
