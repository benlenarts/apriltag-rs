use super::image::ImageU8;
#[cfg(feature = "parallel")]
use super::par::Par;
use super::unionfind::UnionFind;

/// Build connected components on a thresholded image using union-find.
///
/// Two adjacent pixels are connected if they have the same threshold value
/// (both 0 or both 255). Pixels with value 127 (unknown) are never connected.
///
/// Uses asymmetric connectivity: diagonals are only checked for white pixels.
///
/// Pass a pre-allocated `uf` (e.g. from a previous call) to reuse memory.
/// Use `&mut UnionFind::empty()` for one-shot usage.
///
/// When the `parallel` feature is enabled and multiple rayon threads are
/// available, uses row-strip decomposition with shared atomic union-find.
pub fn connected_components(threshed: &ImageU8, uf: &mut UnionFind) {
    #[cfg(feature = "parallel")]
    if matches!(Par::get(), Par::Parallel) {
        connected_components_par(threshed, uf);
        return;
    }
    connected_components_seq(threshed, uf);
}

/// Sequential connected components — identical to the original algorithm.
fn connected_components_seq(threshed: &ImageU8, uf: &mut UnionFind) {
    let w = threshed.width;
    let h = threshed.height;
    let buf = &threshed.buf;
    let stride = threshed.stride;

    assert!(buf.len() >= (h * stride) as usize);

    uf.reset((w * h) as usize);

    for y in 0..h {
        let row = (y * stride) as usize;
        for x in 0..w {
            let v = buf[row + x as usize];
            if v == 127 {
                continue;
            }

            let id = y * w + x;

            let left = if x > 0 {
                buf[row + x as usize - 1]
            } else {
                127
            };

            if left == v {
                uf.union(id, id - 1);
            }

            if y > 0 {
                let prev_row = row - stride as usize;
                let up = buf[prev_row + x as usize];
                if up == v {
                    let upper_left = if x > 0 {
                        buf[prev_row + x as usize - 1]
                    } else {
                        127
                    };
                    if !(left == v && upper_left == v) {
                        uf.union(id, id - w);
                    }
                }
            }

            if v == 255 && left != 255 && x > 0 && y > 0 {
                let prev_row = row - stride as usize;
                let up = buf[prev_row + x as usize];
                if up != 255 {
                    let upper_left = buf[prev_row + x as usize - 1];
                    if upper_left == 255 {
                        uf.union(id, id - w - 1);
                    }
                }
            }

            if v == 255 && x + 1 < w && y > 0 {
                let prev_row = row - stride as usize;
                let up = buf[prev_row + x as usize];
                if up != 255 {
                    let upper_right = buf[prev_row + x as usize + 1];
                    if upper_right == 255 {
                        uf.union(id, id - w + 1);
                    }
                }
            }
        }
    }
}

/// Parallel connected components using row-strip decomposition.
///
/// Algorithm:
/// 1. Reset UF for w*h elements
/// 2. Process first row serially (left-neighbor only, &mut self)
/// 3. Split remaining rows into strips with 1-row gaps between them
/// 4. Process strips in parallel using find_shared/union_shared (&self)
/// 5. Stitch gap rows serially (&mut self)
#[cfg(feature = "parallel")]
fn connected_components_par(threshed: &ImageU8, uf: &mut UnionFind) {
    let w = threshed.width;
    let h = threshed.height;
    let buf = &threshed.buf;
    let stride = threshed.stride;

    assert!(buf.len() >= (h * stride) as usize);
    uf.reset((w * h) as usize);

    // COVERAGE: parallel path only taken with parallel feature + multi-thread
    if h == 0 || w == 0 {
        return;
    }

    // Process first row serially (left-neighbor only)
    {
        let row = 0usize;
        for x in 1..w {
            let v = buf[row + x as usize];
            if v == 127 {
                continue;
            }
            let left = buf[row + x as usize - 1];
            if left == v {
                uf.union(x, x - 1);
            }
        }
    }

    // COVERAGE: parallel path only taken with parallel feature + multi-thread
    if h <= 1 {
        return;
    }

    // Compute strip boundaries for rows 1..h
    let nthreads = rayon::current_num_threads();
    let chunk = (h as usize - 1).max(1) / (nthreads * 4).max(1);
    let chunk = chunk.max(64) as u32;

    // Build strip ranges: [y_start, y_end) with gap rows between strips
    let mut strips: Vec<(u32, u32)> = Vec::new();
    let mut y = 1u32;
    while y < h {
        let y_end = (y + chunk).min(h);
        strips.push((y, y_end));
        // Skip gap row (will be stitched serially)
        y = y_end + 1;
    }

    // Process strips in parallel using shared atomic union-find
    rayon::scope(|s| {
        for &(y_start, y_end) in &strips {
            // SAFETY rationale: each strip processes its own rows independently.
            // union_shared/find_shared use atomic CAS — benign data races on
            // shared UF nodes are correct by construction.
            let uf_ref: &UnionFind = uf;
            s.spawn(move |_| {
                connected_components_strip(buf, w, stride, uf_ref, y_start, y_end);
            });
        }
    });

    // Stitch gap rows serially (the row between each pair of strips)
    for window in strips.windows(2) {
        let gap_y = window[0].1; // first row after strip 0 = gap row
                                 // COVERAGE: guard for edge case where last strip ends at image boundary
        if gap_y >= h {
            continue;
        }
        stitch_row(buf, w, stride, uf, gap_y);
    }
}

/// Process rows [y_start, y_end) using lock-free find_shared/union_shared.
#[cfg(feature = "parallel")]
fn connected_components_strip(
    buf: &[u8],
    w: u32,
    stride: u32,
    uf: &UnionFind,
    y_start: u32,
    y_end: u32,
) {
    for y in y_start..y_end {
        let row = (y * stride) as usize;
        for x in 0..w {
            let v = buf[row + x as usize];
            if v == 127 {
                continue;
            }

            let id = y * w + x;

            let left = if x > 0 {
                buf[row + x as usize - 1]
            } else {
                127
            };

            if left == v {
                uf.union_shared(id, id - 1);
            }

            if y > 0 {
                let prev_row = row - stride as usize;
                let up = buf[prev_row + x as usize];
                if up == v {
                    let upper_left = if x > 0 {
                        buf[prev_row + x as usize - 1]
                    } else {
                        127
                    };
                    if !(left == v && upper_left == v) {
                        uf.union_shared(id, id - w);
                    }
                }
            }

            if v == 255 && left != 255 && x > 0 && y > 0 {
                let prev_row = row - stride as usize;
                let up = buf[prev_row + x as usize];
                if up != 255 {
                    let upper_left = buf[prev_row + x as usize - 1];
                    if upper_left == 255 {
                        uf.union_shared(id, id - w - 1);
                    }
                }
            }

            if v == 255 && x + 1 < w && y > 0 {
                let prev_row = row - stride as usize;
                let up = buf[prev_row + x as usize];
                if up != 255 {
                    let upper_right = buf[prev_row + x as usize + 1];
                    if upper_right == 255 {
                        uf.union_shared(id, id - w + 1);
                    }
                }
            }
        }
    }
}

/// Stitch a single gap row using serial &mut self union.
///
/// Connects pixels in row `y` to their neighbors in row `y-1` (already
/// processed by the strip above) and to left neighbors in the same row.
#[cfg(feature = "parallel")]
fn stitch_row(buf: &[u8], w: u32, stride: u32, uf: &mut UnionFind, y: u32) {
    let row = (y * stride) as usize;
    for x in 0..w {
        let v = buf[row + x as usize];
        if v == 127 {
            continue;
        }

        let id = y * w + x;

        let left = if x > 0 {
            buf[row + x as usize - 1]
        } else {
            127
        };

        // Left neighbor
        if left == v {
            uf.union(id, id - 1);
        }

        // Up neighbor
        let prev_row = row - stride as usize;
        let up = buf[prev_row + x as usize];
        if up == v {
            let upper_left = if x > 0 {
                buf[prev_row + x as usize - 1]
            } else {
                127
            };
            if !(left == v && upper_left == v) {
                uf.union(id, id - w);
            }
        }

        // Upper-left diagonal (white only)
        if v == 255 && left != 255 && x > 0 {
            let up = buf[prev_row + x as usize];
            if up != 255 {
                let upper_left = buf[prev_row + x as usize - 1];
                if upper_left == 255 {
                    uf.union(id, id - w - 1);
                }
            }
        }

        // Upper-right diagonal (white only)
        if v == 255 && x + 1 < w {
            let up = buf[prev_row + x as usize];
            if up != 255 {
                let upper_right = buf[prev_row + x as usize + 1];
                if upper_right == 255 {
                    uf.union(id, id - w + 1);
                }
            }
        }
    }

    // Also connect to the row below (y+1, first row of next strip)
    // That row was processed by the next strip which could see y+1's
    // up-neighbors, but the gap row y wasn't processed yet, so we
    // need to process y+1 looking at y as well.
    let next_y = y + 1;
    // COVERAGE: bounds guards; gap rows are always interior in practice
    if next_y >= (buf.len() as u32).div_ceil(stride) {
        return; // no row below
    }
    let next_row = (next_y * stride) as usize;
    if next_row + w as usize > buf.len() {
        return;
    }
    for x in 0..w {
        let v = buf[next_row + x as usize];
        if v == 127 {
            continue;
        }
        let id = next_y * w + x;

        // Up neighbor (row y)
        let up = buf[row + x as usize];
        if up == v {
            let left = if x > 0 {
                buf[next_row + x as usize - 1]
            } else {
                127
            };
            let upper_left = if x > 0 {
                buf[row + x as usize - 1]
            } else {
                127
            };
            if !(left == v && upper_left == v) {
                uf.union(id, id - w);
            }
        }

        // Upper-left diagonal (white only)
        if v == 255 && x > 0 {
            let left = buf[next_row + x as usize - 1];
            if left != 255 {
                let up = buf[row + x as usize];
                if up != 255 {
                    let upper_left = buf[row + x as usize - 1];
                    if upper_left == 255 {
                        uf.union(id, id - w - 1);
                    }
                }
            }
        }

        // Upper-right diagonal (white only)
        if v == 255 && x + 1 < w {
            let up = buf[row + x as usize];
            if up != 255 {
                let upper_right = buf[row + x as usize + 1];
                if upper_right == 255 {
                    uf.union(id, id - w + 1);
                }
            }
        }
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use super::*;

    fn make_thresh(w: u32, h: u32, pixels: &[u8]) -> ImageU8 {
        ImageU8::from_buf(w, h, w, pixels.to_vec())
    }

    fn run_cc(img: &ImageU8) -> UnionFind {
        let mut uf = UnionFind::empty();
        connected_components(img, &mut uf);
        uf
    }

    #[test]
    fn connected_components_reuses_allocation() {
        let img = make_thresh(3, 3, &[0; 9]);
        let mut uf = UnionFind::new(100); // larger than needed
        connected_components(&img, &mut uf);
        // Should have been reset to 9 elements but kept capacity
        assert_eq!(uf.set_size(0), 9);
    }

    #[test]
    fn uniform_black_single_component() {
        // 3x3 all black → one component
        let img = make_thresh(3, 3, &[0; 9]);
        let mut uf = run_cc(&img);
        let r = uf.find(0);
        for i in 1..9u32 {
            assert_eq!(uf.find(i), r);
        }
    }

    #[test]
    fn unknown_pixels_not_connected() {
        // All unknown → each pixel is its own root
        let img = make_thresh(3, 3, &[127; 9]);
        let mut uf = run_cc(&img);
        for i in 0..9u32 {
            assert_eq!(uf.find(i), i);
        }
    }

    #[test]
    fn black_white_separate_components() {
        // Left column black, right column white
        #[rustfmt::skip]
        let pixels = [
            0, 255,
            0, 255,
        ];
        let img = make_thresh(2, 2, &pixels);
        let mut uf = run_cc(&img);
        // Black pixels (0,0) and (0,1) should be in same component
        assert_eq!(uf.find(0), uf.find(2));
        // White pixels (1,0) and (1,1) should be in same component
        assert_eq!(uf.find(1), uf.find(3));
        // Black and white should be separate
        assert_ne!(uf.find(0), uf.find(1));
    }

    #[test]
    fn white_diagonal_connected() {
        // White pixels only connected diagonally
        #[rustfmt::skip]
        let pixels = [
            255,   0,
              0, 255,
        ];
        let img = make_thresh(2, 2, &pixels);
        let mut uf = run_cc(&img);
        // White (0,0) and (1,1) should NOT be connected (upper-right diagonal
        // check is from perspective of (1,1) looking at upper-left, but (0,0)
        // is upper-left of (1,1), and left=(0,1)=0≠255, up=(1,0)=0≠255,
        // so they SHOULD be connected)
        assert_eq!(uf.find(0), uf.find(3));
    }

    #[test]
    fn black_diagonal_not_connected() {
        // Black pixels only connected diagonally → should NOT connect
        #[rustfmt::skip]
        let pixels = [
              0, 255,
            255,   0,
        ];
        let img = make_thresh(2, 2, &pixels);
        let mut uf = run_cc(&img);
        // Black (0,0) and (1,1) should NOT be connected (diagonals only for white)
        assert_ne!(uf.find(0), uf.find(3));
    }

    #[test]
    fn skip_up_when_path_exists() {
        // When left, upper-left, and up all have same value, skip the up union
        #[rustfmt::skip]
        let pixels = [
            0, 0,
            0, 0,
        ];
        let img = make_thresh(2, 2, &pixels);
        let mut uf = run_cc(&img);
        // All should still be connected (via left and other paths)
        let r = uf.find(0);
        for i in 1..4u32 {
            assert_eq!(uf.find(i), r);
        }
    }

    #[test]
    fn component_sizes_correct() {
        #[rustfmt::skip]
        let pixels = [
            0,   0, 255,
            0, 127, 255,
            0,   0, 255,
        ];
        let img = make_thresh(3, 3, &pixels);
        let mut uf = run_cc(&img);
        // Black component: (0,0),(1,0),(0,1),(0,2),(1,2) = 5 pixels
        assert_eq!(uf.set_size(0), 5);
        // White component: (2,0),(2,1),(2,2) = 3 pixels
        assert_eq!(uf.set_size(2), 3);
    }

    /// Compare parallel vs sequential results on a given image.
    /// Returns the sequential UF for further assertions.
    #[cfg(feature = "parallel")]
    fn assert_par_matches_seq(img: &ImageU8) {
        use std::collections::HashMap;

        let mut uf_seq = UnionFind::empty();
        connected_components_seq(img, &mut uf_seq);

        let mut uf_par = UnionFind::empty();
        connected_components_par(img, &mut uf_par);

        // Flatten both to get canonical roots
        uf_seq.flatten();
        uf_par.flatten();

        let n = (img.width * img.height) as u32;

        // Build equivalence classes for sequential
        let mut seq_classes: HashMap<u32, Vec<u32>> = HashMap::new();
        for i in 0..n {
            seq_classes.entry(uf_seq.find_flat(i)).or_default().push(i);
        }

        // Verify parallel produces the same equivalence classes
        for (_, members) in &seq_classes {
            let par_root = uf_par.find_flat(members[0]);
            for &m in &members[1..] {
                // COVERAGE: assertion message only reached on failure
                assert_eq!(
                    uf_par.find_flat(m),
                    par_root,
                    "pixel {m} has different root in parallel vs sequential"
                );
            }
        }

        // Also verify parallel doesn't merge extra things
        let mut par_classes: HashMap<u32, Vec<u32>> = HashMap::new();
        for i in 0..n {
            par_classes.entry(uf_par.find_flat(i)).or_default().push(i);
        }
        // COVERAGE: assertion message only reached on failure
        assert_eq!(
            seq_classes.len(),
            par_classes.len(),
            "different number of components: seq={}, par={}",
            seq_classes.len(),
            par_classes.len()
        );
    }

    #[cfg(feature = "parallel")]
    #[test]
    fn parallel_matches_sequential_uniform_black() {
        let img = make_thresh(200, 200, &vec![0u8; 200 * 200]);
        assert_par_matches_seq(&img);
    }

    #[cfg(feature = "parallel")]
    #[test]
    fn parallel_matches_sequential_checkerboard() {
        let w = 100u32;
        let h = 200u32;
        let mut pixels = vec![0u8; (w * h) as usize];
        for y in 0..h {
            for x in 0..w {
                pixels[(y * w + x) as usize] = if (x + y) % 2 == 0 { 0 } else { 255 };
            }
        }
        let img = make_thresh(w, h, &pixels);
        assert_par_matches_seq(&img);
    }

    #[cfg(feature = "parallel")]
    #[test]
    fn parallel_matches_sequential_horizontal_stripes() {
        let w = 100u32;
        let h = 200u32;
        let mut pixels = vec![0u8; (w * h) as usize];
        for y in 0..h {
            let v = if y % 4 < 2 { 0 } else { 255 };
            for x in 0..w {
                pixels[(y * w + x) as usize] = v;
            }
        }
        let img = make_thresh(w, h, &pixels);
        assert_par_matches_seq(&img);
    }

    #[cfg(feature = "parallel")]
    #[test]
    fn parallel_matches_sequential_with_unknown() {
        let w = 80u32;
        let h = 150u32;
        let mut pixels = vec![0u8; (w * h) as usize];
        for y in 0..h {
            for x in 0..w {
                pixels[(y * w + x) as usize] = if x % 3 == 0 {
                    127
                } else if y % 2 == 0 {
                    0
                } else {
                    255
                };
            }
        }
        let img = make_thresh(w, h, &pixels);
        assert_par_matches_seq(&img);
    }

    #[cfg(feature = "parallel")]
    #[test]
    fn parallel_component_spanning_strip_boundary() {
        // A single black component that spans many rows, forcing it to cross
        // strip boundaries. The gap-stitching must merge them correctly.
        let w = 10u32;
        let h = 300u32;
        let mut pixels = vec![127u8; (w * h) as usize];
        // Single column of black pixels down the middle
        for y in 0..h {
            pixels[(y * w + 5) as usize] = 0;
        }
        let img = make_thresh(w, h, &pixels);
        assert_par_matches_seq(&img);
    }

    #[cfg(feature = "parallel")]
    #[test]
    fn parallel_matches_sequential_white_diagonal() {
        // White diagonal pattern to test diagonal connectivity across strip boundaries
        let w = 50u32;
        let h = 200u32;
        let mut pixels = vec![0u8; (w * h) as usize];
        for y in 0..h {
            let x = y % w;
            pixels[(y * w + x) as usize] = 255;
        }
        let img = make_thresh(w, h, &pixels);
        assert_par_matches_seq(&img);
    }
}
