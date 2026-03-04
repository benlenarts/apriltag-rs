use super::image::ImageU8;
use super::unionfind::UnionFind;

/// Build connected components on a thresholded image using union-find.
///
/// Two adjacent pixels are connected if they have the same threshold value
/// (both 0 or both 255). Pixels with value 127 (unknown) are never connected.
///
/// Uses asymmetric connectivity: diagonals are only checked for white pixels.
pub fn connected_components(threshed: &ImageU8) -> UnionFind {
    let w = threshed.width;
    let h = threshed.height;
    let mut uf = UnionFind::new((w * h) as usize);

    for y in 0..h {
        for x in 0..w {
            let v = threshed.get(x, y);
            if v == 127 {
                continue;
            }

            let id = y * w + x;

            // Cache left neighbor — used 3x across union() calls below.
            // union() clobbers caller-saved registers, forcing redundant
            // reloads of the immutable image buffer.
            let left = if x > 0 { threshed.get(x - 1, y) } else { 127 };

            // Left neighbor (always check)
            if left == v {
                uf.union(id, id - 1);
            }

            // Up neighbor: skip if left, upper-left, and up all equal v
            if y > 0 {
                let up = threshed.get(x, y - 1);
                if up == v {
                    let upper_left = if x > 0 {
                        threshed.get(x - 1, y - 1)
                    } else {
                        127
                    };
                    if !(left == v && upper_left == v) {
                        uf.union(id, id - w);
                    }
                }
            }

            // Upper-left diagonal: only for white pixels
            if v == 255 && left != 255 && x > 0 && y > 0 {
                let up = threshed.get(x, y - 1);
                if up != 255 {
                    let upper_left = threshed.get(x - 1, y - 1);
                    if upper_left == 255 {
                        uf.union(id, id - w - 1);
                    }
                }
            }

            // Upper-right diagonal: only for white pixels
            if v == 255 && x + 1 < w && y > 0 {
                let up = threshed.get(x, y - 1);
                if up != 255 {
                    let upper_right = threshed.get(x + 1, y - 1);
                    if upper_right == 255 {
                        uf.union(id, id - w + 1);
                    }
                }
            }
        }
    }

    uf
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_thresh(w: u32, h: u32, pixels: &[u8]) -> ImageU8 {
        ImageU8::from_buf(w, h, w, pixels.to_vec())
    }

    #[test]
    fn uniform_black_single_component() {
        // 3x3 all black → one component
        let img = make_thresh(3, 3, &[0; 9]);
        let mut uf = connected_components(&img);
        let r = uf.find(0);
        for i in 1..9u32 {
            assert_eq!(uf.find(i), r);
        }
    }

    #[test]
    fn unknown_pixels_not_connected() {
        // All unknown → each pixel is its own root
        let img = make_thresh(3, 3, &[127; 9]);
        let mut uf = connected_components(&img);
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
        let mut uf = connected_components(&img);
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
        let mut uf = connected_components(&img);
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
        let mut uf = connected_components(&img);
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
        let mut uf = connected_components(&img);
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
        let mut uf = connected_components(&img);
        // Black component: (0,0),(1,0),(0,1),(0,2),(1,2) = 5 pixels
        assert_eq!(uf.set_size(0), 5);
        // White component: (2,0),(2,1),(2,2) = 3 pixels
        assert_eq!(uf.set_size(2), 3);
    }
}
