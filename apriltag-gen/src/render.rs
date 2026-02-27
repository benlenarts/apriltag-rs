use crate::layout::Layout;
use crate::types::{CellType, Pixel};

/// A rendered tag as a grid of pixels.
#[derive(Debug, Clone)]
pub struct RenderedTag {
    /// Grid dimension (same as layout grid_size).
    pub grid_size: usize,
    /// Pixel values in row-major order.
    pub pixels: Vec<Pixel>,
}

impl RenderedTag {
    /// Get the pixel at position (x, y).
    pub fn pixel(&self, x: usize, y: usize) -> Pixel {
        self.pixels[y * self.grid_size + x]
    }

    /// Convert to RGBA pixel data (4 bytes per pixel).
    ///
    /// Black = (0, 0, 0, 255), White = (255, 255, 255, 255),
    /// Transparent = (0, 0, 0, 0).
    pub fn to_rgba(&self) -> Vec<u8> {
        self.pixels
            .iter()
            .flat_map(|p| match p {
                Pixel::Black => [0, 0, 0, 255],
                Pixel::White => [255, 255, 255, 255],
                Pixel::Transparent => [0, 0, 0, 0],
            })
            .collect()
    }
}

/// Render a code using the given layout.
///
/// Matches the Java `ImageLayout.renderToArray()` algorithm:
/// 1. Loop 4 times: rotate image 90 degrees, then fill the top half strip
/// 2. Handle center pixel if grid_size is odd
/// 3. Apply one final rotate90
pub fn render(layout: &Layout, code: u64) -> RenderedTag {
    let size = layout.grid_size;
    let mut im = vec![vec![Pixel::Transparent; size]; size];
    let mut code = code;

    for _ in 0..4 {
        im = rotate90_image(&im);

        #[allow(clippy::needless_range_loop)]
        for y in 0..=size / 2 {
            for x in y..size.saturating_sub(1).saturating_sub(y) {
                let cell = layout.cell(x, y);
                let pixel = match cell {
                    CellType::Data => {
                        let bit = (code >> (layout.nbits as u64 - 1)) & 1;
                        code <<= 1;
                        if bit != 0 {
                            Pixel::White
                        } else {
                            Pixel::Black
                        }
                    }
                    CellType::Black => Pixel::Black,
                    CellType::White => Pixel::White,
                    CellType::Ignored => Pixel::Transparent,
                };
                im[y][x] = pixel;
            }
        }
    }

    // Handle center pixel for odd grid sizes
    if size % 2 == 1 {
        let mid = size / 2;
        let cell = layout.cell(mid, mid);
        im[mid][mid] = match cell {
            CellType::Data => {
                let bit = (code >> (layout.nbits as u64 - 1)) & 1;
                if bit != 0 {
                    Pixel::White
                } else {
                    Pixel::Black
                }
            }
            CellType::Black => Pixel::Black,
            CellType::White => Pixel::White,
            CellType::Ignored => Pixel::Transparent,
        };
    }

    // Final rotation
    let im = rotate90_image(&im);

    let pixels: Vec<Pixel> = im.into_iter().flat_map(|row| row.into_iter()).collect();
    RenderedTag { grid_size: size, pixels }
}

/// Rotate a 2D image 90 degrees clockwise.
///
/// Maps (y, x) → (size-1-x, y), matching Java `ImageLayout.rotate90()`.
fn rotate90_image(im: &[Vec<Pixel>]) -> Vec<Vec<Pixel>> {
    let size = im.len();
    let mut out = vec![vec![Pixel::Transparent; size]; size];
    for y in 0..size {
        for x in 0..size {
            out[size - 1 - x][y] = im[y][x];
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::layout::Layout;

    #[test]
    fn render_tag16h5_code0_borders_correct() {
        let layout = Layout::classic(8).unwrap();
        let tag = render(&layout, 0x27c8); // tag16h5 code 0

        // All edges should be white (classic border)
        for i in 0..8 {
            assert_eq!(tag.pixel(i, 0), Pixel::White, "top edge ({i}, 0)");
            assert_eq!(tag.pixel(i, 7), Pixel::White, "bottom edge ({i}, 7)");
            assert_eq!(tag.pixel(0, i), Pixel::White, "left edge (0, {i})");
            assert_eq!(tag.pixel(7, i), Pixel::White, "right edge (7, {i})");
        }

        // Inner border ring should be black
        for i in 1..7 {
            assert_eq!(tag.pixel(i, 1), Pixel::Black, "inner top ({i}, 1)");
            assert_eq!(tag.pixel(i, 6), Pixel::Black, "inner bottom ({i}, 6)");
        }
        for i in 2..6 {
            assert_eq!(tag.pixel(1, i), Pixel::Black, "inner left (1, {i})");
            assert_eq!(tag.pixel(6, i), Pixel::Black, "inner right (6, {i})");
        }
    }

    #[test]
    fn render_circle21h7_code0_no_transparent_inside() {
        let data = "xxxdddxxxxbbbbbbbxxbwwwwwbxdbwdddwbddbwdddwbddbwdddwbdxbwwwwwbxxbbbbbbbxxxxdddxxx";
        let layout = Layout::from_data_string(data).unwrap();
        let tag = render(&layout, 0x157863);

        // Corner cells should be transparent
        assert_eq!(tag.pixel(0, 0), Pixel::Transparent);
        assert_eq!(tag.pixel(8, 0), Pixel::Transparent);
        assert_eq!(tag.pixel(0, 8), Pixel::Transparent);
        assert_eq!(tag.pixel(8, 8), Pixel::Transparent);

        // Center cell should be black or white (not transparent)
        assert_ne!(tag.pixel(4, 4), Pixel::Transparent);
    }

    #[test]
    fn render_all_zeros_data_is_black() {
        let layout = Layout::classic(8).unwrap();
        let tag = render(&layout, 0x0000);

        // All data cells should be black for code 0
        for y in 2..6 {
            for x in 2..6 {
                assert_eq!(tag.pixel(x, y), Pixel::Black, "data ({x}, {y})");
            }
        }
    }

    #[test]
    fn render_to_rgba_correct_size() {
        let layout = Layout::classic(8).unwrap();
        let tag = render(&layout, 0x27c8);
        let rgba = tag.to_rgba();
        assert_eq!(rgba.len(), 8 * 8 * 4);
    }
}
