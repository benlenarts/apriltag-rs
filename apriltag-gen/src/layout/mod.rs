mod validate;

use crate::error::LayoutError;
use crate::types::CellType;

/// A parsed tag layout defining the spatial arrangement of cells.
#[derive(Debug, Clone)]
pub struct Layout {
    /// Grid dimension (the layout is grid_size x grid_size).
    pub grid_size: usize,
    /// Cell types in row-major order (top-to-bottom, left-to-right).
    pub cells: Vec<CellType>,
    /// Number of data bits in this layout.
    pub nbits: usize,
    /// Row/column index of the black border ring.
    pub border_start: usize,
    /// Width of the data region at the inner border edge.
    pub border_width: usize,
    /// Whether the border order is reversed (Standard/Circle = true, Classic = false).
    pub reversed_border: bool,
}

impl Layout {
    /// Parse a layout from a data string of `d`, `b`, `w`, `x` characters.
    ///
    /// The string length must be a perfect square. The layout must be
    /// rotationally symmetric and contain a valid border.
    pub fn from_data_string(data: &str) -> Result<Layout, LayoutError> {
        let len = data.len();
        let grid_size = (len as f64).sqrt() as usize;
        if grid_size * grid_size != len {
            return Err(LayoutError::NotSquare(len));
        }

        let cells: Vec<CellType> = data
            .chars()
            .enumerate()
            .map(|(i, c)| match c {
                'd' => Ok(CellType::Data),
                'b' => Ok(CellType::Black),
                'w' => Ok(CellType::White),
                'x' => Ok(CellType::Ignored),
                _ => Err(LayoutError::InvalidChar(c, i)),
            })
            .collect::<Result<Vec<_>, _>>()?;

        let nbits = cells.iter().filter(|&&c| c == CellType::Data).count();

        validate::check_symmetry(&cells, grid_size)?;
        let (border_start, reversed_border) = validate::detect_border(&cells, grid_size)?;
        validate::check_border_rings(&cells, grid_size, border_start, reversed_border)?;

        let border_width = grid_size - 2 * border_start;

        Ok(Layout {
            grid_size,
            cells,
            nbits,
            border_start,
            border_width,
            reversed_border,
        })
    }

    /// Get the cell type at grid position (x, y).
    pub fn cell(&self, x: usize, y: usize) -> CellType {
        self.cells[y * self.grid_size + x]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_classic_8x8_tag16h5() {
        // Classic 8x8: dist==0 → w, dist==1 → b, dist>=2 → d
        let data = "wwwwwwwwwbbbbbbwwbddddbwwbddddbwwbddddbwwbddddbwwbbbbbbwwwwwwwww";
        let layout = Layout::from_data_string(data).unwrap();
        assert_eq!(layout.grid_size, 8);
        assert_eq!(layout.nbits, 16);
        assert_eq!(layout.border_start, 1);
        assert_eq!(layout.border_width, 6);
        assert!(!layout.reversed_border);
    }

    #[test]
    fn parse_not_square() {
        let result = Layout::from_data_string("ddd");
        assert!(matches!(result, Err(LayoutError::NotSquare(3))));
    }

    #[test]
    fn parse_invalid_char() {
        let result = Layout::from_data_string("dddZddddd");
        assert!(matches!(result, Err(LayoutError::InvalidChar('Z', 3))));
    }

    #[test]
    fn parse_circle21h7_layout() {
        let data = "xxxdddxxxxbbbbbbbxxbwwwwwbxdbwdddwbddbwdddwbddbwdddwbdxbwwwwwbxxbbbbbbbxxxxdddxxx";
        let layout = Layout::from_data_string(data).unwrap();
        assert_eq!(layout.grid_size, 9);
        assert_eq!(layout.nbits, 21);
        assert!(layout.reversed_border);
    }

    #[test]
    fn parse_standard41h12_layout() {
        let data = "ddddddddddbbbbbbbddbwwwwwbddbwdddwbddbwdddwbddbwdddwbddbwwwwwbddbbbbbbbdddddddddd";
        let layout = Layout::from_data_string(data).unwrap();
        assert_eq!(layout.grid_size, 9);
        assert_eq!(layout.nbits, 41);
        assert!(layout.reversed_border);
    }
}
