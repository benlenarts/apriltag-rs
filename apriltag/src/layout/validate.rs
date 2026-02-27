use crate::error::LayoutError;
use crate::types::CellType;

/// Check that the layout has 4-fold rotational symmetry.
///
/// For all cells in the first quadrant (y in 0..size/2, x in y..size-1-y),
/// verify that the 4 rotated positions have the same cell type.
pub fn check_symmetry(cells: &[CellType], size: usize) -> Result<(), LayoutError> {
    for y in 0..size / 2 {
        for x in y..size - 1 - y {
            let a = cells[y * size + x];
            let b = cells[x * size + (size - 1 - y)];
            let c = cells[(size - 1 - y) * size + (size - 1 - x)];
            let d = cells[(size - 1 - x) * size + y];
            if a != b || a != c || a != d {
                return Err(LayoutError::NotSymmetric);
            }
        }
    }
    Ok(())
}

/// Detect the border location and orientation.
///
/// Walks diagonally from corner (0,0) inward. Classic: finds white then black.
/// Standard/Circle: finds black then white.
pub fn detect_border(cells: &[CellType], size: usize) -> Result<(usize, bool), LayoutError> {
    for i in 0..(size - 1) / 2 {
        let outer = cells[i * size + i];
        let inner = cells[(i + 1) * size + (i + 1)];

        match (outer, inner) {
            (CellType::White, CellType::Black) => {
                // Classic: white outer, black inner
                return Ok((i + 1, false));
            }
            (CellType::Black, CellType::White) => {
                // Standard/Circle: black outer, white inner
                return Ok((i + 1, true));
            }
            _ => continue,
        }
    }
    Err(LayoutError::NoBorder)
}

/// Validate that the border rings are complete and correct.
///
/// The outer ring at `border_start - 1` must be entirely `outside_type`,
/// and the inner ring at `border_start` must be entirely `inside_type`,
/// across the top edge (and by symmetry, all four edges).
pub fn check_border_rings(
    cells: &[CellType],
    size: usize,
    border_start: usize,
    reversed: bool,
) -> Result<(), LayoutError> {
    let (outside_type, inside_type) = if reversed {
        (CellType::Black, CellType::White)
    } else {
        (CellType::White, CellType::Black)
    };

    let outer_row = border_start - 1;
    let inner_row = border_start;

    // Check outer ring along top edge
    for x in outer_row..size - outer_row {
        if cells[outer_row * size + x] != outside_type {
            return Err(LayoutError::InvalidBorder(format!(
                "outer ring cell ({}, {}) should be {:?}",
                x, outer_row, outside_type
            )));
        }
    }

    // Check inner ring along top edge
    for x in inner_row..size - inner_row {
        if cells[inner_row * size + x] != inside_type {
            return Err(LayoutError::InvalidBorder(format!(
                "inner ring cell ({}, {}) should be {:?}",
                x, inner_row, inside_type
            )));
        }
    }

    Ok(())
}
