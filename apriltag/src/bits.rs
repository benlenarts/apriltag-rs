use crate::layout::Layout;
use crate::types::CellType;

/// A bit location in grid coordinates, shifted by border_start.
///
/// Coordinates can be negative for Standard/Circle layouts where
/// data bits extend outside the inner border.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BitLocation {
    pub x: i32,
    pub y: i32,
}

/// Compute bit locations for a layout using quadrant-scan ordering.
///
/// This matches the Java `ImageLayout.getBitLocations()` algorithm:
/// 1. Scan the first quadrant (top-left triangle) for data cells
/// 2. Generate remaining three quadrants by rotating coordinates 90 degrees
/// 3. If grid_size is odd and center cell is data, add it as the final bit
/// 4. Shift all coordinates by subtracting border_start
pub fn bit_locations(layout: &Layout) -> Vec<BitLocation> {
    let size = layout.grid_size;
    let mut locations: Vec<(usize, usize)> = Vec::with_capacity(layout.nbits);

    // First quadrant: top-left triangle
    for y in 0..size / 2 {
        for x in y..size - 1 - y {
            if layout.cell(x, y) == CellType::Data {
                locations.push((x, y));
            }
        }
    }

    let step = locations.len();

    // Remaining three quadrants by successive 90-degree rotations
    // rotate90(x, y) = (size - 1 - y, x)
    while locations.len() < step * 4 {
        let idx = locations.len() - step;
        let (px, py) = locations[idx];
        locations.push((size - 1 - py, px));
    }

    // Center pixel (odd grid size only)
    if size % 2 == 1 && layout.cell(size / 2, size / 2) == CellType::Data {
        locations.push((size / 2, size / 2));
    }

    assert_eq!(locations.len(), layout.nbits);

    // Shift by border_start
    let bs = layout.border_start as i32;
    locations
        .into_iter()
        .map(|(x, y)| BitLocation {
            x: x as i32 - bs,
            y: y as i32 - bs,
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::layout::Layout;

    #[test]
    fn tag16h5_bit_locations() {
        let layout = Layout::classic(8).unwrap();
        let locs = bit_locations(&layout);
        assert_eq!(locs.len(), 16);

        // Expected from tag16h5.c (border_start=1 in Java)
        let expected: Vec<(i32, i32)> = vec![
            (1, 1), (2, 1), (3, 1), (2, 2),
            (4, 1), (4, 2), (4, 3), (3, 2),
            (4, 4), (3, 4), (2, 4), (3, 3),
            (1, 4), (1, 3), (1, 2), (2, 3),
        ];
        let actual: Vec<(i32, i32)> = locs.iter().map(|l| (l.x, l.y)).collect();
        assert_eq!(actual, expected);
    }

    #[test]
    fn tag36h11_bit_locations() {
        let layout = Layout::classic(10).unwrap();
        let locs = bit_locations(&layout);
        assert_eq!(locs.len(), 36);

        // Expected from tag36h11.c (border_start=1 in Java)
        let expected: Vec<(i32, i32)> = vec![
            (1, 1), (2, 1), (3, 1), (4, 1), (5, 1), (2, 2), (3, 2), (4, 2), (3, 3),
            (6, 1), (6, 2), (6, 3), (6, 4), (6, 5), (5, 2), (5, 3), (5, 4), (4, 3),
            (6, 6), (5, 6), (4, 6), (3, 6), (2, 6), (5, 5), (4, 5), (3, 5), (4, 4),
            (1, 6), (1, 5), (1, 4), (1, 3), (1, 2), (2, 5), (2, 4), (2, 3), (3, 4),
        ];
        let actual: Vec<(i32, i32)> = locs.iter().map(|l| (l.x, l.y)).collect();
        assert_eq!(actual, expected);
    }

    #[test]
    fn circle21h7_bit_locations() {
        let data = "xxxdddxxxxbbbbbbbxxbwwwwwbxdbwdddwbddbwdddwbddbwdddwbdxbwwwwwbxxbbbbbbbxxxxdddxxx";
        let layout = Layout::from_data_string(data).unwrap();
        let locs = bit_locations(&layout);
        assert_eq!(locs.len(), 21);

        // Expected from tagCircle21h7.c (border_start=2 in Java)
        let expected: Vec<(i32, i32)> = vec![
            (1, -2), (2, -2), (3, -2),
            (1, 1), (2, 1),
            (6, 1), (6, 2), (6, 3),
            (3, 1), (3, 2),
            (3, 6), (2, 6), (1, 6),
            (3, 3), (2, 3),
            (-2, 3), (-2, 2), (-2, 1),
            (1, 3), (1, 2),
            (2, 2),
        ];
        let actual: Vec<(i32, i32)> = locs.iter().map(|l| (l.x, l.y)).collect();
        assert_eq!(actual, expected);
    }

    #[test]
    fn standard41h12_bit_locations() {
        let data = "ddddddddddbbbbbbbddbwwwwwbddbwdddwbddbwdddwbddbwdddwbddbwwwwwbddbbbbbbbdddddddddd";
        let layout = Layout::from_data_string(data).unwrap();
        let locs = bit_locations(&layout);
        assert_eq!(locs.len(), 41);

        // Expected from tagStandard41h12.c (border_start=2 in Java)
        let expected: Vec<(i32, i32)> = vec![
            (-2, -2), (-1, -2), (0, -2), (1, -2), (2, -2), (3, -2), (4, -2), (5, -2),
            (1, 1), (2, 1),
            (6, -2), (6, -1), (6, 0), (6, 1), (6, 2), (6, 3), (6, 4), (6, 5),
            (3, 1), (3, 2),
            (6, 6), (5, 6), (4, 6), (3, 6), (2, 6), (1, 6), (0, 6), (-1, 6),
            (3, 3), (2, 3),
            (-2, 6), (-2, 5), (-2, 4), (-2, 3), (-2, 2), (-2, 1), (-2, 0), (-2, -1),
            (1, 3), (1, 2),
            (2, 2),
        ];
        let actual: Vec<(i32, i32)> = locs.iter().map(|l| (l.x, l.y)).collect();
        assert_eq!(actual, expected);
    }
}
