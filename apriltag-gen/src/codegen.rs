//! Tag family code generation — greedy lexicode search.
//!
//! Reproduces the algorithm from `TagFamilyGenerator.java`, including
//! the Java LCG pseudo-random iteration order and Ising energy complexity check.
//!
//! This implements Era 2 code generation (AprilTag 3) for Standard, Circle,
//! and Custom tag families. Classic families use `upgrade.rs` instead.

use apriltag::bits;
use apriltag::hamming::{hamming_distance_at_least, rotate90};
use apriltag::layout::Layout;
use apriltag::types::CellType;

const PRIME: u64 = 982_451_653;

/// What a grid cell resolves to for complexity checking.
#[derive(Clone, Copy)]
enum CellKind {
    /// A fixed black or white pixel (true = white, false = black).
    Fixed(bool),
    /// A data bit, identified by its bit index (0 = MSB).
    Data(usize),
    /// Transparent / ignored — not counted.
    Skip,
}

/// Pre-computed grid for fast Ising energy complexity checks.
///
/// Built once per layout from `bit_locations()`. Each cell is either a
/// fixed pixel, a data bit index, or transparent. The `area` (number of
/// non-transparent cells) is also pre-computed since it's constant.
struct ComplexityGrid {
    cells: Vec<CellKind>,
    size: usize,
    nbits: usize,
    /// Number of non-transparent pixels (constant for a given layout).
    area: i32,
}

impl ComplexityGrid {
    fn from_layout(layout: &Layout) -> Self {
        let size = layout.grid_size;
        let mut cells = vec![CellKind::Skip; size * size];

        // Fill fixed cells from the layout
        for y in 0..size {
            for x in 0..size {
                match layout.cell(x, y) {
                    CellType::Black => cells[y * size + x] = CellKind::Fixed(false),
                    CellType::White => cells[y * size + x] = CellKind::Fixed(true),
                    CellType::Data | CellType::Ignored => {}
                }
            }
        }

        // Fill data cells using bit_locations (which gives the render order)
        let bs = layout.border_start as i32;
        for (bit_idx, loc) in bits::bit_locations(layout).iter().enumerate() {
            let gx = (loc.x + bs) as usize;
            let gy = (loc.y + bs) as usize;
            cells[gy * size + gx] = CellKind::Data(bit_idx);
        }

        let area = cells
            .iter()
            .filter(|c| !matches!(c, CellKind::Skip))
            .count() as i32;

        ComplexityGrid {
            cells,
            size,
            nbits: layout.nbits,
            area,
        }
    }

    /// Resolve a cell to a pixel value (true = white, false = black) for a given code.
    fn resolve(&self, idx: usize, code: u64) -> Option<bool> {
        match self.cells[idx] {
            CellKind::Fixed(v) => Some(v),
            CellKind::Data(i) => Some((code >> (self.nbits - 1 - i)) & 1 != 0),
            CellKind::Skip => None,
        }
    }
}

/// Generate tag family codes using the greedy lexicode search.
///
/// `min_complexity` is the per-family seed parameter (from TOML config).
/// The LCG seed is `nbits * 10000 + min_hamming * 100 + min_complexity`.
pub fn generate(layout: &Layout, min_hamming: u32, min_complexity: u32) -> Vec<u64> {
    let nbits = layout.nbits as u32;
    let mask = if nbits >= 64 {
        u64::MAX
    } else {
        (1u64 << nbits) - 1
    };

    // Compute V0 using Java Random LCG
    let seed = nbits as i64 * 10000 + min_hamming as i64 * 100 + min_complexity as i64;
    let v0 = java_random_next_long(seed) as u64 & mask;

    let total = 1u64 << nbits;
    let mut codelist: Vec<u64> = Vec::new();
    let mut rotcodes: Vec<u64> = Vec::new();

    // Pre-build grid once — avoids allocating a pixel grid per candidate
    let grid = ComplexityGrid::from_layout(layout);

    let mut v = v0;
    for _iter in 0..total {
        v = v.wrapping_add(PRIME) & mask;

        if !is_complex_enough(&grid, v) {
            continue;
        }

        let rv1 = rotate90(v, nbits);
        let rv2 = rotate90(rv1, nbits);
        let rv3 = rotate90(rv2, nbits);

        // Self-rotation distance check
        if !hamming_distance_at_least(v, rv1, min_hamming)
            || !hamming_distance_at_least(v, rv2, min_hamming)
            || !hamming_distance_at_least(v, rv3, min_hamming)
            || !hamming_distance_at_least(rv1, rv2, min_hamming)
            || !hamming_distance_at_least(rv1, rv3, min_hamming)
            || !hamming_distance_at_least(rv2, rv3, min_hamming)
        {
            continue;
        }

        // Distance from all previously accepted codes (and their rotations)
        let mut ok = true;
        for &w in &rotcodes {
            if !hamming_distance_at_least(v, w, min_hamming) {
                ok = false;
                break;
            }
        }
        if !ok {
            continue;
        }

        codelist.push(v);
        rotcodes.push(v);
        rotcodes.push(rv1);
        rotcodes.push(rv2);
        rotcodes.push(rv3);
    }

    codelist
}

/// Check if a code has enough visual complexity (Ising energy).
///
/// Counts 4-connected black/white transitions and requires
/// `energy >= 0.3333 * max_energy` where `max_energy = 2 * area`.
///
/// Uses the pre-built `ComplexityGrid` to resolve pixels via bit shifts
/// instead of allocating a full rendered pixel grid per candidate.
fn is_complex_enough(grid: &ComplexityGrid, code: u64) -> bool {
    let size = grid.size;
    let mut energy = 0i32;

    for y in 0..size {
        for x in 0..size {
            let idx = y * size + x;
            let pixel = grid.resolve(idx, code);

            // Horizontal transition (compare with right neighbor)
            if x + 1 < size {
                if let (Some(a), Some(b)) = (pixel, grid.resolve(idx + 1, code)) {
                    if a != b {
                        energy += 1;
                    }
                }
            }

            // Vertical transition (compare with bottom neighbor)
            if y + 1 < size {
                if let (Some(a), Some(b)) = (pixel, grid.resolve(idx + size, code)) {
                    if a != b {
                        energy += 1;
                    }
                }
            }
        }
    }

    let max_energy = 2 * grid.area;
    energy as f64 >= 0.3333 * max_energy as f64
}

/// Reproduce Java's `new Random(seed).nextLong()`.
///
/// Java's Random uses a 48-bit LCG: `state = state * 0x5DEECE66D + 0xB`.
/// `nextLong()` calls `next(32)` twice and combines the results.
fn java_random_next_long(seed: i64) -> i64 {
    let mut state = (seed ^ 0x5DEECE66D_i64) as u64 & ((1u64 << 48) - 1);

    // First call to next(32)
    state = state.wrapping_mul(0x5DEECE66D).wrapping_add(0xB) & ((1u64 << 48) - 1);
    let hi = (state >> 16) as i32;

    // Second call to next(32)
    state = state.wrapping_mul(0x5DEECE66D).wrapping_add(0xB) & ((1u64 << 48) - 1);
    let lo = (state >> 16) as i32;

    ((hi as i64) << 32).wrapping_add(lo as i64)
}

#[cfg(test)]
mod tests {
    use super::*;
    use apriltag::layout::Layout;

    #[test]
    fn java_random_deterministic() {
        // Same seed always produces the same output.
        let v1 = java_random_next_long(210710);
        let v2 = java_random_next_long(210710);
        assert_eq!(v1, v2);
    }

    #[test]
    fn generate_circle21h7_matches_reference() {
        // tagCircle21h7: 21 bits, min_hamming=7, min_complexity=10
        let data =
            "xxxdddxxxxbbbbbbbxxbwwwwwbxdbwdddwbddbwdddwbddbwdddwbdxbwwwwwbxxbbbbbbbxxxxdddxxx";
        let layout = Layout::from_data_string(data).unwrap();
        let codes = generate(&layout, 7, 10);

        assert_eq!(codes.len(), 38, "expected 38 codes for tagCircle21h7");
        assert_eq!(codes[0], 0x157863);
        assert_eq!(codes[37], 0x1ec1e3);

        // Full verification against built-in family
        let family = crate::family::tag_circle21h7();
        assert_eq!(codes, family.codes);
    }
}
