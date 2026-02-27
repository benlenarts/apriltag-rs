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
use smallvec::SmallVec;

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

/// Pre-computed adjacency data for fast Ising energy complexity checks.
///
/// Instead of scanning the full grid on every candidate, we precompute
/// all adjacent cell pairs during construction and store compact lists
/// of shift amounts. This eliminates the nested grid loop, all `resolve()`
/// calls, and skip-cell processing from the hot path.
struct ComplexityGrid {
    /// Constant energy from Fixed-Fixed adjacent pairs.
    base_energy: i32,
    /// Shift amounts for data bits adjacent to white fixed cells.
    /// A data bit adjacent to N white cells appears N times.
    white_data_shifts: SmallVec<[u32; 64]>,
    /// Shift amounts for data bits adjacent to black fixed cells.
    black_data_shifts: SmallVec<[u32; 64]>,
    /// (shift_a, shift_b) for each Data-Data adjacency.
    data_pair_shifts: SmallVec<[(u32, u32); 32]>,
    /// Precomputed threshold: `2 * area` for integer comparison
    /// (`3 * energy >= threshold` ≡ `energy >= 0.3333 * 2 * area`).
    threshold: i32,
}

impl ComplexityGrid {
    fn from_layout(layout: &Layout) -> Self {
        let size = layout.grid_size;
        let nbits = layout.nbits;
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

        // Precompute adjacency pairs
        let mut base_energy = 0i32;
        let mut white_data_shifts: SmallVec<[u32; 64]> = SmallVec::new();
        let mut black_data_shifts: SmallVec<[u32; 64]> = SmallVec::new();
        let mut data_pair_shifts: SmallVec<[(u32, u32); 32]> = SmallVec::new();

        // Process all adjacent pairs (horizontal and vertical)
        for y in 0..size {
            for x in 0..size {
                let a = cells[y * size + x];

                // Horizontal neighbor (x+1, y)
                if x + 1 < size {
                    let b = cells[y * size + x + 1];
                    classify_pair(a, b, nbits, &mut base_energy,
                        &mut white_data_shifts, &mut black_data_shifts,
                        &mut data_pair_shifts);
                }

                // Vertical neighbor (x, y+1)
                if y + 1 < size {
                    let b = cells[(y + 1) * size + x];
                    classify_pair(a, b, nbits, &mut base_energy,
                        &mut white_data_shifts, &mut black_data_shifts,
                        &mut data_pair_shifts);
                }
            }
        }

        ComplexityGrid {
            base_energy,
            white_data_shifts,
            black_data_shifts,
            data_pair_shifts,
            threshold: 2 * area,
        }
    }
}

/// Classify an adjacent cell pair and update the precomputed lists.
fn classify_pair(
    a: CellKind,
    b: CellKind,
    nbits: usize,
    base_energy: &mut i32,
    white_data_shifts: &mut SmallVec<[u32; 64]>,
    black_data_shifts: &mut SmallVec<[u32; 64]>,
    data_pair_shifts: &mut SmallVec<[(u32, u32); 32]>,
) {
    match (a, b) {
        (CellKind::Fixed(va), CellKind::Fixed(vb)) => {
            if va != vb {
                *base_energy += 1;
            }
        }
        (CellKind::Fixed(v), CellKind::Data(i)) | (CellKind::Data(i), CellKind::Fixed(v)) => {
            let shift = (nbits - 1 - i) as u32;
            if v {
                // White fixed: transition when data bit is 0
                white_data_shifts.push(shift);
            } else {
                // Black fixed: transition when data bit is 1
                black_data_shifts.push(shift);
            }
        }
        (CellKind::Data(i), CellKind::Data(j)) => {
            data_pair_shifts.push(((nbits - 1 - i) as u32, (nbits - 1 - j) as u32));
        }
        _ => {} // Skip-anything: no energy contribution
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
/// Uses precomputed adjacency pair lists for speed — no grid scan,
/// no match dispatch, no skip-cell overhead.
fn is_complex_enough(grid: &ComplexityGrid, code: u64) -> bool {
    let mut energy = grid.base_energy;

    // Fixed-white ↔ data: transition when data bit is 0 (black)
    for &shift in &grid.white_data_shifts {
        if (code >> shift) & 1 == 0 {
            energy += 1;
        }
    }

    // Fixed-black ↔ data: transition when data bit is 1 (white)
    for &shift in &grid.black_data_shifts {
        if (code >> shift) & 1 != 0 {
            energy += 1;
        }
    }

    // Data ↔ data: transition when bits differ
    for &(s1, s2) in &grid.data_pair_shifts {
        if ((code >> s1) ^ (code >> s2)) & 1 != 0 {
            energy += 1;
        }
    }

    3 * energy >= grid.threshold
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
