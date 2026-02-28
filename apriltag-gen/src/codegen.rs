//! Tag family code generation — greedy lexicode search.
//!
//! Reproduces the algorithm from `TagFamilyGenerator.java`, including
//! the Java LCG pseudo-random iteration order and Ising energy complexity check.
//!
//! This implements Era 2 code generation (AprilTag 3) for Standard, Circle,
//! and Custom tag families. Classic families use `upgrade.rs` instead.

use apriltag::bits;
use apriltag::hamming::{hamming_distance, hamming_distance_at_least, rotate90};
use apriltag::layout::Layout;
use apriltag::types::CellType;
use smallvec::SmallVec;

/// BK-tree (Burkhard-Keller tree) for fast Hamming-distance range queries.
///
/// Hand-rolled rather than using a crate because the available options
/// (`bk-tree` pulls in `triple_accel` which doesn't compile for wasm32;
/// `bktree` pulls in `num` which is heavier than warranted for ~40 lines
/// of logic).
struct BkTree {
    nodes: Vec<BkNode>,
}

struct BkNode {
    code: u64,
    /// (hamming_distance_to_parent, arena_index)
    children: SmallVec<[(u32, u32); 4]>,
}

impl BkTree {
    fn new() -> Self {
        BkTree { nodes: Vec::new() }
    }

    fn insert(&mut self, code: u64) {
        if self.nodes.is_empty() {
            self.nodes.push(BkNode {
                code,
                children: SmallVec::new(),
            });
            return;
        }

        let mut idx = 0;
        loop {
            let d = hamming_distance(self.nodes[idx].code, code);
            // Look for existing child at this distance
            if let Some(&(_, child_idx)) =
                self.nodes[idx].children.iter().find(|(dist, _)| *dist == d)
            {
                idx = child_idx as usize;
                continue;
            }
            // No child at this distance — add new node
            let new_idx = self.nodes.len() as u32;
            self.nodes.push(BkNode {
                code,
                children: SmallVec::new(),
            });
            self.nodes[idx].children.push((d, new_idx));
            return;
        }
    }

    /// Returns `true` if any stored code has Hamming distance < `threshold` from `query`.
    fn has_any_closer_than(&self, query: u64, threshold: u32) -> bool {
        if self.nodes.is_empty() {
            return false;
        }
        let mut stack: SmallVec<[u32; 64]> = SmallVec::new();
        stack.push(0);
        while let Some(idx) = stack.pop() {
            let node = &self.nodes[idx as usize];
            let d = hamming_distance(node.code, query);
            if d < threshold {
                return true;
            }
            // Triangle inequality: only visit children where
            // |child_dist - d| < threshold, i.e.
            // child_dist in (d - threshold + 1 ..= d + threshold - 1)
            let lo = d.saturating_sub(threshold - 1);
            let hi = d + threshold - 1;
            for &(child_dist, child_idx) in &node.children {
                if child_dist >= lo && child_dist <= hi {
                    stack.push(child_idx);
                }
            }
        }
        false
    }
}

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
                    classify_pair(
                        a,
                        b,
                        nbits,
                        &mut base_energy,
                        &mut white_data_shifts,
                        &mut black_data_shifts,
                        &mut data_pair_shifts,
                    );
                }

                // Vertical neighbor (x, y+1)
                if y + 1 < size {
                    let b = cells[(y + 1) * size + x];
                    classify_pair(
                        a,
                        b,
                        nbits,
                        &mut base_energy,
                        &mut white_data_shifts,
                        &mut black_data_shifts,
                        &mut data_pair_shifts,
                    );
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
    generate_with_progress(layout, min_hamming, min_complexity, |_, _, _| {})
}

/// Generate tag family codes with progress reporting.
///
/// Same as [`generate`], but calls `on_progress(iter, total, codes_found)` periodically
/// during the search. The callback interval scales with the search space size.
pub fn generate_with_progress(
    layout: &Layout,
    min_hamming: u32,
    min_complexity: u32,
    mut on_progress: impl FnMut(u64, u64, usize),
) -> Vec<u64> {
    let nbits = layout.nbits as u32;
    let mask = (1u64 << nbits) - 1;

    // Compute V0 using Java Random LCG
    let seed = nbits as i64 * 10000 + min_hamming as i64 * 100 + min_complexity as i64;
    let v0 = java_random_next_long(seed) as u64 & mask;

    let total = 1u64 << nbits;
    let mut codelist: Vec<u64> = Vec::new();
    let mut rotcodes = BkTree::new();

    // Pre-build grid once — avoids allocating a pixel grid per candidate
    let grid = ComplexityGrid::from_layout(layout);

    // Report every ~1M iterations, but at least every 1024 for tiny families
    let report_interval = (total / 1000).clamp(1024, 1_000_000);

    let mut v = v0;
    for iter in 0..total {
        if iter % report_interval == 0 {
            on_progress(iter, total, codelist.len());
        }

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
        if rotcodes.has_any_closer_than(v, min_hamming) {
            continue;
        }

        codelist.push(v);
        rotcodes.insert(v);
        rotcodes.insert(rv1);
        rotcodes.insert(rv2);
        rotcodes.insert(rv3);
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
    fn bk_tree_empty_has_no_matches() {
        let tree = BkTree::new();
        assert!(!tree.has_any_closer_than(0, 1));
        assert!(!tree.has_any_closer_than(0xFF, 5));
    }

    #[test]
    fn bk_tree_exact_match() {
        let mut tree = BkTree::new();
        tree.insert(0b1010);
        // Exact match: distance 0, which is < any threshold >= 1
        assert!(tree.has_any_closer_than(0b1010, 1));
        // Distance 0 is not < 0... but threshold=0 means "closer than 0" which nothing can be
        // (threshold of 0 is a degenerate case but let's not test that — min_hamming is always >= 1)
    }

    #[test]
    fn bk_tree_near_match() {
        let mut tree = BkTree::new();
        tree.insert(0b1010);
        // Distance 1 from 0b1011 — should be found with threshold 2 but not threshold 1
        assert!(tree.has_any_closer_than(0b1011, 2));
        assert!(!tree.has_any_closer_than(0b1011, 1));
    }

    #[test]
    fn bk_tree_matches_naive_scan() {
        // Property test: BK-tree gives same answers as a linear scan
        let codes: Vec<u64> = vec![
            0x157863, 0x05E9B9, 0x1A831E, 0x0B4C74, 0x0DC6D2, 0x1F3F28, 0x00A87E, 0x12C195,
        ];
        let mut tree = BkTree::new();
        for &c in &codes {
            tree.insert(c);
        }

        let queries: Vec<u64> = vec![0x157863, 0x000000, 0x1FFFFF, 0x0AAAAA, 0x155555, 0x1EC1E3];
        for threshold in 1..=12 {
            for &q in &queries {
                let naive = codes.iter().any(|&c| hamming_distance(c, q) < threshold);
                let bk = tree.has_any_closer_than(q, threshold);
                assert_eq!(
                    bk, naive,
                    "mismatch for query={:#x} threshold={}: bk={} naive={}",
                    q, threshold, bk, naive
                );
            }
        }
    }

    #[test]
    fn bk_tree_duplicate_distance_children() {
        // Insert codes that have the same Hamming distance from the root
        // to exercise the child-lookup path
        let mut tree = BkTree::new();
        tree.insert(0b0000); // root
        tree.insert(0b0001); // distance 1 from root
        tree.insert(0b0010); // also distance 1 — goes to child of 0b0001
        tree.insert(0b0100); // also distance 1 — deeper nesting
        tree.insert(0b1000); // also distance 1

        // All four single-bit codes should be findable
        assert!(tree.has_any_closer_than(0b0001, 1));
        assert!(tree.has_any_closer_than(0b0010, 1));
        assert!(tree.has_any_closer_than(0b0100, 1));
        assert!(tree.has_any_closer_than(0b1000, 1));

        // A code at distance 2 from all stored codes should not match with threshold=1
        // 0b0011 is distance 2 from 0b0000, distance 1 from 0b0001 and 0b0010
        assert!(tree.has_any_closer_than(0b0011, 2));
        assert!(!tree.has_any_closer_than(0b1111, 1));
    }

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
