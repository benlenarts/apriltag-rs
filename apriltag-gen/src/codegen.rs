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

/// Hybrid code set: flat scan for small N, BK-tree for large N.
///
/// For small sets (≤ BK_TREE_THRESHOLD), a flat `Vec<u64>` with sequential
/// scan is faster due to cache-friendly access and no tree overhead.
/// Once the set grows large enough for triangle-inequality pruning to
/// outweigh the tree overhead, codes are migrated into a BK-tree.
struct CodeSet {
    /// Flat storage used when len ≤ BK_TREE_THRESHOLD.
    flat: Vec<u64>,
    /// BK-tree used when len > BK_TREE_THRESHOLD.
    tree: Vec<BkNode>,
}

/// Crossover point where BK-tree pruning beats flat scan.
/// Empirically tuned: flat scan wins for small families (circle21h7 = 152 rotcodes),
/// BK-tree wins for large families (standard52h13 = ~780K rotcodes).
const BK_TREE_THRESHOLD: usize = 512;

struct BkNode {
    code: u64,
    /// (hamming_distance_to_parent, arena_index)
    children: SmallVec<[(u32, u32); 4]>,
}

impl CodeSet {
    fn new() -> Self {
        CodeSet {
            flat: Vec::new(),
            tree: Vec::new(),
        }
    }

    fn insert(&mut self, code: u64) {
        if self.tree.is_empty() {
            self.flat.push(code);
            // Migrate to BK-tree when we cross the threshold
            if self.flat.len() > BK_TREE_THRESHOLD {
                for &c in &self.flat {
                    Self::bk_insert(&mut self.tree, c);
                }
                self.flat.clear();
                self.flat.shrink_to_fit();
            }
        } else {
            Self::bk_insert(&mut self.tree, code);
        }
    }

    fn bk_insert(tree: &mut Vec<BkNode>, code: u64) {
        if tree.is_empty() {
            tree.push(BkNode {
                code,
                children: SmallVec::new(),
            });
            return;
        }

        let mut idx = 0;
        loop {
            let d = hamming_distance(tree[idx].code, code);
            if let Some(&(_, child_idx)) = tree[idx].children.iter().find(|(dist, _)| *dist == d) {
                idx = child_idx as usize;
                continue;
            }
            let new_idx = tree.len() as u32;
            tree.push(BkNode {
                code,
                children: SmallVec::new(),
            });
            tree[idx].children.push((d, new_idx));
            return;
        }
    }

    /// Returns `true` if any stored code has Hamming distance < `threshold` from `query`.
    fn has_any_closer_than(&self, query: u64, threshold: u32) -> bool {
        if !self.tree.is_empty() {
            return Self::bk_query(&self.tree, query, threshold);
        }
        // Flat scan — branch-free loop, auto-vectorizable
        self.flat
            .iter()
            .any(|&c| (c ^ query).count_ones() < threshold)
    }

    fn bk_query(tree: &[BkNode], query: u64, threshold: u32) -> bool {
        let mut stack: SmallVec<[u32; 64]> = SmallVec::new();
        stack.push(0);
        while let Some(idx) = stack.pop() {
            let node = &tree[idx as usize];
            let d = hamming_distance(node.code, query);
            if d < threshold {
                return true;
            }
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
/// Uses bitmask + popcount instead of per-shift loops. For each data bit,
/// we precompute `net = black_adj - white_adj` and group bits by net value
/// into bitmasks. The energy from fixed-data adjacencies reduces to a few
/// `count_ones()` calls (hardware popcount) instead of iterating shift lists.
struct ComplexityGrid {
    /// Constant energy: Fixed-Fixed pairs + total white adjacencies.
    /// `base_energy + total_white_adj` gives energy assuming all data bits are 0.
    constant_energy: i32,
    /// Bitmasks grouped by net coefficient (black_adj - white_adj).
    /// `coeff_masks[i]` holds the mask for bits where net == `coeff_values[i]`.
    coeff_masks: SmallVec<[(i32, u64); 4]>,
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
        let mut white_adj = vec![0i32; nbits]; // per-bit white adjacency count
        let mut black_adj = vec![0i32; nbits]; // per-bit black adjacency count
        let mut data_pair_shifts: SmallVec<[(u32, u32); 32]> = SmallVec::new();

        // Process all adjacent pairs (horizontal and vertical)
        for y in 0..size {
            for x in 0..size {
                let a = cells[y * size + x];

                // Check both horizontal (x+1, y) and vertical (x, y+1) neighbors
                let neighbors = [
                    (x + 1 < size).then(|| cells[y * size + x + 1]),
                    (y + 1 < size).then(|| cells[(y + 1) * size + x]),
                ];

                for b in neighbors.into_iter().flatten() {
                    match (a, b) {
                        (CellKind::Fixed(va), CellKind::Fixed(vb)) => {
                            if va != vb {
                                base_energy += 1;
                            }
                        }
                        (CellKind::Fixed(v), CellKind::Data(i))
                        | (CellKind::Data(i), CellKind::Fixed(v)) => {
                            if v {
                                white_adj[i] += 1;
                            } else {
                                black_adj[i] += 1;
                            }
                        }
                        (CellKind::Data(i), CellKind::Data(j)) => {
                            data_pair_shifts.push(((nbits - 1 - i) as u32, (nbits - 1 - j) as u32));
                        }
                        _ => {}
                    }
                }
            }
        }

        // Build bitmasks grouped by net coefficient (black_adj - white_adj).
        // Energy from fixed-data pairs = sum_white + sum_bits(net_i * bit_i),
        // where net_i = black_adj[i] - white_adj[i].
        let mut total_white = 0i32;
        let mut coeff_map: SmallVec<[(i32, u64); 4]> = SmallVec::new();
        for i in 0..nbits {
            total_white += white_adj[i];
            let net = black_adj[i] - white_adj[i];
            if net == 0 {
                continue;
            }
            let shift = (nbits - 1 - i) as u32;
            if let Some(entry) = coeff_map.iter_mut().find(|(c, _)| *c == net) {
                entry.1 |= 1u64 << shift;
            } else {
                coeff_map.push((net, 1u64 << shift));
            }
        }

        ComplexityGrid {
            constant_energy: base_energy + total_white,
            coeff_masks: coeff_map,
            data_pair_shifts,
            threshold: 2 * area,
        }
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
    let mut rotcodes = CodeSet::new();

    // Pre-build grid once — avoids allocating a pixel grid per candidate
    let grid = ComplexityGrid::from_layout(layout);

    // Report every 1M candidates (or every candidate for tiny families).
    let report_interval = 1_000_000u64.min(total).max(1);

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
/// Fixed-data energy uses bitmask + `count_ones()` (hardware popcount)
/// instead of per-shift loops. Each distinct net coefficient
/// `(black_adj - white_adj)` gets one popcount call, typically 1-2 total.
fn is_complex_enough(grid: &ComplexityGrid, code: u64) -> bool {
    let mut energy = grid.constant_energy;

    // Fixed ↔ data: popcount per net-coefficient group
    for &(coeff, mask) in &grid.coeff_masks {
        energy += coeff * (code & mask).count_ones() as i32;
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
        let tree = CodeSet::new();
        assert!(!tree.has_any_closer_than(0, 1));
        assert!(!tree.has_any_closer_than(0xFF, 5));
    }

    #[test]
    fn bk_tree_exact_match() {
        let mut tree = CodeSet::new();
        tree.insert(0b1010);
        // Exact match: distance 0, which is < any threshold >= 1
        assert!(tree.has_any_closer_than(0b1010, 1));
        // Distance 0 is not < 0... but threshold=0 means "closer than 0" which nothing can be
        // (threshold of 0 is a degenerate case but let's not test that — min_hamming is always >= 1)
    }

    #[test]
    fn bk_tree_near_match() {
        let mut tree = CodeSet::new();
        tree.insert(0b1010);
        // Distance 1 from 0b1011 — should be found with threshold 2 but not threshold 1
        assert!(tree.has_any_closer_than(0b1011, 2));
        assert!(!tree.has_any_closer_than(0b1011, 1));
    }

    #[test]
    fn codeset_matches_naive_scan_flat() {
        // Property test: flat path gives same answers as a naive linear scan
        let codes: Vec<u64> = vec![
            0x157863, 0x05E9B9, 0x1A831E, 0x0B4C74, 0x0DC6D2, 0x1F3F28, 0x00A87E, 0x12C195,
        ];
        let mut set = CodeSet::new();
        for &c in &codes {
            set.insert(c);
        }
        assert!(set.tree.is_empty(), "should use flat path for 8 codes");

        let queries: Vec<u64> = vec![0x157863, 0x000000, 0x1FFFFF, 0x0AAAAA, 0x155555, 0x1EC1E3];
        for threshold in 1..=12 {
            for &q in &queries {
                let naive = codes.iter().any(|&c| hamming_distance(c, q) < threshold);
                let result = set.has_any_closer_than(q, threshold);
                assert_eq!(
                    result, naive,
                    "mismatch for query={:#x} threshold={}: got={} naive={}",
                    q, threshold, result, naive
                );
            }
        }
    }

    #[test]
    fn codeset_matches_naive_scan_bktree() {
        // Property test: BK-tree path gives same answers as a naive linear scan.
        // Insert enough codes to cross BK_TREE_THRESHOLD.
        let mut codes = Vec::new();
        let mut rng = 0x12345678u64;
        for _ in 0..BK_TREE_THRESHOLD + 100 {
            rng = rng.wrapping_mul(6364136223846793005).wrapping_add(1);
            codes.push(rng & 0x1FFFFF); // 21-bit codes
        }
        let mut set = CodeSet::new();
        for &c in &codes {
            set.insert(c);
        }
        assert!(!set.tree.is_empty(), "should use BK-tree path");

        let queries: Vec<u64> = vec![codes[0], codes[100], 0x000000, 0x1FFFFF, 0x0AAAAA];
        for threshold in [1, 3, 5, 7, 10] {
            for &q in &queries {
                let naive = codes.iter().any(|&c| hamming_distance(c, q) < threshold);
                let result = set.has_any_closer_than(q, threshold);
                assert_eq!(
                    result, naive,
                    "mismatch for query={:#x} threshold={}: got={} naive={}",
                    q, threshold, result, naive
                );
            }
        }
    }

    #[test]
    fn codeset_duplicate_distance_children() {
        // Insert codes that have the same Hamming distance from each other
        // to exercise the BK-tree child-lookup path.
        // Force BK-tree by inserting > BK_TREE_THRESHOLD codes, then check
        // specific distance queries.
        let mut set = CodeSet::new();
        // Fill with enough to trigger BK-tree migration
        for i in 0..BK_TREE_THRESHOLD + 10 {
            set.insert(i as u64 * 7919); // spread-out codes
        }
        assert!(!set.tree.is_empty());

        // Insert known codes and verify lookup
        set.insert(0b0000);
        set.insert(0b0001);
        set.insert(0b0010);
        set.insert(0b0100);
        set.insert(0b1000);

        assert!(set.has_any_closer_than(0b0001, 1));
        assert!(set.has_any_closer_than(0b0010, 1));
        assert!(set.has_any_closer_than(0b0100, 1));
        assert!(set.has_any_closer_than(0b1000, 1));
        assert!(set.has_any_closer_than(0b0011, 2));
    }

    #[test]
    #[ignore] // run with: cargo test -p apriltag-gen --release -- bench_standard41h12 --ignored --nocapture
    fn bench_standard41h12() {
        let layout = Layout::standard(9).unwrap();
        let start = std::time::Instant::now();
        let codes = generate_with_progress(&layout, 12, 10, |iter, total, found| {
            let pct = iter as f64 / total as f64 * 100.0;
            let d = ((total as f64).log10() - 8.0).ceil().max(1.0) as usize;
            println!(
                "{:>width$.prec$}% ({}/{}) — {} codes found, elapsed {:.2?}",
                pct,
                iter,
                total,
                found,
                start.elapsed(),
                width = d + 4,
                prec = d,
            );
        });
        let elapsed = start.elapsed();
        println!(
            "tagStandard41h12: {} codes in {:.2?} ({} candidates)",
            codes.len(),
            elapsed,
            1u64 << 41
        );
    }

    #[test]
    #[ignore] // run with: cargo test -p apriltag-gen --release -- bench_inner_scan_scaling --ignored --nocapture
    fn bench_inner_scan_scaling() {
        use std::time::Instant;

        let threshold = 12u32;
        let nbits = 41;
        let mask = (1u64 << nbits) - 1;
        let num_queries = 100_000;

        for &n in &[100, 500, 1000, 5000, 10_000, 50_000] {
            // Generate N codes deterministically
            let mut rng = 0xDEADBEEFu64;
            let codes: Vec<u64> = (0..n)
                .map(|_| {
                    rng = rng.wrapping_mul(6364136223846793005).wrapping_add(1);
                    rng & mask
                })
                .collect();

            // Generate M queries
            let queries: Vec<u64> = (0..num_queries)
                .map(|_| {
                    rng = rng.wrapping_mul(6364136223846793005).wrapping_add(1);
                    rng & mask
                })
                .collect();

            // Populate CodeSet
            let mut set = CodeSet::new();
            for &c in &codes {
                set.insert(c);
            }

            // Benchmark CodeSet queries
            let start = Instant::now();
            let mut hits_set = 0u32;
            for &q in &queries {
                if set.has_any_closer_than(q, threshold) {
                    hits_set += 1;
                }
            }
            let set_time = start.elapsed();

            // Benchmark naive Vec scan
            let start = Instant::now();
            let mut hits_vec = 0u32;
            for &q in &queries {
                if codes.iter().any(|&c| (c ^ q).count_ones() < threshold) {
                    hits_vec += 1;
                }
            }
            let vec_time = start.elapsed();

            assert_eq!(hits_set, hits_vec);
            let speedup = vec_time.as_secs_f64() / set_time.as_secs_f64();
            println!(
                "N={:>6}: CodeSet={:.2?}  Vec={:.2?}  speedup={:.2}x  hits={}/{}",
                n, set_time, vec_time, speedup, hits_set, num_queries
            );
        }
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
