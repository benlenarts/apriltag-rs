//! Tag family code generation — greedy lexicode search.
//!
//! Reproduces the algorithm from `TagFamilyGenerator.java`, including
//! the Java LCG pseudo-random iteration order and Ising energy complexity check.
//!
//! This implements Era 2 code generation (AprilTag 3) for Standard, Circle,
//! and Custom tag families. Classic families use `upgrade.rs` instead.

use apriltag::hamming::{hamming_distance_at_least, rotate90};
use apriltag::layout::Layout;
use apriltag::render;
use apriltag::types::Pixel;

const PRIME: u64 = 982_451_653;

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

    let mut v = v0;
    for _iter in 0..total {
        v = v.wrapping_add(PRIME) & mask;

        if !is_complex_enough(layout, v) {
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

/// Check if a rendered code has enough visual complexity (Ising energy).
///
/// Counts 4-connected black/white transitions and requires
/// `energy >= 0.3333 * max_energy` where `max_energy = 2 * area`.
fn is_complex_enough(layout: &Layout, code: u64) -> bool {
    let tag = render::render(layout, code);
    let size = tag.grid_size;
    let mut energy = 0i32;

    // Horizontal transitions
    for y in 0..size {
        for x in 0..size - 1 {
            if is_bw_transition(tag.pixel(x, y), tag.pixel(x + 1, y)) {
                energy += 1;
            }
        }
    }

    // Vertical transitions
    for x in 0..size {
        for y in 0..size - 1 {
            if is_bw_transition(tag.pixel(x, y), tag.pixel(x, y + 1)) {
                energy += 1;
            }
        }
    }

    // Area = non-transparent pixels
    let area: i32 = tag
        .pixels
        .iter()
        .filter(|&&p| p == Pixel::Black || p == Pixel::White)
        .count() as i32;

    let max_energy = 2 * area;
    energy as f64 >= 0.3333 * max_energy as f64
}

fn is_bw_transition(a: Pixel, b: Pixel) -> bool {
    matches!(
        (a, b),
        (Pixel::Black, Pixel::White) | (Pixel::White, Pixel::Black)
    )
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
    #[ignore] // ~56s in debug — run with `cargo test -- --ignored`
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
