/// Rotate a code word 90 degrees (one quadrant shift).
///
/// Matches the Java `TagFamily.rotate90()`:
/// - When `nbits % 4 == 0`: left-rotate all bits by `nbits/4` positions
/// - When `nbits % 4 == 1`: preserve the center bit (LSB), rotate remaining bits
pub fn rotate90(w: u64, nbits: u32) -> u64 {
    let (p, l): (u64, u64) = if nbits % 4 == 1 {
        (nbits as u64 - 1, 1)
    } else {
        (nbits as u64, 0)
    };

    let result = ((w >> l) << (p / 4 + l)) | ((w >> (3 * p / 4 + l)) << l) | (w & l);

    result & ((1u64 << nbits) - 1)
}

/// Compute the Hamming distance between two code words.
pub fn hamming_distance(a: u64, b: u64) -> u32 {
    (a ^ b).count_ones()
}

/// Check whether the Hamming distance between two code words is at least `min_val`.
///
/// Uses early-exit by checking 16-bit chunks, matching the Java reference.
pub fn hamming_distance_at_least(a: u64, b: u64, min_val: u32) -> bool {
    let mut w = a ^ b;
    let mut count = 0u32;
    while w != 0 {
        count += (w & 0xFFFF).count_ones();
        if count >= min_val {
            return true;
        }
        w >>= 16;
    }
    false
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rotate90_four_times_returns_original_36bits() {
        let code: u64 = 0xd7e00984b; // tag36h11 code 0
        let mut v = code;
        for _ in 0..4 {
            v = rotate90(v, 36);
        }
        assert_eq!(v, code);
    }

    #[test]
    fn rotate90_four_times_returns_original_16bits() {
        let code: u64 = 0x27c8; // tag16h5 code 0
        let mut v = code;
        for _ in 0..4 {
            v = rotate90(v, 16);
        }
        assert_eq!(v, code);
    }

    #[test]
    fn rotate90_four_times_returns_original_41bits() {
        // 41 % 4 == 1, has center bit
        let code: u64 = 0x1bd8a64ad10; // tagStandard41h12 code 0
        let mut v = code;
        for _ in 0..4 {
            v = rotate90(v, 41);
        }
        assert_eq!(v, code);
    }

    #[test]
    fn rotate90_four_times_returns_original_21bits() {
        // 21 % 4 == 1, has center bit
        let code: u64 = 0x157863; // tagCircle21h7 code 0
        let mut v = code;
        for _ in 0..4 {
            v = rotate90(v, 21);
        }
        assert_eq!(v, code);
    }

    #[test]
    fn rotate90_four_times_returns_original_25bits() {
        // 25 % 4 == 1, has center bit
        let code: u64 = 0x156f1f4; // tag25h9 code 0
        let mut v = code;
        for _ in 0..4 {
            v = rotate90(v, 25);
        }
        assert_eq!(v, code);
    }

    #[test]
    fn rotate90_four_times_returns_original_49bits() {
        // 49 % 4 == 1, has center bit
        let code: u64 = 0xc6c921d8614a; // tagCircle49h12 code 0
        let mut v = code;
        for _ in 0..4 {
            v = rotate90(v, 49);
        }
        assert_eq!(v, code);
    }

    #[test]
    fn rotate90_produces_different_intermediate() {
        let code: u64 = 0xd7e00984b;
        let r1 = rotate90(code, 36);
        assert_ne!(r1, code);
        let r2 = rotate90(r1, 36);
        assert_ne!(r2, code);
        assert_ne!(r2, r1);
    }

    #[test]
    fn hamming_distance_identical() {
        assert_eq!(hamming_distance(0x1234, 0x1234), 0);
    }

    #[test]
    fn hamming_distance_one_bit() {
        assert_eq!(hamming_distance(0b1010, 0b1011), 1);
    }

    #[test]
    fn hamming_distance_at_least_true() {
        assert!(hamming_distance_at_least(0xFF, 0x00, 8));
    }

    #[test]
    fn hamming_distance_at_least_false() {
        assert!(!hamming_distance_at_least(0xFF, 0xFE, 2));
    }

    #[test]
    fn hamming_distance_at_least_exact() {
        // Distance is exactly 8
        assert!(hamming_distance_at_least(0xFF, 0x00, 8));
        assert!(!hamming_distance_at_least(0xFF, 0x00, 9));
    }
}
