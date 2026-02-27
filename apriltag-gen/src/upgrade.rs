//! Upgrade classic tag family codes from row-major to quadrant-scanned bit ordering.
//!
//! Classic families (tag16h5, tag25h9, tag36h11) were generated with an older
//! AprilTag version that used row-major bit ordering. This module ports the
//! `upgradeCode()` function from `TagFamily.java` to remap those codes into
//! the quadrant-scanned bit ordering used by AprilTag 3.

use crate::bits::BitLocation;

/// Remap a single code from old row-major bit ordering to quadrant-scanned ordering.
///
/// `old_code` — the code in legacy row-major format
/// `bit_locations` — quadrant-scanned bit positions (from `bits::bit_locations`)
/// `data_size` — side length of the data area (sqrt of nbits): 4, 5, or 6
///
/// Port of `TagFamily.upgradeCode()` from `TagFamily.java:52-63`.
pub fn upgrade_code(old_code: u64, bit_locations: &[BitLocation], data_size: usize) -> u64 {
    let size = data_size;
    let mut code: u64 = 0;

    for loc in bit_locations {
        code <<= 1;
        let bit_idx = (size as i32 - loc.x) + (size as i32 - loc.y) * size as i32;
        if (old_code & (1u64 << bit_idx as u32)) != 0 {
            code |= 1;
        }
    }

    code
}

/// Upgrade a slice of old-format codes to quadrant-scanned ordering.
///
/// `old_codes` — codes in legacy row-major format
/// `bit_locations` — quadrant-scanned bit positions for this layout
/// `data_size` — side length of the data area (sqrt of nbits)
pub fn upgrade_codes(
    old_codes: &[u64],
    bit_locations: &[BitLocation],
    data_size: usize,
) -> Vec<u64> {
    old_codes
        .iter()
        .map(|&c| upgrade_code(c, bit_locations, data_size))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bits;
    use crate::layout::Layout;

    #[test]
    fn upgrade_tag16h5_matches_c_reference() {
        let layout = Layout::classic(8).unwrap();
        let locs = bits::bit_locations(&layout);
        let data_size = 4; // sqrt(16)

        // Old codes from Tag16h5.java (row-major bit ordering)
        let old_codes: Vec<u64> = vec![
            0x231b, 0x2ea5, 0x346a, 0x45b9, 0x79a6, 0x7f6b, 0xb358, 0xe745,
            0xfe59, 0x156d, 0x380b, 0xf0ab, 0x0d84, 0x4736, 0x8c72, 0xaf10,
            0x093c, 0x93b4, 0xa503, 0x468f, 0xe137, 0x5795, 0xdf42, 0x1c1d,
            0xe9dc, 0x73ad, 0xad5f, 0xd530, 0x07ca, 0xaf2e,
        ];

        let upgraded = upgrade_codes(&old_codes, &locs, data_size);

        // Expected: C reference codes from tag16h5.c
        let expected: Vec<u64> = vec![
            0x27c8, 0x31b6, 0x3859, 0x569c, 0x6c76, 0x7ddb, 0xaf09, 0xf5a1,
            0xfb8b, 0x1cb9, 0x28ca, 0xe8dc, 0x1426, 0x5770, 0x9253, 0xb702,
            0x063a, 0x8f34, 0xb4c0, 0x51ec, 0xe6f0, 0x5fa4, 0xdd43, 0x1aaa,
            0xe62f, 0x6dbc, 0xb6eb, 0xde10, 0x154d, 0xb57a,
        ];

        assert_eq!(upgraded.len(), expected.len());
        assert_eq!(upgraded, expected);
    }
}
