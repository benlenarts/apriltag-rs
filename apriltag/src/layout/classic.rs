/// Generate the data string for a classic layout of the given grid size.
///
/// Classic layout uses L-infinity (Chebyshev) distance from the grid boundary:
/// - dist == 0: white border (`w`)
/// - dist == 1: black border (`b`)
/// - dist >= 2: data (`d`)
pub fn classic_data_string(size: usize) -> String {
    let mut s = String::with_capacity(size * size);
    for y in 0..size {
        for x in 0..size {
            let dist = l_inf_dist_to_edge(x, y, size);
            s.push(match dist {
                0 => 'w',
                1 => 'b',
                _ => 'd',
            });
        }
    }
    s
}

fn l_inf_dist_to_edge(x: usize, y: usize, size: usize) -> usize {
    x.min(size - 1 - x).min(y.min(size - 1 - y))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn classic_8x8() {
        let s = classic_data_string(8);
        assert_eq!(s.len(), 64);
        // Row 0: all white
        assert_eq!(&s[0..8], "wwwwwwww");
        // Row 1: w + 6b + w
        assert_eq!(&s[8..16], "wbbbbbbw");
        // Row 2: w b dddd b w
        assert_eq!(&s[16..24], "wbddddbw");
    }

    #[test]
    fn classic_10x10() {
        let s = classic_data_string(10);
        assert_eq!(s.len(), 100);
        // Row 0: all white
        assert_eq!(&s[0..10], "wwwwwwwwww");
        // Row 1: w + 8b + w
        assert_eq!(&s[10..20], "wbbbbbbbbw");
        // Row 2: w b dddddd b w
        assert_eq!(&s[20..30], "wbddddddbw");
    }
}
