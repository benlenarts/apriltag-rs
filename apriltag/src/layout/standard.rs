/// Generate the data string for a standard layout of the given grid size.
///
/// Standard layout uses L-infinity distance from the grid boundary:
/// - dist == 1: black border (`b`)
/// - dist == 2: white border (`w`)
/// - dist == 0 or dist >= 3: data (`d`)
pub fn standard_data_string(size: usize) -> String {
    let mut s = String::with_capacity(size * size);
    for y in 0..size {
        for x in 0..size {
            let dist = l_inf_dist_to_edge(x, y, size);
            s.push(match dist {
                1 => 'b',
                2 => 'w',
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
    fn standard_9x9_matches_reference() {
        // From TagStandard41h12.java
        let expected = "ddddddddddbbbbbbbddbwwwwwbddbwdddwbddbwdddwbddbwdddwbddbwwwwwbddbbbbbbbdddddddddd";
        let got = standard_data_string(9);
        assert_eq!(got, expected);
    }

    #[test]
    fn standard_10x10_matches_reference() {
        // From TagStandard52h13.java
        let expected = "dddddddddddbbbbbbbbddbwwwwwwbddbwddddwbddbwddddwbddbwddddwbddbwddddwbddbwwwwwwbddbbbbbbbbddddddddddd";
        let got = standard_data_string(10);
        assert_eq!(got, expected);
    }
}
