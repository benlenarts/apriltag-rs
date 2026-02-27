/// Generate the data string for a circle layout of the given grid size.
///
/// Circle layout uses L2 (Euclidean) distance from the grid center:
/// - `cutoff = size / 2.0 - 0.25`
/// - `border_distance = ceil(size/2.0 - cutoff*sqrt(0.5) - 0.5)`
/// - L-inf dist from edge == border_distance: black border (`b`)
/// - L-inf dist from edge == border_distance + 1: white border (`w`)
/// - L2 dist from center <= cutoff: data (`d`)
/// - otherwise: ignored (`x`)
pub fn circle_data_string(size: usize) -> String {
    let cutoff = size as f64 / 2.0 - 0.25;
    let border_distance =
        (size as f64 / 2.0 - cutoff * (0.5_f64).sqrt() - 0.5).ceil() as usize;

    let mut s = String::with_capacity(size * size);
    for y in 0..size {
        for x in 0..size {
            let linf = l_inf_dist_to_edge(x, y, size);
            let l2 = l2_dist_to_center(x, y, size);

            if linf == border_distance {
                s.push('b');
            } else if linf == border_distance + 1 {
                s.push('w');
            } else if l2 <= cutoff {
                s.push('d');
            } else {
                s.push('x');
            }
        }
    }
    s
}

fn l_inf_dist_to_edge(x: usize, y: usize, size: usize) -> usize {
    x.min(size - 1 - x).min(y.min(size - 1 - y))
}

fn l2_dist_to_center(x: usize, y: usize, size: usize) -> f64 {
    let r = size as f64 / 2.0;
    ((x as f64 + 0.5 - r).powi(2) + (y as f64 + 0.5 - r).powi(2)).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn circle_9x9_matches_reference() {
        // From TagCircle21h7.java
        let expected = "xxxdddxxxxbbbbbbbxxbwwwwwbxdbwdddwbddbwdddwbddbwdddwbdxbwwwwwbxxbbbbbbbxxxxdddxxx";
        let got = circle_data_string(9);
        assert_eq!(got, expected);
    }

    #[test]
    fn circle_11x11_matches_reference() {
        // From TagCircle49h12.java
        let expected = "xxxxdddxxxxxxdddddddxxxdbbbbbbbdxxdbwwwwwbdxddbwdddwbddddbwdddwbddddbwdddwbddxdbwwwwwbdxxdbbbbbbbdxxxdddddddxxxxxxdddxxxx";
        let got = circle_data_string(11);
        assert_eq!(got, expected);
    }
}
