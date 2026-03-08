/// Forward elimination with partial pivoting on an `R×C` augmented matrix.
///
/// Reduces `a` to row echelon form in-place. Returns `None` if any
/// pivot is smaller than `tol` (singular or near-singular system).
pub(crate) fn forward_eliminate<const R: usize, const C: usize>(
    a: &mut [[f64; C]; R],
    tol: f64,
) -> Option<()> {
    for col in 0..R {
        // Find pivot
        let mut max_val = a[col][col].abs();
        let mut max_row = col;
        for row in (col + 1)..R {
            let v = a[row][col].abs();
            if v > max_val {
                max_val = v;
                max_row = row;
            }
        }
        if max_val < tol {
            return None;
        }

        // Swap
        if max_row != col {
            a.swap(col, max_row);
        }

        // Eliminate below
        let pivot = a[col][col];
        for row in (col + 1)..R {
            let factor = a[row][col] / pivot;
            for c in col..C {
                a[row][c] -= factor * a[col][c];
            }
        }
    }

    Some(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn forward_eliminate_3x4() {
        // Solve: x=1, y=2, z=3
        // x + 2y + 3z = 14
        // 2x + 5y + 3z = 21
        // x + 0y + 8z = 25
        let mut a = [
            [1.0, 2.0, 3.0, 14.0],
            [2.0, 5.0, 3.0, 21.0],
            [1.0, 0.0, 8.0, 25.0],
        ];
        assert!(forward_eliminate::<3, 4>(&mut a, 1e-10).is_some());

        // Back-substitute
        let mut x = [0.0; 3];
        for row in (0..3).rev() {
            let mut sum = a[row][3];
            for c in (row + 1)..3 {
                sum -= a[row][c] * x[c];
            }
            x[row] = sum / a[row][row];
        }
        assert!((x[0] - 1.0).abs() < 1e-10);
        assert!((x[1] - 2.0).abs() < 1e-10);
        assert!((x[2] - 3.0).abs() < 1e-10);
    }

    #[test]
    fn forward_eliminate_8x9() {
        // Identity-like 8x8 system with RHS = row index + 1
        let mut a = [[0.0f64; 9]; 8];
        for i in 0..8 {
            a[i][i] = 1.0;
            a[i][8] = (i + 1) as f64;
        }
        assert!(forward_eliminate::<8, 9>(&mut a, 1e-10).is_some());

        let mut x = [0.0; 8];
        for row in (0..8).rev() {
            let mut sum = a[row][8];
            for c in (row + 1)..8 {
                sum -= a[row][c] * x[c];
            }
            x[row] = sum / a[row][row];
        }
        for i in 0..8 {
            assert!((x[i] - (i + 1) as f64).abs() < 1e-10);
        }
    }

    #[test]
    fn forward_eliminate_singular() {
        let mut a = [[1.0, 2.0, 3.0], [2.0, 4.0, 6.0], [3.0, 6.0, 9.0]];
        assert!(forward_eliminate::<3, 3>(&mut a, 1e-10).is_none());
    }
}
