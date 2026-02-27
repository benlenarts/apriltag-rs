# 004 — 2×2 symmetric eigenvalue decomposition

## Goal
Implement an analytic eigenvalue/eigenvector solver for 2×2 symmetric (covariance) matrices, used in line fitting.

## Preconditions
- 003 complete: `math.rs` exists

## Postconditions
- `[[2,1],[1,2]]` → eigenvalues `(3.0, 1.0)` (larger first)
- Identity matrix → eigenvalues `(1.0, 1.0)`
- Diagonal matrix `[[5,0],[0,3]]` → eigenvalues `(5.0, 3.0)`
- Eigenvectors are orthogonal and unit-length
- Near-zero off-diagonal → eigenvalues ≈ diagonal entries

## Description
Add to `math.rs`:

```rust
pub fn eigenvalues_sym2x2(cxx: f64, cxy: f64, cyy: f64) -> (f64, f64)
pub fn eigenvector_sym2x2(cxx: f64, cxy: f64, cyy: f64, eigenvalue: f64) -> (f64, f64)
```

Analytic formula for 2×2 symmetric matrix eigenvalues:
- `trace = cxx + cyy`
- `det = cxx * cyy - cxy * cxy`
- `discriminant = sqrt(trace² - 4·det)`
- `λ1 = (trace + discriminant) / 2` (larger)
- `λ2 = (trace - discriminant) / 2` (smaller)

Eigenvector for eigenvalue λ: normalize `(cxy, λ - cxx)` or `(λ - cyy, cxy)`.

This is the critical inner operation for line fitting — called once per segment per combination in the exhaustive corner search.

## References
- `docs/detection-spec.md` §5 — line fitting uses smallest eigenvalue as MSE, largest eigenvector as direction
