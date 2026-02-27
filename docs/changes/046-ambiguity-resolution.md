# 046 — Ambiguity resolution

## Goal
Handle the planar pose ambiguity where up to 2 local minima exist for the reprojection error.

## Preconditions
- 045 complete: orthogonal iteration

## Postconditions
- Near-perpendicular tag → single solution (second minimum not found)
- Oblique tag → two candidates tested, lower-error one returned
- Two solutions differ by > 0.1 rad rotation
- Always returns the pose with lower reprojection error

## Description
Add to `pose.rs`:

```rust
pub fn estimate_pose_with_ambiguity(
    det: &Detection,
    tagsize: f64,
    fx: f64, fy: f64,
    cx: f64, cy: f64,
) -> (Pose, Option<Pose>)  // (best, alternative)
```

Algorithm (Schweighofer & Pinz 2006):
1. Get initial pose via `estimate_pose()` (change 044)
2. Run orthogonal iteration → pose1, error1
3. Parameterize error as function of rotation angle around viewing axis
4. Find additional local minimum via degree-4 polynomial solver
5. If second minimum found and > 0.1 rad from first:
   - Initialize orthogonal iteration from second minimum
   - Run iteration → pose2, error2
6. Return (min_error_pose, Some(other_pose))
7. If no second minimum: return (pose1, None)

For most applications (tags not at extreme oblique angles), only one solution exists.

## References
- `docs/detection-spec.md` §12.4 — "Schweighofer & Pinz 2006: parameterize error vs rotation, solve degree-4"
