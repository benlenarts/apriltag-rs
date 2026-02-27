# 054 — CLI pose estimation integration

## Goal
Add `--pose` flag to the CLI that computes pose estimates when camera intrinsics are provided.

## Preconditions
- 053 complete: JSON output working
- 046 complete: pose estimation in `apriltag-detect`

## Postconditions
- `--pose --tag-size 0.1 --fx 800 --fy 800 --cx 320 --cy 240` → detections include pose data
- Missing `--tag-size` or any `--fx/fy/cx/cy` when `--pose` set → clear error
- Without `--pose` → no pose data in output
- Pose output: `rotation` (9 floats), `translation` (3 floats), `error` (float)

## Description
Add to `apriltag-detect-cli/src/main.rs`:

```rust
fn main() -> anyhow::Result<()> {
    let args = Args::parse();

    // Validate pose args
    if args.pose {
        let tag_size = args.tag_size.ok_or_else(|| anyhow!("--pose requires --tag-size"))?;
        let fx = args.fx.ok_or_else(|| anyhow!("--pose requires --fx"))?;
        let fy = args.fy.ok_or_else(|| anyhow!("--pose requires --fy"))?;
        let cx = args.cx.ok_or_else(|| anyhow!("--pose requires --cx"))?;
        let cy = args.cy.ok_or_else(|| anyhow!("--pose requires --cy"))?;
    }

    for image_path in &args.images {
        let image = load_image(image_path)?;
        let detections = detector.detect(&image);

        let output_dets: Vec<_> = detections.iter().map(|det| {
            let pose = if args.pose {
                let (p, _) = estimate_pose_with_ambiguity(det, tag_size, fx, fy, cx, cy);
                Some(OutputPose { ... })
            } else {
                None
            };
            OutputDetection { ..., pose }
        }).collect();

        // serialize and output
    }
    Ok(())
}
```

## References
- Architecture decision — CLI pose options
- `docs/detection-spec.md` §12 — pose requires tagsize + camera intrinsics
