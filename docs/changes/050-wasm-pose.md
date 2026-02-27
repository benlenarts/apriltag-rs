# 050 — wasm-bindgen pose estimation

## Goal
Expose pose estimation to JavaScript via the WASM Detector.

## Preconditions
- 046 complete: `estimate_pose_with_ambiguity` in `apriltag-detect`
- 049 complete: WASM Detector with detect()

## Postconditions
- `detector.estimate_pose(detection, tagsize, fx, fy, cx, cy)` returns `WasmPose`
- Returns the best pose (lower error of the two candidates)
- Invalid detection input → clear `JsError`

## Description
Add to WASM `Detector` impl:

```rust
#[wasm_bindgen]
impl Detector {
    pub fn estimate_pose(
        &self,
        detection: JsValue,
        tagsize: f64,
        fx: f64, fy: f64,
        cx: f64, cy: f64,
    ) -> Result<JsValue, JsError> {
        let det: WasmDetection = serde_wasm_bindgen::from_value(detection)?;
        let core_det = convert_to_core_detection(&det);
        let (pose, _alt) = estimate_pose_with_ambiguity(&core_det, tagsize, fx, fy, cx, cy);
        let wasm_pose = WasmPose {
            rotation: pose.rotation.to_vec(),
            translation: pose.translation,
            error: pose.error,
        };
        Ok(serde_wasm_bindgen::to_value(&wasm_pose)?)
    }
}
```

## References
- Architecture decision — WASM API includes pose estimation
- `docs/detection-spec.md` §12 — pose estimation requires camera intrinsics + tag size
