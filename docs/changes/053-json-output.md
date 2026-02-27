# 053 — JSON output of detections

## Goal
Output detection results as structured JSON to stdout or file.

## Preconditions
- 052 complete: images loaded
- 042 complete: detection pipeline returns results

## Postconditions
- Output is valid JSON
- Schema: `{ "file": "...", "image_width": N, "image_height": N, "detections": [...] }`
- Each detection: `{ "family": "...", "id": N, "hamming": N, "decision_margin": F, "center": [F, F], "corners": [[F,F],[F,F],[F,F],[F,F]] }`
- `--pretty` → indented JSON
- Multiple files → one JSON object per line (JSONL) or array
- `--output <file>` → write to file instead of stdout

## Description
Add to `apriltag-detect-cli/src/main.rs`:

```rust
#[derive(Serialize)]
struct OutputResult {
    file: String,
    image_width: u32,
    image_height: u32,
    detections: Vec<OutputDetection>,
}

#[derive(Serialize)]
struct OutputDetection {
    family: String,
    id: u32,
    hamming: u32,
    decision_margin: f32,
    center: [f64; 2],
    corners: [[f64; 2]; 4],
    #[serde(skip_serializing_if = "Option::is_none")]
    pose: Option<OutputPose>,
}

#[derive(Serialize)]
struct OutputPose {
    rotation: Vec<f64>,
    translation: [f64; 3],
    error: f64,
}
```

For each input file:
1. Load image
2. Run detection
3. Build `OutputResult`
4. Serialize: `serde_json::to_string` or `to_string_pretty`
5. Print to stdout or write to output file

## References
- Architecture decision — CLI JSON output format
