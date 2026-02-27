# 048 — Tsify type definitions

## Goal
Define WASM-specific wrapper types with `#[derive(Tsify)]` for automatic TypeScript interface generation.

## Preconditions
- 047 complete: `apriltag-wasm` crate compiles

## Postconditions
- `WasmDetection`, `WasmPose`, `WasmDetectorConfig` types defined
- `#[derive(Tsify, Serialize, Deserialize)]` on all types
- `wasm-pack build` generates `.d.ts` with correct TypeScript interfaces
- Types mirror the core `Detection`, `Pose`, `DetectorConfig` structures

## Description
Add to `apriltag-wasm/src/lib.rs`:

```rust
use tsify_next::Tsify;
use serde::{Serialize, Deserialize};

#[derive(Tsify, Serialize, Deserialize)]
#[tsify(into_wasm_abi, from_wasm_abi)]
pub struct WasmDetectorConfig {
    pub families: Vec<String>,
    #[serde(default = "default_decimate")]
    pub quad_decimate: Option<f32>,
    #[serde(default)]
    pub quad_sigma: Option<f32>,
    #[serde(default)]
    pub refine_edges: Option<bool>,
    #[serde(default)]
    pub decode_sharpening: Option<f64>,
}

#[derive(Tsify, Serialize)]
#[tsify(into_wasm_abi)]
pub struct WasmDetection {
    pub family: String,
    pub id: u32,
    pub hamming: u32,
    pub decision_margin: f32,
    pub center: [f64; 2],
    pub corners: [[f64; 2]; 4],
}

#[derive(Tsify, Serialize)]
#[tsify(into_wasm_abi)]
pub struct WasmPose {
    pub rotation: Vec<f64>,      // 9 elements, row-major
    pub translation: [f64; 3],
    pub error: f64,
}
```

These generate TypeScript:
```typescript
interface WasmDetection { family: string; id: number; ... }
interface WasmPose { rotation: number[]; translation: [number, number, number]; ... }
```

## References
- Architecture decision — tsify-next for TypeScript type generation
- `tsify-next` crate documentation
