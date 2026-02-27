# 049 — wasm-bindgen detect entry point

## Goal
Expose the `Detector` class to JavaScript with `detect(data, width, height)` and `detect_rgba()` methods.

## Preconditions
- 048 complete: tsify types defined

## Postconditions
- JS can `new Detector(config)` with family names
- `detector.detect(uint8Array, width, height)` returns `WasmDetection[]` from grayscale data
- `detector.detect_rgba(uint8Array, width, height)` auto-converts RGBA → grayscale and detects
- Detector holds stateful workspace → zero allocation on subsequent calls
- Invalid inputs (wrong buffer size) → clear `JsError`

## Description
Add to `apriltag-wasm/src/lib.rs`:

```rust
#[wasm_bindgen]
pub struct Detector {
    inner: apriltag_detect::Detector,
}

#[wasm_bindgen]
impl Detector {
    #[wasm_bindgen(constructor)]
    pub fn new(config: WasmDetectorConfig) -> Result<Detector, JsError> {
        let mut det_config = DetectorConfig::default();
        // Apply optional overrides from config
        let mut inner = apriltag_detect::Detector::new(det_config);
        for name in &config.families {
            let family = builtin_family(name).ok_or_else(|| JsError::new("unknown family"))?;
            inner.add_family(family, 2);
        }
        Ok(Detector { inner })
    }

    pub fn detect(&mut self, data: &[u8], width: u32, height: u32) -> Result<JsValue, JsError> {
        // Validate: data.len() == width * height
        let image = ImageU8::from_buffer(data, width, height);
        let detections = self.inner.detect(&image);
        let wasm_dets: Vec<WasmDetection> = detections.into_iter().map(convert).collect();
        Ok(serde_wasm_bindgen::to_value(&wasm_dets)?)
    }

    pub fn detect_rgba(&mut self, data: &[u8], width: u32, height: u32) -> Result<JsValue, JsError> {
        // Convert RGBA → grayscale: gray = 0.299*R + 0.587*G + 0.114*B
        // Then call detect()
    }
}
```

**RGBA conversion**: `gray = (77*r + 150*g + 29*b) >> 8` (integer approximation).

**Zero-copy for grayscale**: `&[u8]` in wasm-bindgen maps directly to a view into WASM linear memory — no copy when called from `Uint8Array`.

## References
- Architecture decision — WASM API: grayscale + RGBA entry points
- `wasm-bindgen` documentation for `&[u8]` handling
