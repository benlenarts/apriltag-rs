# 056 — WASM build verification

## Goal
Verify that the full detection pipeline compiles to wasm32-unknown-unknown and the WASM bindings build successfully.

## Preconditions
- All previous changes complete

## Postconditions
- `cargo build --target wasm32-unknown-unknown -p apriltag -p apriltag-detect` succeeds
- `wasm-pack build apriltag-wasm --target web` succeeds
- No `std`-only APIs in the core detection path
- Generated `.wasm` file is reasonably sized (< 1 MB for detection-only)
- TypeScript `.d.ts` file generated with correct interfaces

## Description
1. Run WASM builds and fix any compilation errors:
   - Replace any `std::time` usage with `#[cfg(not(target_arch = "wasm32"))]` guards
   - Ensure no filesystem or threading APIs in `apriltag-detect` core path
   - Verify `HashMap` from `std::collections` works in WASM (it does)

2. Check WASM binary size:
   ```bash
   wasm-pack build apriltag-wasm --target web --release
   ls -la apriltag-wasm/pkg/*.wasm
   ```

3. Verify TypeScript types:
   ```bash
   cat apriltag-wasm/pkg/apriltag_wasm.d.ts
   ```

4. Optional: run wasm-opt for size optimization:
   ```bash
   wasm-opt -Oz apriltag-wasm/pkg/apriltag_wasm_bg.wasm -o optimized.wasm
   ```

## References
- `CLAUDE.md` — "cargo build --target wasm32-unknown-unknown" verification command
- Architecture decision — WASM compatibility tenet
