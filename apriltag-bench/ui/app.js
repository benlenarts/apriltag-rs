/**
 * AprilTag Bench — interactive web UI.
 *
 * Loads two WASM modules:
 * - apriltag-bench-wasm: scene generation + distortion
 * - apriltag-wasm: tag detection
 *
 * Flow: slider change → generate scene (bench WASM) → detect (apriltag WASM)
 *       → compute metrics → render canvas + overlays.
 */

// WASM module references (set after init)
let benchWasm = null;
let detectorWasm = null;
let detector = null;

const statusEl = document.getElementById("status");

// ── WASM Initialization ──────────────────────────────────────────────

async function initWasm() {
  try {
    // Try loading WASM modules from expected paths
    // wasm-pack builds to pkg/ directories
    const benchPkg = "../../apriltag-bench-wasm/pkg/apriltag_bench_wasm.js";
    const detectorPkg = "../../apriltag-wasm/pkg/apriltag_wasm.js";

    const [benchMod, detectorMod] = await Promise.all([
      import(benchPkg).catch(() => null),
      import(detectorPkg).catch(() => null),
    ]);

    if (benchMod) {
      if (benchMod.default) await benchMod.default();
      benchWasm = benchMod;
    }

    if (detectorMod) {
      if (detectorMod.default) await detectorMod.default();
      detectorWasm = detectorMod;
      // Create a detector instance with full config
      const config = buildDetectorConfig(getFamily());
      detector = new detectorMod.Detector(config);
      lastDetectorKey = detectorConfigKey(getFamily());
    }

    if (benchWasm && detectorWasm) {
      statusEl.textContent = "Ready";
    } else if (benchWasm) {
      statusEl.textContent = "Scene gen ready (detector WASM not found)";
    } else if (detectorWasm) {
      statusEl.textContent = "Detector ready (bench WASM not found)";
    } else {
      statusEl.textContent =
        "WASM not loaded — run wasm-pack build for both crates. Using demo mode.";
    }
  } catch (e) {
    statusEl.textContent = `WASM load error: ${e.message}. Using demo mode.`;
    console.error("WASM init error:", e);
  }

  // Run initial render regardless
  update();
}

// ── DOM References ───────────────────────────────────────────────────

const canvas = document.getElementById("canvas");
const ctx = canvas.getContext("2d");

// Sliders and their value displays
const controls = {
  family:   { el: document.getElementById("family") },
  tagId:    { el: document.getElementById("tagId") },
  tagSize:  { el: document.getElementById("tagSize"),  valEl: document.getElementById("tagSizeVal") },
  rotation: { el: document.getElementById("rotation"), valEl: document.getElementById("rotationVal"), suffix: "\u00B0" },
  tiltX:    { el: document.getElementById("tiltX"),     valEl: document.getElementById("tiltXVal"),    suffix: "\u00B0" },
  tiltY:    { el: document.getElementById("tiltY"),     valEl: document.getElementById("tiltYVal"),    suffix: "\u00B0" },
  imgWidth: { el: document.getElementById("imgWidth") },
  imgHeight:{ el: document.getElementById("imgHeight") },
  noise:    { el: document.getElementById("noise"),     valEl: document.getElementById("noiseVal") },
  blur:     { el: document.getElementById("blur"),      valEl: document.getElementById("blurVal"),     decimals: 1 },
  contrast: { el: document.getElementById("contrast"),  valEl: document.getElementById("contrastVal"), decimals: 2 },
  // Detector config
  quadDecimate:    { el: document.getElementById("quadDecimate"),    valEl: document.getElementById("quadDecimateVal"),    decimals: 1 },
  quadSigma:       { el: document.getElementById("quadSigma"),       valEl: document.getElementById("quadSigmaVal"),       decimals: 1 },
  refineEdges:     { el: document.getElementById("refineEdges") },
  decodeSharpening:{ el: document.getElementById("decodeSharpening"),valEl: document.getElementById("decodeSharpeningVal"),decimals: 2 },
  maxHamming:      { el: document.getElementById("maxHamming") },
  // Quad threshold params
  minClusterPixels:  { el: document.getElementById("minClusterPixels") },
  maxNmaxima:        { el: document.getElementById("maxNmaxima") },
  criticalAngle:     { el: document.getElementById("criticalAngle"),     valEl: document.getElementById("criticalAngleVal"),     suffix: "\u00B0" },
  maxLineFitMse:     { el: document.getElementById("maxLineFitMse"),     valEl: document.getElementById("maxLineFitMseVal") },
  minWhiteBlackDiff: { el: document.getElementById("minWhiteBlackDiff") },
  deglitch:          { el: document.getElementById("deglitch") },
  // Overlay
  showGT:   { el: document.getElementById("showGT") },
  showDet:  { el: document.getElementById("showDet") },
  showErrors: { el: document.getElementById("showErrors") },
};

// Metric displays
const metricEls = {
  rate:    document.getElementById("metricRate"),
  rmse:    document.getElementById("metricRMSE"),
  maxErr:  document.getElementById("metricMaxErr"),
  genTime: document.getElementById("metricGenTime"),
  detTime: document.getElementById("metricDetTime"),
  count:   document.getElementById("metricCount"),
};

const detectionsEl = document.getElementById("detections");

// ── Value Getters ────────────────────────────────────────────────────

function getFamily()   { return controls.family.el.value; }
function getTagId()    { return parseInt(controls.tagId.el.value, 10); }
function getTagSize()  { return parseFloat(controls.tagSize.el.value); }
function getRotation() { return parseFloat(controls.rotation.el.value); }
function getTiltX()    { return parseFloat(controls.tiltX.el.value); }
function getTiltY()    { return parseFloat(controls.tiltY.el.value); }
function getWidth()    { return parseInt(controls.imgWidth.el.value, 10); }
function getHeight()   { return parseInt(controls.imgHeight.el.value, 10); }
function getNoise()    { return parseFloat(controls.noise.el.value); }
function getBlur()     { return parseFloat(controls.blur.el.value); }
function getContrast() { return parseFloat(controls.contrast.el.value); }
// Detector config getters
function getQuadDecimate()    { return parseFloat(controls.quadDecimate.el.value); }
function getQuadSigma()       { return parseFloat(controls.quadSigma.el.value); }
function getRefineEdges()     { return controls.refineEdges.el.checked; }
function getDecodeSharpening(){ return parseFloat(controls.decodeSharpening.el.value); }
function getMaxHamming()      { return parseInt(controls.maxHamming.el.value, 10); }
function getMinClusterPixels(){ return parseInt(controls.minClusterPixels.el.value, 10); }
function getMaxNmaxima()      { return parseInt(controls.maxNmaxima.el.value, 10); }
function getCriticalAngle()   { return parseFloat(controls.criticalAngle.el.value); }
function getMaxLineFitMse()   { return parseFloat(controls.maxLineFitMse.el.value); }
function getMinWhiteBlackDiff(){ return parseInt(controls.minWhiteBlackDiff.el.value, 10); }
function getDeglitch()        { return controls.deglitch.el.checked; }
function showGT()      { return controls.showGT.el.checked; }
function showDet()     { return controls.showDet.el.checked; }
function showErrors()  { return controls.showErrors.el.checked; }

/**
 * Build a WasmDetectorConfig from current UI state.
 * Returns a plain object matching the WasmDetectorConfig shape.
 */
function buildDetectorConfig(family) {
  const angleDeg = getCriticalAngle();
  const angleRad = angleDeg * Math.PI / 180;
  return {
    families: [family],
    quad_decimate: getQuadDecimate(),
    quad_sigma: getQuadSigma(),
    refine_edges: getRefineEdges(),
    decode_sharpening: getDecodeSharpening(),
    max_hamming: getMaxHamming(),
    min_cluster_pixels: getMinClusterPixels(),
    max_nmaxima: getMaxNmaxima(),
    cos_critical_rad: Math.cos(angleRad),
    max_line_fit_mse: getMaxLineFitMse(),
    min_white_black_diff: getMinWhiteBlackDiff(),
    deglitch: getDeglitch(),
  };
}

/** Serialize detector config to a string for change detection. */
function detectorConfigKey(family) {
  return JSON.stringify(buildDetectorConfig(family));
}

// ── Event Binding ────────────────────────────────────────────────────

let updateTimer = null;

function scheduleUpdate() {
  // Update value display immediately
  for (const [, ctrl] of Object.entries(controls)) {
    if (ctrl.valEl) {
      const val = parseFloat(ctrl.el.value);
      const decimals = ctrl.decimals || 0;
      ctrl.valEl.textContent = val.toFixed(decimals) + (ctrl.suffix || "");
    }
  }
  // Debounce the expensive update
  if (updateTimer) clearTimeout(updateTimer);
  updateTimer = setTimeout(update, 50);
}

for (const [, ctrl] of Object.entries(controls)) {
  ctrl.el.addEventListener("input", scheduleUpdate);
  ctrl.el.addEventListener("change", scheduleUpdate);
}

// ── Core Update Loop ─────────────────────────────────────────────────

let lastScene = null;
let lastDetections = [];
let lastDetectorKey = "";

async function update() {
  const width = getWidth();
  const height = getHeight();
  const family = getFamily();
  const tagId = getTagId();
  const tagSize = getTagSize();
  const rotation = getRotation();
  const tiltX = getTiltX();
  const tiltY = getTiltY();
  const noise = getNoise();
  const blur = getBlur();
  const contrast = getContrast();

  canvas.width = width;
  canvas.height = height;

  let imageData;
  let groundTruth = [];
  let genTimeMs = 0;

  if (benchWasm && benchWasm.generateScene) {
    // Use WASM scene generation
    const t0 = performance.now();
    try {
      const scene = benchWasm.generateScene(
        width, height, family, tagId, tagSize,
        rotation, tiltX, tiltY, noise, blur, contrast
      );
      genTimeMs = performance.now() - t0;
      lastScene = scene;

      // Convert grayscale to RGBA for canvas
      const gray = scene.imageData;
      imageData = new ImageData(width, height);
      for (let i = 0; i < gray.length; i++) {
        const v = gray[i];
        imageData.data[i * 4]     = v;
        imageData.data[i * 4 + 1] = v;
        imageData.data[i * 4 + 2] = v;
        imageData.data[i * 4 + 3] = 255;
      }
      groundTruth = scene.groundTruth || [];
    } catch (e) {
      console.error("Scene generation error:", e);
      statusEl.textContent = `Error: ${e.message}`;
      return;
    }
  } else {
    // Demo mode: draw a placeholder
    imageData = new ImageData(width, height);
    for (let i = 0; i < width * height; i++) {
      const v = 128;
      imageData.data[i * 4]     = v;
      imageData.data[i * 4 + 1] = v;
      imageData.data[i * 4 + 2] = v;
      imageData.data[i * 4 + 3] = 255;
    }
    genTimeMs = 0;
  }

  // Render image to canvas
  ctx.putImageData(imageData, 0, 0);

  // Detect
  let detections = [];
  let detTimeMs = 0;

  if (detector && lastScene) {
    try {
      // Recreate detector if config changed (family, thresholds, etc.)
      const configKey = detectorConfigKey(family);
      if (configKey !== lastDetectorKey) {
        detector = new detectorWasm.Detector(buildDetectorConfig(family));
        lastDetectorKey = configKey;
      }
      const grayData = lastScene.imageData;
      const t0 = performance.now();
      const rawDets = detector.detect(grayData, width, height);
      detTimeMs = performance.now() - t0;
      detections = rawDets || [];
    } catch (e) {
      console.error("Detection error:", e);
    }
  }

  lastDetections = detections;

  // Draw overlays
  drawOverlays(groundTruth, detections);

  // Update metrics
  updateMetrics(groundTruth, detections, genTimeMs, detTimeMs);
}

// ── Overlay Drawing ──────────────────────────────────────────────────

function drawOverlays(groundTruth, detections) {
  // Draw ground truth corners (green)
  if (showGT() && groundTruth.length > 0) {
    for (const gt of groundTruth) {
      drawQuad(gt.corners, "#4caf50", 2);
      drawCornerDots(gt.corners, "#4caf50", 4);
    }
  }

  // Draw detected corners (red)
  if (showDet() && detections.length > 0) {
    for (const det of detections) {
      const corners = det.corners || formatDetCorners(det);
      if (corners) {
        drawQuad(corners, "#f44336", 2);
        drawCornerDots(corners, "#f44336", 3);
      }
    }
  }

  // Draw error lines (yellow)
  if (showErrors() && groundTruth.length > 0 && detections.length > 0) {
    for (const gt of groundTruth) {
      const matched = findMatchingDetection(gt, detections);
      if (matched) {
        const detCorners = matched.corners || formatDetCorners(matched);
        if (detCorners) {
          drawErrorLines(gt.corners, detCorners);
        }
      }
    }
  }
}

function formatDetCorners(det) {
  // apriltag-wasm may return corners in different formats
  if (det.corners && Array.isArray(det.corners)) return det.corners;
  if (det.corner0 !== undefined) {
    return [det.corner0, det.corner1, det.corner2, det.corner3];
  }
  return null;
}

function findMatchingDetection(gt, detections) {
  return detections.find(d => d.id === gt.tagId || d.tagId === gt.tagId);
}

function drawQuad(corners, color, lineWidth) {
  ctx.strokeStyle = color;
  ctx.lineWidth = lineWidth;
  ctx.beginPath();
  ctx.moveTo(corners[0][0], corners[0][1]);
  for (let i = 1; i < 4; i++) {
    ctx.lineTo(corners[i][0], corners[i][1]);
  }
  ctx.closePath();
  ctx.stroke();
}

function drawCornerDots(corners, color, radius) {
  ctx.fillStyle = color;
  for (const [x, y] of corners) {
    ctx.beginPath();
    ctx.arc(x, y, radius, 0, Math.PI * 2);
    ctx.fill();
  }
}

function drawErrorLines(gtCorners, detCorners) {
  ctx.strokeStyle = "#ffeb3b";
  ctx.lineWidth = 1;
  ctx.setLineDash([3, 3]);
  for (let i = 0; i < 4; i++) {
    ctx.beginPath();
    ctx.moveTo(gtCorners[i][0], gtCorners[i][1]);
    ctx.lineTo(detCorners[i][0], detCorners[i][1]);
    ctx.stroke();
  }
  ctx.setLineDash([]);
}

// ── Metrics Computation ──────────────────────────────────────────────

function updateMetrics(groundTruth, detections, genTimeMs, detTimeMs) {
  const nExpected = groundTruth.length;
  const nDetected = detections.length;

  // Detection rate
  let rate = 0;
  if (nExpected > 0) {
    let matched = 0;
    for (const gt of groundTruth) {
      if (findMatchingDetection(gt, detections)) matched++;
    }
    rate = matched / nExpected;
  }

  // Corner RMSE
  let totalSqErr = 0;
  let nCorners = 0;
  let maxErr = 0;

  for (const gt of groundTruth) {
    const det = findMatchingDetection(gt, detections);
    if (!det) continue;
    const detCorners = det.corners || formatDetCorners(det);
    if (!detCorners) continue;

    for (let i = 0; i < 4; i++) {
      const dx = gt.corners[i][0] - detCorners[i][0];
      const dy = gt.corners[i][1] - detCorners[i][1];
      const err = Math.sqrt(dx * dx + dy * dy);
      totalSqErr += dx * dx + dy * dy;
      nCorners++;
      if (err > maxErr) maxErr = err;
    }
  }

  const rmse = nCorners > 0 ? Math.sqrt(totalSqErr / nCorners) : 0;

  // Update display
  const rateEl = metricEls.rate;
  rateEl.textContent = `${(rate * 100).toFixed(0)}%`;
  rateEl.className = `value ${rate >= 1 ? "good" : rate > 0 ? "warn" : "bad"}`;

  const rmseEl = metricEls.rmse;
  rmseEl.textContent = `${rmse.toFixed(2)} px`;
  rmseEl.className = `value ${rmse < 1 ? "good" : rmse < 3 ? "warn" : "bad"}`;

  const maxErrEl = metricEls.maxErr;
  maxErrEl.textContent = `${maxErr.toFixed(2)} px`;
  maxErrEl.className = `value ${maxErr < 1 ? "good" : maxErr < 5 ? "warn" : "bad"}`;

  metricEls.genTime.textContent = `${genTimeMs.toFixed(1)} ms`;
  metricEls.genTime.className = "value neutral";

  metricEls.detTime.textContent = `${detTimeMs.toFixed(1)} ms`;
  metricEls.detTime.className = "value neutral";

  metricEls.count.textContent = nDetected.toString();
  metricEls.count.className = `value ${nDetected === nExpected && nExpected > 0 ? "good" : nDetected > 0 ? "warn" : nExpected === 0 ? "neutral" : "bad"}`;

  // Detection list
  detectionsEl.innerHTML = "";
  for (const det of detections) {
    const row = document.createElement("div");
    row.className = "det-row";
    const id = det.id ?? det.tagId ?? "?";
    const ham = det.hamming ?? "?";
    const margin = (det.decisionMargin ?? det.decision_margin ?? 0).toFixed(1);
    const cx = (det.center?.[0] ?? 0).toFixed(1);
    const cy = (det.center?.[1] ?? 0).toFixed(1);
    row.innerHTML = `Tag <span class="tag-id">${id}</span>: hamming=${ham}, margin=${margin}, center=(${cx}, ${cy})`;
    detectionsEl.appendChild(row);
  }
}

// ── Initialize ───────────────────────────────────────────────────────

initWasm();
