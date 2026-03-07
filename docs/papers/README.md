# Reference Papers

Papers relevant to this project's detection pipeline, tag family generation, and pose estimation. Some are downloaded by `scripts/fetch-references.sh`; this file catalogs the full set we've surveyed.

## Core AprilTag papers

These define the detection pipeline and tag family design this project implements.

### Olson 2011 — "AprilTag: A robust and flexible visual fiducial system"
- Original AprilTag paper. Gradient-based quad detection, lexicode tag families, graph-based segmentation.
- Defines the core pipeline: grayscale → gradient → segmentation → quad fitting → homography → decode.

### Wang & Olson 2016 — "AprilTag 2: Efficient and robust fiducial detection"
- Major speed improvements: weighted least-squares line fitting on gradient clusters, precomputed tag family lookup tables, improved quad rejection heuristics.
- This is the primary reference for our detection implementation.

### Krogius, Haggenmiller & Olson 2019 — "Flexible Layouts for Fiducial Tags"
- Introduces flexible tag layouts (Standard41h12, Standard52h13, etc.) and the generation algorithm with configurable border, data bits, and minimum Hamming distance.
- Primary reference for our `codegen.rs` implementation.

## Image processing foundations

### Felzenszwalb & Huttenlocher 2004 — "Efficient Graph-Based Image Segmentation"
- Graph-based segmentation using union-find. The AprilTag pipeline uses a simplified variant for clustering gradient-similar pixels.

## Pose estimation & accuracy

### Abbas et al. 2019 — "Analysis and Improvements in AprilTag Based State Estimation"
- **Access:** Open access (CC BY 4.0). DOI: [10.3390/s19245480](https://doi.org/10.3390/s19245480)
- Empirical accuracy analysis of AprilTag pose output, validated against motion capture.
- **Key findings:**
  - Yaw angle (camera rotation about vertical axis) is the **primary** error source. When the camera z-axis doesn't point at the tag center, variance in x explodes from ~0.002 cm² to ~194 cm² at 70cm.
  - Distance increases error: ~1cm x-error, ~0.4cm y-error at 70cm even in ideal conditions.
  - Lateral offset roughly doubles error vs centered viewing.
- **SYAC correction:** Pure trigonometric post-processing using detected yaw and distance. Reduces off-axis errors by 50-70%. ~10 lines of code, no hardware required.
- **Limitation:** Only analyzes 2D (x, y, yaw); full 6-DOF (roll, pitch, z-height) is noted as future work.
- See issue #90 for actionable work items.

### Kallwies et al. 2020 — "Determining and Improving the Localization Accuracy of AprilTag Detection"
- **Access:** Paywalled (ICRA 2020). IEEE: https://ieeexplore.ieee.org/document/9197427/
- Sub-pixel edge refinement achieving 0.017px vs 0.17px median corner error. Partial occlusion filtering. Comparison of 4 AprilTag libraries.
- **Companion repo:** https://github.com/UniBwTAS/apriltags_tas — **no license** on the improvement code (all rights reserved by default). The `apriltags/` subdir is LGPL-2.1 (Humhu/apriltags C++ port), not the improvements. We can study the approach but must independently reimplement. Algorithmic ideas are not copyrightable.

### Schweighofer & Pinz 2006 — pose ambiguity for planar targets
- **Access:** Paywalled.
- Describes two local minima that exist when estimating pose from a planar target. The core insight is well-summarized in secondary sources. Revisit when pose estimation is actively developed.

## Surveyed but not applicable

### Garrido-Jurado et al. 2014 — ArUco dictionary generation
- Different marker system. ArUco maximizes inter-marker Hamming distance; our codegen follows the AprilTag reference (LCG-based). Distinct problem.

### Jin et al. 2017 — RGB-D fiducial fusion
- Requires depth sensor data. This project is monocular RGB only.

### FPGA/GPU acceleration papers
- Hardware-specific. Project targets pure Rust/WASM.

### Fiala 2005 — ARTag
- Historical predecessor to AprilTag. No insights beyond the core AprilTag papers.
