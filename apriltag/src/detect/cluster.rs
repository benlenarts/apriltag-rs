use super::image::ImageU8;
use super::unionfind::UnionFind;

/// An edge point with fixed-point coordinates and gradient direction.
#[derive(Debug, Clone, Copy)]
pub struct Pt {
    /// 2x actual x coordinate (half-pixel precision).
    pub x: u16,
    /// 2x actual y coordinate (half-pixel precision).
    pub y: u16,
    /// Gradient direction x component.
    pub gx: i16,
    /// Gradient direction y component.
    pub gy: i16,
    /// Angular sort key for quad fitting (bit pattern of non-negative f32).
    pub slope: u32,
}

/// A cluster of edge points sharing the same component-pair boundary.
#[derive(Debug, Clone)]
pub struct Cluster {
    pub points: Vec<Pt>,
}

const EMPTY: u32 = u32::MAX;

/// Hash table entry with its own growable point buffer.
struct Entry {
    key: u64,
    next: u32, // index of next entry in chain (EMPTY = end)
    points: Vec<Pt>,
}

/// Hash table for grouping boundary points by cluster key during the pixel scan.
///
/// Uses separate chaining with a flat bucket array and pool-allocated entries.
/// Each entry owns a `Vec<Pt>` that grows independently. Recycled Vecs are
/// retained across frames to avoid per-cluster heap allocation.
pub struct ClusterMap {
    buckets: Vec<u32>,
    entries: Vec<Entry>,
    /// Pool of cleared `Vec<Pt>` recycled from previous frames.
    free_vecs: Vec<Vec<Pt>>,
}

impl Default for ClusterMap {
    fn default() -> Self {
        Self::new()
    }
}

impl ClusterMap {
    pub fn new() -> Self {
        Self {
            buckets: Vec::new(),
            entries: Vec::new(),
            free_vecs: Vec::new(),
        }
    }

    /// Prepare for a new frame. Recycles inner Vecs and clears the table.
    fn reset(&mut self, n_buckets: usize) {
        // Recycle entry Vecs into the free pool
        for mut entry in self.entries.drain(..) {
            entry.points.clear();
            self.free_vecs.push(entry.points);
        }
        self.buckets.clear();
        self.buckets.resize(n_buckets, EMPTY);
    }

    /// Get a recycled Vec or create a new one.
    #[inline]
    fn alloc_vec(&mut self) -> Vec<Pt> {
        self.free_vecs.pop().unwrap_or_default()
    }

    /// Insert a point into the cluster identified by `key`.
    #[inline]
    fn insert(&mut self, key: u64, pt: Pt) {
        let bucket = self.bucket_index(key);
        let mut idx = self.buckets[bucket];

        // Walk chain looking for existing entry with this key
        while idx != EMPTY {
            let entry = &self.entries[idx as usize];
            if entry.key == key {
                break;
            }
            idx = entry.next;
        }

        if idx != EMPTY {
            self.entries[idx as usize].points.push(pt);
        } else {
            // New entry with recycled Vec
            let entry_idx = self.entries.len() as u32;
            let mut points = self.alloc_vec();
            points.push(pt);
            self.entries.push(Entry {
                key,
                next: self.buckets[bucket],
                points,
            });
            self.buckets[bucket] = entry_idx;
        }
    }

    #[inline(always)]
    fn bucket_index(&self, key: u64) -> usize {
        // FxHash-style multiply for u64 keys
        let hash = key.wrapping_mul(0x517cc1b727220a95);
        (hash as usize) % self.buckets.len()
    }
}

/// Extract boundary points between adjacent black/white components and group
/// them into clusters keyed by the ordered pair of component representatives.
///
/// Each cluster represents a potential quad edge.
///
/// Points are inserted directly into a hash table during the scan (O(n) amortized),
/// avoiding the O(n log n) sort that dominates on noisy images with many boundary points.
pub fn gradient_clusters(
    threshed: &ImageU8,
    uf: &mut UnionFind,
    min_cluster_size: u32,
    cluster_map: &mut ClusterMap,
) -> Vec<Cluster> {
    let w = threshed.width;
    let h = threshed.height;
    let min_component_size = 25u32;

    // Size the hash table at ~20% of pixel count, matching C reference
    let n_buckets = ((w as usize * h as usize) / 5).max(16);
    cluster_map.reset(n_buckets);

    let buf = &threshed.buf;
    let stride = threshed.stride as usize;

    // Check a neighbor offset and add a boundary point if valid.
    // All accesses are guaranteed in-bounds because x ∈ [1, w-2] and y ∈ [1, h-2]
    // with dx, dy ∈ {-1, 0, 1}.
    // `rep0` is pre-computed by the caller to avoid redundant find() calls.
    // Returns true when a boundary point was added.
    macro_rules! do_conn {
        ($map:expr, $uf:expr, $buf:expr, $stride:expr,
         $x:expr, $y:expr, $rep0:expr, $v0:expr,
         $dx:expr, $dy:expr, $w:expr, $min_component_size:expr) => {{
            let nx = ($x as i32 + $dx) as usize;
            let ny = ($y as i32 + $dy) as usize;
            let v1 = $buf[ny * $stride + nx];
            if $v0 as i32 + v1 as i32 == 255 {
                let id1 = ny as u32 * $w + nx as u32;
                let rep1_root = $uf.find(id1);
                if $uf.root_size(rep1_root) >= $min_component_size {
                    let rep0 = $rep0 as u64;
                    let rep1 = rep1_root as u64;
                    let key = if rep0 < rep1 {
                        (rep0 << 32) | rep1
                    } else {
                        (rep1 << 32) | rep0
                    };
                    let gx = $dx as i16 * (v1 as i16 - $v0 as i16);
                    let gy = $dy as i16 * (v1 as i16 - $v0 as i16);
                    let pt = Pt {
                        x: (2 * $x as i32 + $dx) as u16,
                        y: (2 * $y as i32 + $dy) as u16,
                        gx,
                        gy,
                        slope: 0,
                    };
                    $map.insert(key, pt);
                    true
                } else {
                    false
                }
            } else {
                false
            }
        }};
    }

    // Loop over interior pixels only (skip border). All neighbor offsets
    // (dx,dy) ∈ {-1,0,1} are guaranteed in-bounds, eliminating bounds checks.
    // The C reference uses the same range restriction.
    for y in 1..h.saturating_sub(1) {
        let row_off = y as usize * stride;
        let mut connected_last = false;
        for x in 1..w.saturating_sub(1) {
            let v0 = buf[row_off + x as usize];
            if v0 == 127 {
                connected_last = false;
                continue;
            }

            // Compute rep0 once for this pixel, reuse across all do_conn! calls
            let rep0 = uf.find(y * w + x);
            if uf.root_size(rep0) < min_component_size {
                connected_last = false;
                continue;
            }

            // 4-connectivity
            do_conn!(
                cluster_map,
                uf,
                buf,
                stride,
                x,
                y,
                rep0,
                v0,
                1,
                0,
                w,
                min_component_size
            );
            do_conn!(
                cluster_map,
                uf,
                buf,
                stride,
                x,
                y,
                rep0,
                v0,
                0,
                1,
                w,
                min_component_size
            );

            // 8-connectivity with deduplication: checking (1,1) on the
            // previous pixel and (-1,1) on the current pixel produces the
            // same midpoint. Only check (-1,1) when the previous pixel
            // did not connect via (1,1).
            if !connected_last {
                do_conn!(
                    cluster_map,
                    uf,
                    buf,
                    stride,
                    x,
                    y,
                    rep0,
                    v0,
                    -1,
                    1,
                    w,
                    min_component_size
                );
            }
            connected_last = do_conn!(
                cluster_map,
                uf,
                buf,
                stride,
                x,
                y,
                rep0,
                v0,
                1,
                1,
                w,
                min_component_size
            );
        }
    }

    // Collect clusters that meet the minimum size threshold.
    // Swap out the Vec<Pt> so the entry retains an empty Vec for recycling.
    let mut clusters: Vec<Cluster> = Vec::new();
    for entry in &mut cluster_map.entries {
        if entry.points.len() >= min_cluster_size as usize {
            let points = std::mem::take(&mut entry.points);
            clusters.push(Cluster { points });
        }
    }

    // Sort by descending size for determinism
    clusters.sort_by(|a, b| b.points.len().cmp(&a.points.len()));

    clusters
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::detect::connected::connected_components;
    use crate::detect::unionfind::UnionFind;

    fn make_thresh(w: u32, h: u32, pixels: &[u8]) -> ImageU8 {
        ImageU8::from_buf(w, h, w, pixels.to_vec())
    }

    fn run_cc(img: &ImageU8) -> UnionFind {
        let mut uf = UnionFind::empty();
        connected_components(img, &mut uf);
        uf
    }

    #[test]
    fn no_clusters_in_uniform_image() {
        let img = make_thresh(8, 8, &vec![0u8; 64]);
        let mut uf = run_cc(&img);
        let clusters = gradient_clusters(&img, &mut uf, 5, &mut ClusterMap::default());
        assert!(clusters.is_empty());
    }

    #[test]
    fn clusters_at_black_white_boundary() {
        // Left half black, right half white
        let mut pixels = vec![0u8; 64];
        for y in 0..8 {
            for x in 4..8 {
                pixels[y * 8 + x] = 255;
            }
        }
        let img = make_thresh(8, 8, &pixels);
        let mut uf = run_cc(&img);
        let clusters = gradient_clusters(&img, &mut uf, 1, &mut ClusterMap::new());
        assert!(!clusters.is_empty());
    }

    #[test]
    fn gradient_direction_correct() {
        // Simple 2-column image: left black, right white
        let mut pixels = vec![0u8; 64]; // 8x8
        for y in 0..8 {
            for x in 4..8 {
                pixels[y * 8 + x] = 255;
            }
        }
        let img = make_thresh(8, 8, &pixels);
        let mut uf = run_cc(&img);
        let clusters = gradient_clusters(&img, &mut uf, 1, &mut ClusterMap::new());

        // Find a point on the boundary x=3→4 (dx=1)
        let boundary_pts: Vec<&Pt> = clusters
            .iter()
            .flat_map(|c| c.points.iter())
            .filter(|p| p.x == 7) // 2*3 + 1 = 7
            .collect();

        assert!(!boundary_pts.is_empty());
        // gx = dx * (v1 - v0) = 1 * (255 - 0) = 255
        for p in &boundary_pts {
            assert_eq!(p.gx, 255);
        }
    }

    #[test]
    fn small_components_filtered_out() {
        // Single white pixel in sea of black → component too small
        let mut pixels = vec![0u8; 100]; // 10x10
        pixels[55] = 255; // one pixel
        let img = make_thresh(10, 10, &pixels);
        let mut uf = run_cc(&img);
        let clusters = gradient_clusters(&img, &mut uf, 1, &mut ClusterMap::new());
        // White component has only 1 pixel, below threshold of 25
        assert!(clusters.is_empty());
    }

    #[test]
    fn no_duplicate_midpoints_on_horizontal_boundary() {
        // A horizontal boundary (top black, bottom white) causes duplicate
        // midpoints when pixel (x,y) checks (1,1) → (x+1,y+1) and then
        // pixel (x+1,y) checks (-1,1) → (x,y+1). Both produce a midpoint
        // at the same (2x+1, 2y+1) coordinates. The C reference avoids
        // this with `connected_last` tracking.
        let size = 40u32;
        let mut pixels = vec![0u8; (size * size) as usize];
        // Bottom half white
        for y in (size / 2)..size {
            for x in 0..size {
                pixels[(y * size + x) as usize] = 255;
            }
        }
        let img = make_thresh(size, size, &pixels);
        let mut uf = run_cc(&img);
        let clusters = gradient_clusters(&img, &mut uf, 1, &mut ClusterMap::new());

        // Collect all midpoints across all clusters
        let mut all_points: Vec<(u16, u16)> = clusters
            .iter()
            .flat_map(|c| c.points.iter())
            .map(|p| (p.x, p.y))
            .collect();
        let total = all_points.len();
        all_points.sort();
        all_points.dedup();
        let unique = all_points.len();

        assert_eq!(
            total,
            unique,
            "found {} duplicate midpoints out of {} total",
            total - unique,
            total
        );
    }

    #[test]
    fn small_clusters_filtered_by_min_cluster_size() {
        // Left half black, right half white in a large-enough image
        let size = 40u32;
        let mut pixels = vec![0u8; (size * size) as usize];
        for y in 0..size {
            for x in (size / 2)..size {
                pixels[(y * size + x) as usize] = 255;
            }
        }
        let img = make_thresh(size, size, &pixels);
        let mut uf = run_cc(&img);

        // With min_cluster_size=1, we get clusters
        let all = gradient_clusters(&img, &mut uf, 1, &mut ClusterMap::new());
        assert!(!all.is_empty());

        // With a very high threshold, all clusters are filtered out
        let mut uf2 = run_cc(&img);
        let filtered = gradient_clusters(&img, &mut uf2, 100_000, &mut ClusterMap::new());
        assert!(filtered.is_empty());
    }

    #[test]
    fn unknown_pixels_ignored() {
        let mut pixels = vec![127u8; 64];
        // Set a small region to black/white
        for y in 0..4 {
            for x in 0..4 {
                pixels[y * 8 + x] = 0;
            }
        }
        let img = make_thresh(8, 8, &pixels);
        let mut uf = run_cc(&img);
        let clusters = gradient_clusters(&img, &mut uf, 1, &mut ClusterMap::new());
        // No boundary between black and white (only black and unknown)
        assert!(clusters.is_empty());
    }
}
