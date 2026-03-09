#[cfg(feature = "parallel")]
use rayon::prelude::*;

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

    /// Create a new ClusterMap pre-sized for `n_buckets`.
    #[cfg(feature = "parallel")]
    fn with_capacity(n_buckets: usize) -> Self {
        Self {
            buckets: vec![EMPTY; n_buckets],
            entries: Vec::new(),
            free_vecs: Vec::new(),
        }
    }

    /// Collect clusters meeting the minimum size threshold, returning
    /// `(key, points)` pairs so strips can be merged by key.
    #[cfg(feature = "parallel")]
    fn collect_keyed(&mut self, min_cluster_size: u32) -> Vec<(u64, Vec<Pt>)> {
        let mut result = Vec::new();
        for entry in &mut self.entries {
            if entry.points.len() >= min_cluster_size as usize {
                result.push((entry.key, std::mem::take(&mut entry.points)));
            }
        }
        result
    }

    /// Recycle point Vecs from consumed clusters back into the free pool.
    pub fn recycle_clusters(&mut self, clusters: &mut Vec<Cluster>) {
        for cluster in clusters.drain(..) {
            let mut points = cluster.points;
            points.clear();
            self.free_vecs.push(points);
        }
    }

    #[inline(always)]
    fn bucket_index(&self, key: u64) -> usize {
        // FxHash-style multiply for u64 keys
        let hash = key.wrapping_mul(0x517cc1b727220a95);
        (hash as usize) % self.buckets.len()
    }
}

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
            let rep1_root = $uf.find_flat(id1);
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

/// Scan rows `y0..y1` and insert boundary points into `cluster_map`.
///
/// Requires [`UnionFind::flatten`] to have been called so that
/// [`find_flat`](UnionFind::find_flat) returns the correct root.
fn scan_rows(
    buf: &[u8],
    stride: usize,
    w: u32,
    y0: u32,
    y1: u32,
    uf: &UnionFind,
    cluster_map: &mut ClusterMap,
) {
    let min_component_size = 25u32;
    for y in y0..y1 {
        let row_off = y as usize * stride;
        let mut connected_last = false;
        for x in 1..w.saturating_sub(1) {
            let v0 = buf[row_off + x as usize];
            if v0 == 127 {
                connected_last = false;
                continue;
            }

            let rep0 = uf.find_flat(y * w + x);
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

            // 8-connectivity with deduplication
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
    out: &mut Vec<Cluster>,
) {
    let w = threshed.width;
    let h = threshed.height;

    // Flatten the union-find so find_flat() works in O(1)
    uf.flatten();

    let buf = &threshed.buf;
    let stride = threshed.stride as usize;

    let y_start = 1u32;
    let y_end = h.saturating_sub(1);

    // COVERAGE: parallel feature block — only compiled with --features parallel
    #[cfg(feature = "parallel")]
    {
        let _ = cluster_map; // per-thread maps used instead
                             // Split image into horizontal strips, one per rayon task.
                             // Each strip gets its own ClusterMap, then we merge results.
        let n_rows = y_end.saturating_sub(y_start) as usize;
        if n_rows == 0 {
            out.clear();
            return;
        }

        // Use ~4 tasks per rayon thread, minimum 64 rows per chunk
        let n_threads = rayon::current_num_threads();
        let rows_per_chunk = (n_rows / (n_threads * 4)).max(64).min(n_rows);
        let n_chunks = n_rows.div_ceil(rows_per_chunk);
        let n_buckets = ((w as usize * rows_per_chunk) / 5).max(16);

        // Parallel scan: each chunk produces keyed clusters (key, points)
        let chunk_results: Vec<Vec<(u64, Vec<Pt>)>> = (0..n_chunks)
            .into_par_iter()
            .map_init(
                || ClusterMap::with_capacity(n_buckets),
                |local_map, chunk_idx| {
                    let cy0 = y_start + (chunk_idx * rows_per_chunk) as u32;
                    let cy1 = y_end.min(cy0 + rows_per_chunk as u32);
                    local_map.reset(n_buckets);
                    scan_rows(buf, stride, w, cy0, cy1, uf, local_map);
                    // Collect all clusters (even small ones) so merging
                    // can combine strips that individually are below threshold
                    local_map.collect_keyed(1)
                },
            )
            .collect();

        // Merge clusters from different strips with the same key
        merge_strip_clusters(chunk_results, min_cluster_size, out);
    }

    #[cfg(not(feature = "parallel"))]
    {
        let n_buckets = ((w as usize * h as usize) / 5).max(16);
        cluster_map.reset(n_buckets);
        scan_rows(buf, stride, w, y_start, y_end, uf, cluster_map);

        // Collect clusters that meet the minimum size threshold.
        out.clear();
        for entry in &mut cluster_map.entries {
            if entry.points.len() >= min_cluster_size as usize {
                let points = std::mem::take(&mut entry.points);
                out.push(Cluster { points });
            }
        }
    }

    // Sort by descending size for determinism
    out.sort_by(|a, b| b.points.len().cmp(&a.points.len()));
}

/// Merge keyed clusters from multiple strips. Clusters with the same key
/// (component-pair) across strips get their points concatenated, then
/// the combined cluster is kept only if it meets `min_cluster_size`.
#[cfg(feature = "parallel")]
fn merge_strip_clusters(
    chunk_results: Vec<Vec<(u64, Vec<Pt>)>>,
    min_cluster_size: u32,
    out: &mut Vec<Cluster>,
) {
    use std::collections::HashMap;

    out.clear();

    // Fast path: single chunk, no merging needed
    if chunk_results.len() <= 1 {
        if let Some(keyed) = chunk_results.into_iter().next() {
            for (_, points) in keyed {
                if points.len() >= min_cluster_size as usize {
                    out.push(Cluster { points });
                }
            }
        }
        return;
    }

    // Merge by key: same component-pair boundary may appear in multiple strips
    let total_entries: usize = chunk_results.iter().map(|v| v.len()).sum();
    let mut map: HashMap<u64, Vec<Pt>> = HashMap::with_capacity(total_entries);

    for keyed_clusters in chunk_results {
        for (key, points) in keyed_clusters {
            map.entry(key)
                .and_modify(|existing| existing.extend_from_slice(&points))
                .or_insert(points);
        }
    }

    for (_, points) in map {
        if points.len() >= min_cluster_size as usize {
            out.push(Cluster { points });
        }
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
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
        let mut clusters = Vec::new();
        gradient_clusters(&img, &mut uf, 5, &mut ClusterMap::default(), &mut clusters);
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
        let mut clusters = Vec::new();
        gradient_clusters(&img, &mut uf, 1, &mut ClusterMap::new(), &mut clusters);
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
        let mut clusters = Vec::new();
        gradient_clusters(&img, &mut uf, 1, &mut ClusterMap::new(), &mut clusters);

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
        let mut clusters = Vec::new();
        gradient_clusters(&img, &mut uf, 1, &mut ClusterMap::new(), &mut clusters);
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
        let mut clusters = Vec::new();
        gradient_clusters(&img, &mut uf, 1, &mut ClusterMap::new(), &mut clusters);

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

        // no duplicate midpoints
        assert_eq!(total, unique);
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
        let mut all = Vec::new();
        gradient_clusters(&img, &mut uf, 1, &mut ClusterMap::new(), &mut all);
        assert!(!all.is_empty());

        // With a very high threshold, all clusters are filtered out
        let mut uf2 = run_cc(&img);
        let mut filtered = Vec::new();
        gradient_clusters(
            &img,
            &mut uf2,
            100_000,
            &mut ClusterMap::new(),
            &mut filtered,
        );
        assert!(filtered.is_empty());
    }

    #[test]
    fn hash_collision_chain_walk() {
        // Exercise the chain-walk path (lines 88-90) in ClusterMap::insert by
        // inserting two distinct keys that hash to the same bucket.
        let mut map = ClusterMap::new();
        let n_buckets = 16usize;
        map.reset(n_buckets);

        // Find two keys that collide: both hash to bucket 0.
        // hash(key) = key.wrapping_mul(0x517cc1b727220a95) % n_buckets
        let key_a = 0u64;
        let mut key_b = 1u64;
        let target = (key_a.wrapping_mul(0x517cc1b727220a95) as usize) % n_buckets;
        loop {
            let h = (key_b.wrapping_mul(0x517cc1b727220a95) as usize) % n_buckets;
            if h == target {
                break;
            }
            key_b += 1;
        }

        let pt = Pt {
            x: 10,
            y: 20,
            gx: 1,
            gy: 0,
            slope: 0,
        };
        map.insert(key_a, pt);
        // Second insert with a different key in the same bucket forces the
        // chain walk (while loop iterates past key_a's entry before creating
        // a new entry for key_b).
        map.insert(key_b, pt);
        // Also insert into key_a again to walk past key_b in the chain.
        map.insert(key_a, pt);

        // key_a should have 2 points, key_b should have 1
        let a_entry = map.entries.iter().find(|e| e.key == key_a).unwrap();
        let b_entry = map.entries.iter().find(|e| e.key == key_b).unwrap();
        assert_eq!(a_entry.points.len(), 2);
        assert_eq!(b_entry.points.len(), 1);
    }

    #[test]
    fn cluster_map_reset_recycles_entries() {
        // Exercise ClusterMap::reset recycling path (lines 67-70):
        // after a frame with populated entries, reset should drain
        // entries and push their point Vecs into free_vecs.
        let mut map = ClusterMap::new();
        map.reset(16);

        let pt = Pt {
            x: 10,
            y: 20,
            gx: 1,
            gy: 0,
            slope: 0,
        };
        map.insert(0, pt);
        map.insert(1, pt);
        assert_eq!(map.entries.len(), 2);

        // Reset should drain entries and recycle their Vecs
        map.reset(16);
        assert_eq!(map.entries.len(), 0);
        assert_eq!(map.free_vecs.len(), 2);
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
        let mut clusters = Vec::new();
        gradient_clusters(&img, &mut uf, 1, &mut ClusterMap::new(), &mut clusters);
        // No boundary between black and white (only black and unknown)
        assert!(clusters.is_empty());
    }
}
