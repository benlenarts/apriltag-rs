use std::collections::HashMap;

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
    /// Angular sort key for quad fitting.
    pub slope: f32,
}

/// A cluster of edge points sharing the same component-pair boundary.
#[derive(Debug, Clone)]
pub struct Cluster {
    pub points: Vec<Pt>,
}

/// Extract boundary points between adjacent black/white components and group
/// them into clusters keyed by the ordered pair of component representatives.
///
/// Each cluster represents a potential quad edge.
pub fn gradient_clusters(
    threshed: &ImageU8,
    uf: &mut UnionFind,
    min_cluster_size: u32,
) -> Vec<Cluster> {
    let w = threshed.width;
    let h = threshed.height;
    let min_component_size = 25u32;

    let neighbor_offsets: [(i32, i32); 4] = [(1, 0), (0, 1), (-1, 1), (1, 1)];

    let mut cluster_map: HashMap<u64, Vec<Pt>> = HashMap::new();

    for y in 0..h {
        for x in 0..w {
            let v0 = threshed.get(x, y);
            if v0 == 127 {
                continue;
            }

            let id0 = y * w + x;
            if uf.set_size(id0) < min_component_size {
                continue;
            }

            for &(dx, dy) in &neighbor_offsets {
                let nx = x as i32 + dx;
                let ny = y as i32 + dy;

                if nx < 0 || nx >= w as i32 || ny < 0 || ny >= h as i32 {
                    continue;
                }

                let nx = nx as u32;
                let ny = ny as u32;
                let v1 = threshed.get(nx, ny);

                if v0 as i32 + v1 as i32 != 255 {
                    continue;
                }

                let id1 = ny * w + nx;
                if uf.set_size(id1) < min_component_size {
                    continue;
                }

                let rep0 = uf.find(id0) as u64;
                let rep1 = uf.find(id1) as u64;
                let key = if rep0 < rep1 {
                    (rep0 << 32) | rep1
                } else {
                    (rep1 << 32) | rep0
                };

                let gx = dx as i16 * (v1 as i16 - v0 as i16);
                let gy = dy as i16 * (v1 as i16 - v0 as i16);

                let pt = Pt {
                    x: (2 * x as i32 + dx) as u16,
                    y: (2 * y as i32 + dy) as u16,
                    gx,
                    gy,
                    slope: 0.0,
                };

                cluster_map.entry(key).or_default().push(pt);
            }
        }
    }

    // Filter by minimum cluster size and collect
    let mut clusters: Vec<Cluster> = cluster_map
        .into_values()
        .filter(|pts| pts.len() >= min_cluster_size as usize)
        .map(|points| Cluster { points })
        .collect();

    // Sort for determinism
    clusters.sort_by(|a, b| b.points.len().cmp(&a.points.len()));

    clusters
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::detect::connected::connected_components;

    fn make_thresh(w: u32, h: u32, pixels: &[u8]) -> ImageU8 {
        ImageU8::from_buf(w, h, w, pixels.to_vec())
    }

    #[test]
    fn no_clusters_in_uniform_image() {
        let img = make_thresh(8, 8, &vec![0u8; 64]);
        let mut uf = connected_components(&img);
        let clusters = gradient_clusters(&img, &mut uf, 5);
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
        let mut uf = connected_components(&img);
        let clusters = gradient_clusters(&img, &mut uf, 1);
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
        let mut uf = connected_components(&img);
        let clusters = gradient_clusters(&img, &mut uf, 1);

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
        let mut uf = connected_components(&img);
        let clusters = gradient_clusters(&img, &mut uf, 1);
        // White component has only 1 pixel, below threshold of 25
        assert!(clusters.is_empty());
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
        let mut uf = connected_components(&img);
        let clusters = gradient_clusters(&img, &mut uf, 1);
        // No boundary between black and white (only black and unknown)
        assert!(clusters.is_empty());
    }
}
