//! PNG rendering for individual tags and mosaics.

use anyhow::{Context, Result};
use apriltag_gen::family::TagFamily;
use apriltag_gen::render::{self, RenderedTag};
use apriltag_gen::types::Pixel;
use std::path::Path;

/// Write a single tag as a PNG file with the given scale and border.
pub fn write_tag_png(tag: &RenderedTag, scale: usize, border: usize, path: &Path) -> Result<()> {
    let img = tag_to_image(tag, scale, border);
    let (width, height) = (img.width, img.height);
    write_grayscale_png(path, &img.pixels, width, height)
}

/// Write a mosaic of all tags in a family as a PNG.
pub fn write_mosaic_png(
    family: &TagFamily,
    scale: usize,
    spacing: usize,
    columns: usize,
    output_path: &str,
) -> Result<()> {
    let ncodes = family.codes.len();
    let cols = columns.min(ncodes);
    let rows = ncodes.div_ceil(cols);

    // Compute cell dimensions (tag + border)
    let tag_img_size = (family.layout.grid_size + 2) * scale; // 1-cell border on each side
    let spacing_px = spacing * scale;

    let img_width = cols * tag_img_size + (cols.saturating_sub(1)) * spacing_px;
    let img_height = rows * tag_img_size + (rows.saturating_sub(1)) * spacing_px;

    // White background
    let mut pixels = vec![255u8; img_width * img_height];

    for (idx, &code) in family.codes.iter().enumerate() {
        let col = idx % cols;
        let row = idx / cols;
        let x_off = col * (tag_img_size + spacing_px);
        let y_off = row * (tag_img_size + spacing_px);

        let tag = render::render(&family.layout, code);
        let img = tag_to_image(&tag, scale, 1);

        // Blit tag image into mosaic
        for y in 0..img.height {
            for x in 0..img.width {
                let dst_x = x_off + x;
                let dst_y = y_off + y;
                if dst_x < img_width && dst_y < img_height {
                    pixels[dst_y * img_width + dst_x] = img.pixels[y * img.width + x];
                }
            }
        }
    }

    write_grayscale_png(Path::new(output_path), &pixels, img_width, img_height)
}

struct GrayImage {
    pixels: Vec<u8>,
    width: usize,
    height: usize,
}

/// Convert a RenderedTag to a grayscale image with scale and border.
fn tag_to_image(tag: &RenderedTag, scale: usize, border: usize) -> GrayImage {
    let size = tag.grid_size + 2 * border;
    let img_size = size * scale;
    let mut pixels = vec![255u8; img_size * img_size]; // white background

    for y in 0..tag.grid_size {
        for x in 0..tag.grid_size {
            let pixel = tag.pixel(x, y);
            let gray = match pixel {
                Pixel::Black => 0u8,
                Pixel::White => 255u8,
                Pixel::Transparent => 255u8, // transparent renders as white
            };

            // Scale and offset by border
            let ox = (x + border) * scale;
            let oy = (y + border) * scale;
            for sy in 0..scale {
                for sx in 0..scale {
                    pixels[(oy + sy) * img_size + (ox + sx)] = gray;
                }
            }
        }
    }

    GrayImage {
        pixels,
        width: img_size,
        height: img_size,
    }
}

fn write_grayscale_png(path: &Path, pixels: &[u8], width: usize, height: usize) -> Result<()> {
    let file = std::fs::File::create(path)
        .with_context(|| format!("creating {}", path.display()))?;
    let w = std::io::BufWriter::new(file);

    let mut encoder = png::Encoder::new(w, width as u32, height as u32);
    encoder.set_color(png::ColorType::Grayscale);
    encoder.set_depth(png::BitDepth::Eight);

    let mut writer = encoder
        .write_header()
        .with_context(|| format!("writing PNG header for {}", path.display()))?;
    writer
        .write_image_data(pixels)
        .with_context(|| format!("writing PNG data for {}", path.display()))?;

    Ok(())
}
