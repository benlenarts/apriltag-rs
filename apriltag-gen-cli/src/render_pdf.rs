//! PDF rendering for individual tags and mosaics.

use anyhow::{Context, Result};
use apriltag_gen::family::TagFamily;
use apriltag_gen::render::{self, RenderedTag};
use apriltag_gen::types::Pixel;
use printpdf::*;

/// Default tag cell size in mm
const CELL_SIZE_MM: f32 = 2.0;

/// Write a single tag as a PDF file.
pub fn write_tag_pdf(tag: &RenderedTag, border: usize, path: &str) -> Result<()> {
    let total_cells = tag.grid_size + 2 * border;
    let page_size_mm = total_cells as f32 * CELL_SIZE_MM + 20.0; // 10mm margin each side

    let (doc, page1, layer1) =
        PdfDocument::new("AprilTag", Mm(page_size_mm), Mm(page_size_mm), "Tag");
    let layer = doc.get_page(page1).get_layer(layer1);

    let margin_mm = (page_size_mm - total_cells as f32 * CELL_SIZE_MM) / 2.0;
    draw_tag(&layer, tag, border, margin_mm, margin_mm, CELL_SIZE_MM);

    doc.save(&mut std::io::BufWriter::new(
        std::fs::File::create(path).with_context(|| format!("creating {path}"))?,
    ))
    .with_context(|| format!("writing PDF to {path}"))?;

    Ok(())
}

/// Write a mosaic of all tags in a family as a PDF (A4 pages).
pub fn write_mosaic_pdf(
    family: &TagFamily,
    spacing: usize,
    columns: usize,
    path: &str,
) -> Result<()> {
    let ncodes = family.codes.len();
    let cols = columns.min(ncodes);

    let tag_cells = family.layout.grid_size + 2; // 1 cell border
    let cell_mm = CELL_SIZE_MM;
    let spacing_mm = spacing as f32 * cell_mm;
    let tag_mm = tag_cells as f32 * cell_mm;

    // A4 page
    let page_w_mm: f32 = 210.0;
    let page_h_mm: f32 = 297.0;
    let margin_mm: f32 = 10.0;

    let usable_h = page_h_mm - 2.0 * margin_mm;

    // Compute how many rows fit per page
    let rows_per_page = ((usable_h + spacing_mm) / (tag_mm + spacing_mm)).floor() as usize;
    let rows_per_page = rows_per_page.max(1);

    let total_rows = ncodes.div_ceil(cols);
    let total_pages = total_rows.div_ceil(rows_per_page);

    let (doc, first_page, first_layer) =
        PdfDocument::new("AprilTag Mosaic", Mm(page_w_mm), Mm(page_h_mm), "Page 1");

    for page_idx in 0..total_pages {
        let layer = if page_idx == 0 {
            doc.get_page(first_page).get_layer(first_layer)
        } else {
            let (p, l) = doc.add_page(
                Mm(page_w_mm),
                Mm(page_h_mm),
                format!("Page {}", page_idx + 1),
            );
            doc.get_page(p).get_layer(l)
        };

        let start_row = page_idx * rows_per_page;
        let end_row = total_rows.min(start_row + rows_per_page);

        for row in start_row..end_row {
            let local_row = row - start_row;
            for col in 0..cols {
                let idx = row * cols + col;
                if idx >= ncodes {
                    break;
                }

                let tag = render::render(&family.layout, family.codes[idx]);
                let x_mm = margin_mm + col as f32 * (tag_mm + spacing_mm);
                // PDF coordinates are bottom-up; place first row at top
                let y_mm = page_h_mm
                    - margin_mm
                    - (local_row + 1) as f32 * tag_mm
                    - local_row as f32 * spacing_mm;

                draw_tag(&layer, &tag, 1, x_mm, y_mm, cell_mm);
            }
        }
    }

    doc.save(&mut std::io::BufWriter::new(
        std::fs::File::create(path).with_context(|| format!("creating {path}"))?,
    ))
    .with_context(|| format!("writing PDF to {path}"))?;

    Ok(())
}

/// Draw a tag on a PDF layer at the given position.
fn draw_tag(
    layer: &PdfLayerReference,
    tag: &RenderedTag,
    border: usize,
    x_mm: f32,
    y_mm: f32,
    cell_mm: f32,
) {
    let size = tag.grid_size;

    // Draw white border background
    let total = size + 2 * border;
    layer.set_fill_color(Color::Rgb(Rgb::new(1.0, 1.0, 1.0, None)));
    layer.set_outline_color(Color::Rgb(Rgb::new(1.0, 1.0, 1.0, None)));
    let rect = Rect::new(
        Mm(x_mm),
        Mm(y_mm),
        Mm(x_mm + total as f32 * cell_mm),
        Mm(y_mm + total as f32 * cell_mm),
    );
    layer.add_rect(rect);

    // Draw each cell
    for cy in 0..size {
        for cx in 0..size {
            let pixel = tag.pixel(cx, cy);
            let color = match pixel {
                Pixel::Black => Some(Color::Rgb(Rgb::new(0.0, 0.0, 0.0, None))),
                Pixel::White => Some(Color::Rgb(Rgb::new(1.0, 1.0, 1.0, None))),
                Pixel::Transparent => None,
            };

            if let Some(c) = color {
                layer.set_fill_color(c.clone());
                layer.set_outline_color(c);
                // PDF y is bottom-up: row 0 is at the top
                let px = x_mm + (cx + border) as f32 * cell_mm;
                let py = y_mm + (size - 1 - cy + border) as f32 * cell_mm;
                let rect = Rect::new(Mm(px), Mm(py), Mm(px + cell_mm), Mm(py + cell_mm));
                layer.add_rect(rect);
            }
        }
    }
}
