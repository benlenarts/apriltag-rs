//! Compare allocation counts between fresh buffers and reused buffers.
//!
//! Run with: cargo run -p apriltag --example alloc_comparison --release

use std::alloc::{GlobalAlloc, Layout, System};
use std::sync::atomic::{AtomicUsize, Ordering};

struct CountingAlloc;
static ALLOC_COUNT: AtomicUsize = AtomicUsize::new(0);
static ALLOC_BYTES: AtomicUsize = AtomicUsize::new(0);

unsafe impl GlobalAlloc for CountingAlloc {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        ALLOC_COUNT.fetch_add(1, Ordering::Relaxed);
        ALLOC_BYTES.fetch_add(layout.size(), Ordering::Relaxed);
        unsafe { System.alloc(layout) }
    }
    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        unsafe { System.dealloc(ptr, layout) }
    }
}

#[global_allocator]
static A: CountingAlloc = CountingAlloc;

fn reset() {
    ALLOC_COUNT.store(0, Ordering::SeqCst);
    ALLOC_BYTES.store(0, Ordering::SeqCst);
}

fn snapshot() -> (usize, usize) {
    (
        ALLOC_COUNT.load(Ordering::SeqCst),
        ALLOC_BYTES.load(Ordering::SeqCst),
    )
}

fn main() {
    use apriltag::detect::detector::{Detector, DetectorBuffers};
    use apriltag::detect::image::ImageU8;
    use apriltag::family;
    use apriltag::types::Pixel;

    // Build 640x480 image with a tag
    let fam = family::tag36h11();
    let rendered = fam.tag(0).render();
    let (w, h) = (640u32, 480u32);
    let mut img = ImageU8::new(w, h);
    for y in 0..h {
        for x in 0..w {
            img.set(x, y, 255);
        }
    }
    let scale = 40u32;
    let tag_px = rendered.grid_size as u32 * scale;
    let ox = (w - tag_px) / 2;
    let oy = (h - tag_px) / 2;
    for ty in 0..rendered.grid_size {
        for tx in 0..rendered.grid_size {
            let val = match rendered.pixel(tx, ty) {
                Pixel::Black => 0u8,
                Pixel::White | Pixel::Transparent => 255u8,
            };
            for dy in 0..scale {
                for dx in 0..scale {
                    img.set(
                        ox + tx as u32 * scale + dx,
                        oy + ty as u32 * scale + dy,
                        val,
                    );
                }
            }
        }
    }

    let detector = Detector::builder()
        .quad_sigma(0.8)
        .add_family(fam, 2)
        .build();

    // --- detect() with fresh buffers ---
    println!("=== detect() with fresh buffers each call ===");
    for i in 0..3 {
        reset();
        let dets = detector.detect(&img, &mut DetectorBuffers::new());
        let (count, bytes) = snapshot();
        println!(
            "  call {}: {} allocs, {} bytes ({:.1} KB), {} detections",
            i + 1,
            count,
            bytes,
            bytes as f64 / 1024.0,
            dets.len()
        );
    }

    // --- detect() reusing buffers ---
    println!("\n=== detect() reusing buffers ===");
    let mut buffers = DetectorBuffers::new();
    for i in 0..3 {
        reset();
        let dets = detector.detect(&img, &mut buffers);
        let (count, bytes) = snapshot();
        println!(
            "  call {}: {} allocs, {} bytes ({:.1} KB), {} detections",
            i + 1,
            count,
            bytes,
            bytes as f64 / 1024.0,
            dets.len()
        );
    }
}
