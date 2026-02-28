/// Reference C apriltag3 implementation via FFI.
///
/// This module is only available when the `reference` feature is enabled.
/// Requires running `scripts/fetch-references.sh` to obtain the C source.
use apriltag::detect::image::ImageU8;

/// A detection result from the reference C implementation.
#[derive(Debug, Clone)]
pub struct ReferenceDetection {
    pub id: i32,
    pub hamming: i32,
    pub decision_margin: f32,
    pub corners: [[f64; 2]; 4],
    pub center: [f64; 2],
}

/// Configuration for the reference detector.
pub struct ReferenceConfig {
    pub quad_decimate: f32,
    pub nthreads: i32,
}

impl Default for ReferenceConfig {
    fn default() -> Self {
        Self {
            quad_decimate: 2.0,
            nthreads: 1,
        }
    }
}

#[repr(C)]
struct BenchDetection {
    id: i32,
    hamming: i32,
    decision_margin: f32,
    corners: [f64; 8],
    center: [f64; 2],
}

extern "C" {
    fn bench_reference_detect(
        buf: *const u8,
        width: i32,
        height: i32,
        stride: i32,
        family: *const std::ffi::c_char,
        quad_decimate: f32,
        nthreads: i32,
        out_count: *mut i32,
    ) -> *mut BenchDetection;

    fn bench_free_detections(detections: *mut BenchDetection);
}

/// Detect tags using the reference C implementation.
///
/// # Safety
/// This calls into the C apriltag3 library. The image data must be valid.
pub fn reference_detect(
    img: &ImageU8,
    family: &str,
    config: &ReferenceConfig,
) -> Vec<ReferenceDetection> {
    let family_cstr = std::ffi::CString::new(family).expect("family name contains null byte");

    let mut count: i32 = 0;

    let raw = unsafe {
        bench_reference_detect(
            img.data.as_ptr(),
            img.width as i32,
            img.height as i32,
            img.stride as i32,
            family_cstr.as_ptr(),
            config.quad_decimate,
            config.nthreads,
            &mut count,
        )
    };

    if raw.is_null() || count <= 0 {
        return Vec::new();
    }

    let mut results = Vec::with_capacity(count as usize);
    for i in 0..count as usize {
        let det = unsafe { &*raw.add(i) };
        results.push(ReferenceDetection {
            id: det.id,
            hamming: det.hamming,
            decision_margin: det.decision_margin,
            corners: [
                [det.corners[0], det.corners[1]],
                [det.corners[2], det.corners[3]],
                [det.corners[4], det.corners[5]],
                [det.corners[6], det.corners[7]],
            ],
            center: det.center,
        });
    }

    unsafe {
        bench_free_detections(raw);
    }

    results
}
