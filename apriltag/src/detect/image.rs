/// Read-only access to a grayscale image.
///
/// Implemented by both [`ImageU8`] (owned) and [`ImageRef`] (borrowed).
/// The detection pipeline accepts `&impl GrayImage` so callers can pass
/// either type without copying pixel data.
pub trait GrayImage {
    fn width(&self) -> u32;
    fn height(&self) -> u32;
    fn stride(&self) -> u32;
    fn buf(&self) -> &[u8];

    /// Get the pixel value at (x, y).
    #[inline]
    fn get(&self, x: u32, y: u32) -> u8 {
        self.buf()[(y * self.stride() + x) as usize]
    }

    /// Get a slice of the pixel data for row `y` (width pixels, ignoring stride padding).
    #[inline]
    fn row(&self, y: u32) -> &[u8] {
        let offset = (y * self.stride()) as usize;
        &self.buf()[offset..offset + self.width() as usize]
    }

    /// Returns true if bilinear interpolation at `(px, py)` and one pixel in
    /// any direction stays within bounds.
    #[inline]
    fn interpolation_safe(&self, px: f64, py: f64) -> bool {
        px >= 1.5
            && py >= 1.5
            && px <= self.width() as f64 - 1.5
            && py <= self.height() as f64 - 1.5
    }

    /// Bilinear interpolation without coordinate clamping.
    ///
    /// The caller must ensure that `(px, py)` is far enough from the image
    /// boundary that all four sample pixels are in bounds.
    #[inline]
    fn interpolate_unclamped(&self, px: f64, py: f64) -> f64 {
        let buf = self.buf();
        let stride = self.stride() as usize;
        let x = px - 0.5;
        let y = py - 0.5;
        let x0 = x.floor() as usize;
        let y0 = y.floor() as usize;
        debug_assert!(x0 < self.width() as usize - 1);
        debug_assert!(y0 < self.height() as usize - 1);
        let fx = x - x0 as f64;
        let fy = y - y0 as f64;
        let row0 = y0 * stride;
        let row1 = row0 + stride;
        let v00 = buf[row0 + x0] as f64;
        let v10 = buf[row0 + x0 + 1] as f64;
        let v01 = buf[row1 + x0] as f64;
        let v11 = buf[row1 + x0 + 1] as f64;
        v00 * (1.0 - fx) * (1.0 - fy)
            + v10 * fx * (1.0 - fy)
            + v01 * (1.0 - fx) * fy
            + v11 * fx * fy
    }

    /// Bilinear interpolation at sub-pixel coordinates with clamping.
    fn interpolate(&self, px: f64, py: f64) -> f64 {
        let x = px - 0.5;
        let y = py - 0.5;
        let x0 = x.floor() as i64;
        let y0 = y.floor() as i64;
        let x1 = x0 + 1;
        let y1 = y0 + 1;

        let fx = x - x0 as f64;
        let fy = y - y0 as f64;

        let w = self.width() as i64;
        let h = self.height() as i64;

        let clamp_x = |v: i64| v.clamp(0, w - 1) as u32;
        let clamp_y = |v: i64| v.clamp(0, h - 1) as u32;

        let v00 = self.get(clamp_x(x0), clamp_y(y0)) as f64;
        let v10 = self.get(clamp_x(x1), clamp_y(y0)) as f64;
        let v01 = self.get(clamp_x(x0), clamp_y(y1)) as f64;
        let v11 = self.get(clamp_x(x1), clamp_y(y1)) as f64;

        v00 * (1.0 - fx) * (1.0 - fy)
            + v10 * fx * (1.0 - fy)
            + v01 * (1.0 - fx) * fy
            + v11 * fx * fy
    }

    /// Copy the image data into an owned [`ImageU8`].
    fn to_image_u8(&self) -> ImageU8 {
        ImageU8::from_buf(
            self.width(),
            self.height(),
            self.stride(),
            self.buf().to_vec(),
        )
    }
}

/// A borrowed, read-only view of grayscale image data.
///
/// Use this to pass `&[u8]` pixel data into the detection pipeline without copying.
///
/// ```
/// use apriltag::detect::image::ImageRef;
/// use apriltag::detect::image::GrayImage;
///
/// let pixels = vec![0u8; 640 * 480];
/// let img = ImageRef::new(640, 480, 640, &pixels);
/// assert_eq!(img.width(), 640);
/// assert_eq!(img.height(), 480);
/// ```
#[derive(Debug, Clone, Copy)]
pub struct ImageRef<'a> {
    width: u32,
    height: u32,
    stride: u32,
    buf: &'a [u8],
}

impl<'a> ImageRef<'a> {
    /// Create a borrowed image view.
    ///
    /// `stride` must be >= `width`, and `buf` must contain at least `stride * height` bytes.
    pub fn new(width: u32, height: u32, stride: u32, buf: &'a [u8]) -> Self {
        assert!(stride >= width);
        assert!(buf.len() >= (stride * height) as usize);
        Self {
            width,
            height,
            stride,
            buf,
        }
    }
}

impl GrayImage for ImageRef<'_> {
    #[inline]
    fn width(&self) -> u32 {
        self.width
    }
    #[inline]
    fn height(&self) -> u32 {
        self.height
    }
    #[inline]
    fn stride(&self) -> u32 {
        self.stride
    }
    #[inline]
    fn buf(&self) -> &[u8] {
        self.buf
    }
}

/// Grayscale image with row-major pixel data.
///
/// ```
/// use apriltag::detect::image::ImageU8;
///
/// let mut img = ImageU8::new(100, 80);
/// assert_eq!(img.width, 100);
/// assert_eq!(img.height, 80);
/// assert_eq!(img.get(0, 0), 0); // initialized to zero
///
/// img.set(10, 20, 128);
/// assert_eq!(img.get(10, 20), 128);
/// ```
#[derive(Debug, Clone)]
pub struct ImageU8 {
    pub width: u32,
    pub height: u32,
    pub stride: u32,
    pub buf: Vec<u8>,
}

impl GrayImage for ImageU8 {
    #[inline]
    fn width(&self) -> u32 {
        self.width
    }
    #[inline]
    fn height(&self) -> u32 {
        self.height
    }
    #[inline]
    fn stride(&self) -> u32 {
        self.stride
    }
    #[inline]
    fn buf(&self) -> &[u8] {
        &self.buf
    }
}

impl ImageU8 {
    /// Create a new image filled with zeros.
    pub fn new(width: u32, height: u32) -> Self {
        let stride = width;
        let buf = vec![0u8; (stride * height) as usize];
        Self {
            width,
            height,
            stride,
            buf,
        }
    }

    /// Create an image from pixel data where stride equals width.
    ///
    /// This is the common case — use [`from_buf`](Self::from_buf) when stride differs from width.
    pub fn from_pixels(width: u32, height: u32, buf: Vec<u8>) -> Self {
        Self::from_buf(width, height, width, buf)
    }

    /// Create an image from existing pixel data.
    ///
    /// `stride` must be >= `width`, and `buf` must contain at least `stride * height` bytes.
    pub fn from_buf(width: u32, height: u32, stride: u32, buf: Vec<u8>) -> Self {
        assert!(stride >= width);
        assert!(buf.len() >= (stride * height) as usize);
        Self {
            width,
            height,
            stride,
            buf,
        }
    }

    /// Get the pixel value at (x, y).
    #[inline]
    pub fn get(&self, x: u32, y: u32) -> u8 {
        self.buf[(y * self.stride + x) as usize]
    }

    /// Get a slice of the pixel data for row `y` (width pixels, ignoring stride padding).
    #[inline]
    pub fn row(&self, y: u32) -> &[u8] {
        let offset = (y * self.stride) as usize;
        &self.buf[offset..offset + self.width as usize]
    }

    /// Set the pixel value at (x, y).
    #[inline]
    pub fn set(&mut self, x: u32, y: u32, val: u8) {
        self.buf[(y * self.stride + x) as usize] = val;
    }

    /// Create a zeroed image, reusing an existing buffer to avoid allocation.
    ///
    /// The buffer is cleared and resized to fit `width * height` pixels.
    /// If the buffer already has sufficient capacity, no allocation occurs.
    pub fn new_reuse(width: u32, height: u32, mut buf: Vec<u8>) -> Self {
        let len = (width * height) as usize;
        buf.clear();
        buf.resize(len, 0);
        Self {
            width,
            height,
            stride: width,
            buf,
        }
    }

    /// Reconfigure for new dimensions, reusing the allocation. Clears pixel data.
    pub fn reshape(&mut self, width: u32, height: u32) {
        let len = (width * height) as usize;
        self.buf.clear();
        self.buf.resize(len, 0);
        self.width = width;
        self.height = height;
        self.stride = width;
    }

    /// Consume the image and return the backing buffer for reuse.
    pub fn into_buf(self) -> Vec<u8> {
        self.buf
    }

    /// Returns true if bilinear interpolation at `(px, py)` and one pixel in
    /// any direction stays within bounds.
    #[inline]
    pub fn interpolation_safe(&self, px: f64, py: f64) -> bool {
        GrayImage::interpolation_safe(self, px, py)
    }

    /// Bilinear interpolation without coordinate clamping.
    #[inline]
    pub fn interpolate_unclamped(&self, px: f64, py: f64) -> f64 {
        GrayImage::interpolate_unclamped(self, px, py)
    }

    /// Bilinear interpolation at sub-pixel coordinates with clamping.
    pub fn interpolate(&self, px: f64, py: f64) -> f64 {
        GrayImage::interpolate(self, px, py)
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use super::*;

    #[test]
    fn image_ref_new() {
        let data = vec![1, 2, 3, 0, 4, 5, 6, 0];
        let img = ImageRef::new(3, 2, 4, &data);
        assert_eq!(img.width(), 3);
        assert_eq!(img.height(), 2);
        assert_eq!(img.stride(), 4);
        assert_eq!(img.buf().len(), 8);
    }

    #[test]
    fn image_ref_trait_methods() {
        let data = vec![1, 2, 3, 0, 4, 5, 6, 0];
        let img = ImageRef::new(3, 2, 4, &data);
        assert_eq!(img.get(0, 0), 1);
        assert_eq!(img.get(2, 0), 3);
        assert_eq!(img.get(0, 1), 4);
        assert_eq!(img.row(0), &[1, 2, 3]);
        assert_eq!(img.row(1), &[4, 5, 6]);
    }

    #[test]
    fn image_ref_to_image_u8() {
        let data = vec![10, 20, 30, 40];
        let img = ImageRef::new(2, 2, 2, &data);
        let owned = img.to_image_u8();
        assert_eq!(owned.width, 2);
        assert_eq!(owned.height, 2);
        assert_eq!(owned.buf, vec![10, 20, 30, 40]);
    }

    #[test]
    fn image_ref_interpolation() {
        let mut data = vec![0u8; 100];
        // 10x10 image, set pixel (1,1) = 100
        data[1 * 10 + 1] = 100;
        let img = ImageRef::new(10, 10, 10, &data);
        // At exact pixel center (1.5, 1.5)
        let val = img.interpolate(1.5, 1.5);
        assert!((val - 100.0).abs() < 1e-10);
    }

    #[test]
    fn image_ref_interpolate_unclamped() {
        let mut data = vec![0u8; 100];
        for y in 0..10u32 {
            for x in 0..10u32 {
                data[(y * 10 + x) as usize] = (x * 25 + y * 10) as u8;
            }
        }
        let img = ImageRef::new(10, 10, 10, &data);
        let owned = ImageU8::from_buf(10, 10, 10, data.clone());
        // Interior points should match
        let mut px = 2.0;
        while px <= 8.0 {
            let mut py = 2.0;
            while py <= 8.0 {
                let ref_val = img.interpolate_unclamped(px, py);
                let owned_val = owned.interpolate_unclamped(px, py);
                assert!((ref_val - owned_val).abs() < 1e-10);
                py += 0.37;
            }
            px += 0.37;
        }
    }

    #[test]
    fn image_ref_interpolation_safe() {
        let data = vec![0u8; 100];
        let img = ImageRef::new(10, 10, 10, &data);
        assert!(img.interpolation_safe(5.0, 5.0));
        assert!(!img.interpolation_safe(1.0, 5.0));
        assert!(!img.interpolation_safe(9.0, 5.0));
    }

    #[test]
    #[should_panic]
    fn image_ref_new_stride_too_small() {
        let data = vec![0u8; 4];
        ImageRef::new(3, 2, 2, &data); // stride < width
    }

    #[test]
    #[should_panic]
    fn image_ref_new_buf_too_small() {
        let data = vec![0u8; 3];
        ImageRef::new(2, 2, 2, &data); // buf too small
    }

    #[test]
    fn imageu8_trait_methods_match_inherent() {
        let mut img = ImageU8::new(10, 8);
        img.set(3, 4, 42);
        // Trait methods should match inherent methods
        assert_eq!(GrayImage::get(&img, 3, 4), img.get(3, 4));
        assert_eq!(GrayImage::row(&img, 4), img.row(4));
        assert_eq!(GrayImage::width(&img), img.width);
        assert_eq!(GrayImage::height(&img), img.height);
        assert_eq!(GrayImage::stride(&img), img.stride);
        assert_eq!(GrayImage::buf(&img), img.buf.as_slice());
    }

    #[test]
    fn imageu8_to_image_u8_is_copy() {
        let mut img = ImageU8::new(4, 4);
        img.set(1, 1, 99);
        let copy = img.to_image_u8();
        assert_eq!(copy.get(1, 1), 99);
        assert_eq!(copy.width, img.width);
    }

    #[test]
    fn new_creates_zeroed_image() {
        let img = ImageU8::new(10, 8);
        assert_eq!(img.width, 10);
        assert_eq!(img.height, 8);
        assert_eq!(img.stride, 10);
        assert_eq!(img.buf.len(), 80);
        assert!(img.buf.iter().all(|&b| b == 0));
    }

    #[test]
    fn get_set_pixel() {
        let mut img = ImageU8::new(4, 4);
        img.set(2, 3, 128);
        assert_eq!(img.get(2, 3), 128);
        assert_eq!(img.get(0, 0), 0);
    }

    #[test]
    fn from_pixels_sets_stride_to_width() {
        let buf = vec![1, 2, 3, 4, 5, 6];
        let img = ImageU8::from_pixels(3, 2, buf);
        assert_eq!(img.width, 3);
        assert_eq!(img.height, 2);
        assert_eq!(img.stride, 3);
        assert_eq!(img.get(0, 0), 1);
        assert_eq!(img.get(2, 1), 6);
    }

    #[test]
    fn from_buf_with_stride() {
        let buf = vec![1, 2, 3, 0, 4, 5, 6, 0]; // stride=4, width=3
        let img = ImageU8::from_buf(3, 2, 4, buf);
        assert_eq!(img.get(0, 0), 1);
        assert_eq!(img.get(2, 0), 3);
        assert_eq!(img.get(0, 1), 4);
        assert_eq!(img.get(2, 1), 6);
    }

    #[test]
    fn row_returns_pixel_slice() {
        let buf = vec![1, 2, 3, 0, 4, 5, 6, 0]; // stride=4, width=3
        let img = ImageU8::from_buf(3, 2, 4, buf);
        assert_eq!(img.row(0), &[1, 2, 3]);
        assert_eq!(img.row(1), &[4, 5, 6]);
    }

    #[test]
    fn new_reuse_produces_identical_image() {
        let fresh = ImageU8::new(10, 8);
        let reused = ImageU8::new_reuse(10, 8, Vec::new());
        assert_eq!(fresh.width, reused.width);
        assert_eq!(fresh.height, reused.height);
        assert_eq!(fresh.stride, reused.stride);
        assert_eq!(fresh.buf, reused.buf);
    }

    #[test]
    fn new_reuse_reuses_capacity() {
        let buf = Vec::with_capacity(1024);
        let img = ImageU8::new_reuse(10, 8, buf);
        assert_eq!(img.buf.len(), 80);
        assert!(img.buf.capacity() >= 1024);
    }

    #[test]
    fn new_reuse_clears_old_data() {
        let buf = vec![42u8; 200];
        let img = ImageU8::new_reuse(10, 8, buf);
        assert!(img.buf.iter().all(|&b| b == 0));
    }

    #[test]
    fn reshape_reconfigures_dimensions() {
        let mut img = ImageU8::new(10, 8);
        img.set(0, 0, 42);
        img.reshape(5, 5);
        assert_eq!(img.width, 5);
        assert_eq!(img.height, 5);
        assert_eq!(img.stride, 5);
        assert_eq!(img.buf.len(), 25);
        assert!(img.buf.iter().all(|&b| b == 0));
    }

    #[test]
    fn reshape_reuses_capacity() {
        let mut img = ImageU8::new(0, 0);
        img.buf = Vec::with_capacity(1024);
        img.reshape(10, 8);
        assert_eq!(img.buf.len(), 80);
        assert!(img.buf.capacity() >= 1024);
    }

    #[test]
    fn into_buf_returns_buffer() {
        let img = ImageU8::new(10, 8);
        let buf = img.into_buf();
        assert_eq!(buf.len(), 80);
    }

    #[test]
    fn into_buf_roundtrip_preserves_capacity() {
        let img = ImageU8::new_reuse(10, 8, Vec::with_capacity(1024));
        let buf = img.into_buf();
        assert!(buf.capacity() >= 1024);
        let img2 = ImageU8::new_reuse(5, 5, buf);
        assert_eq!(img2.buf.len(), 25);
        assert!(img2.buf.capacity() >= 1024);
    }

    #[test]
    fn interpolate_at_pixel_center() {
        let mut img = ImageU8::new(4, 4);
        img.set(1, 1, 100);
        // At exact pixel center (1.5, 1.5), px-0.5 = (1.0, 1.0)
        // floor gives (1,1), fx=0, fy=0 → exactly pixel (1,1)
        let val = img.interpolate(1.5, 1.5);
        assert!((val - 100.0).abs() < 1e-10);
    }

    #[test]
    fn interpolate_between_pixels() {
        let mut img = ImageU8::new(4, 4);
        img.set(0, 0, 0);
        img.set(1, 0, 100);
        // At (1.0, 0.5): px-0.5 = (0.5, 0.0), x0=0, y0=0, fx=0.5, fy=0
        // = 0*(1-0.5) + 100*0.5 = 50
        let val = img.interpolate(1.0, 0.5);
        assert!((val - 50.0).abs() < 1e-10);
    }

    #[test]
    fn interpolate_unclamped_matches_interpolate_for_interior() {
        let mut img = ImageU8::new(10, 10);
        for y in 0..10 {
            for x in 0..10 {
                img.set(x, y, (x * 25 + y * 10) as u8);
            }
        }
        // Sample a grid of interior points
        let mut px = 2.0;
        while px <= 8.0 {
            let mut py = 2.0;
            while py <= 8.0 {
                let clamped = img.interpolate(px, py);
                let unclamped = img.interpolate_unclamped(px, py);
                // clamped and unclamped should agree for interior points
                assert!((clamped - unclamped).abs() < 1e-10);
                py += 0.37;
            }
            px += 0.37;
        }
    }

    #[test]
    fn interpolation_safe_interior_points() {
        let img = ImageU8::new(10, 10);
        assert!(img.interpolation_safe(5.0, 5.0));
        assert!(img.interpolation_safe(1.5, 1.5));
        assert!(img.interpolation_safe(8.5, 8.5));
    }

    #[test]
    fn interpolation_safe_boundary_points() {
        let img = ImageU8::new(10, 10);
        assert!(!img.interpolation_safe(1.0, 5.0));
        assert!(!img.interpolation_safe(5.0, 1.0));
        assert!(!img.interpolation_safe(9.0, 5.0));
        assert!(!img.interpolation_safe(5.0, 9.0));
    }

    #[test]
    fn interpolate_clamps_at_edges() {
        let mut img = ImageU8::new(2, 2);
        img.set(0, 0, 200);
        img.set(1, 0, 200);
        img.set(0, 1, 200);
        img.set(1, 1, 200);
        // Interpolate beyond bounds
        let val = img.interpolate(-1.0, -1.0);
        assert!((val - 200.0).abs() < 1e-10);
    }
}
