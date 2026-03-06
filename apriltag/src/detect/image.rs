/// Grayscale image with row-major pixel data.
#[derive(Debug, Clone)]
pub struct ImageU8 {
    pub width: u32,
    pub height: u32,
    pub stride: u32,
    pub buf: Vec<u8>,
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

    /// Consume the image and return the backing buffer for reuse.
    pub fn into_buf(self) -> Vec<u8> {
        self.buf
    }

    /// Returns true if bilinear interpolation at `(px, py)` and one pixel in
    /// any direction stays within bounds — i.e. all of `(px±1, py±1)` are safe
    /// for [`interpolate_unclamped`](Self::interpolate_unclamped).
    #[inline]
    pub fn interpolation_safe(&self, px: f64, py: f64) -> bool {
        px >= 1.5 && py >= 1.5 && px <= self.width as f64 - 1.5 && py <= self.height as f64 - 1.5
    }

    /// Bilinear interpolation without coordinate clamping.
    ///
    /// The caller must ensure that `(px, py)` is far enough from the image
    /// boundary that all four sample pixels are in bounds. Use
    /// [`interpolation_safe`](Self::interpolation_safe) to check.
    #[inline]
    pub fn interpolate_unclamped(&self, px: f64, py: f64) -> f64 {
        let x = px - 0.5;
        let y = py - 0.5;
        let x0 = x.floor() as usize;
        let y0 = y.floor() as usize;
        debug_assert!(x0 < self.width as usize - 1);
        debug_assert!(y0 < self.height as usize - 1);
        let fx = x - x0 as f64;
        let fy = y - y0 as f64;
        let row0 = y0 * self.stride as usize;
        let row1 = row0 + self.stride as usize;
        let v00 = self.buf[row0 + x0] as f64;
        let v10 = self.buf[row0 + x0 + 1] as f64;
        let v01 = self.buf[row1 + x0] as f64;
        let v11 = self.buf[row1 + x0 + 1] as f64;
        v00 * (1.0 - fx) * (1.0 - fy)
            + v10 * fx * (1.0 - fy)
            + v01 * (1.0 - fx) * fy
            + v11 * fx * fy
    }

    /// Bilinear interpolation at sub-pixel coordinates.
    ///
    /// Uses the convention from the spec: floor(px - 0.5) for the base pixel.
    pub fn interpolate(&self, px: f64, py: f64) -> f64 {
        let x = px - 0.5;
        let y = py - 0.5;
        let x0 = x.floor() as i64;
        let y0 = y.floor() as i64;
        let x1 = x0 + 1;
        let y1 = y0 + 1;

        let fx = x - x0 as f64;
        let fy = y - y0 as f64;

        let w = self.width as i64;
        let h = self.height as i64;

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
}

#[cfg(test)]
mod tests {
    use super::*;

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
