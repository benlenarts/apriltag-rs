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
        Self { width, height, stride, buf }
    }

    /// Create an image from existing pixel data.
    ///
    /// `stride` must be >= `width`, and `buf` must contain at least `stride * height` bytes.
    pub fn from_buf(width: u32, height: u32, stride: u32, buf: Vec<u8>) -> Self {
        assert!(stride >= width);
        assert!(buf.len() >= (stride * height) as usize);
        Self { width, height, stride, buf }
    }

    /// Get the pixel value at (x, y).
    #[inline]
    pub fn get(&self, x: u32, y: u32) -> u8 {
        self.buf[(y * self.stride + x) as usize]
    }

    /// Set the pixel value at (x, y).
    #[inline]
    pub fn set(&mut self, x: u32, y: u32, val: u8) {
        self.buf[(y * self.stride + x) as usize] = val;
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
