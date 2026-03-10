#![forbid(unsafe_code)]
#![deny(clippy::unwrap_used, clippy::expect_used)]

pub mod bits;
pub mod detect;
pub mod error;
pub mod family;
pub mod hamming;
pub mod layout;
pub mod render;
pub mod tag;
pub mod types;

// Re-export commonly used types at the crate root for ergonomic imports.
pub use detect::detector::{Detection, Detector, DetectorBuffers, DetectorBuilder, DetectorConfig};
pub use detect::image::{GrayImage, ImageRef, ImageU8};
