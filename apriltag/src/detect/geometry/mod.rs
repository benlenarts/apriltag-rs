pub(crate) mod linear_solve;
#[allow(clippy::needless_range_loop)]
mod mat3;
mod vec2;
mod vec3;

pub(crate) use linear_solve::forward_eliminate;
pub use mat3::{det, inv, Mat3};
pub use vec2::Vec2;
pub use vec3::Vec3;
