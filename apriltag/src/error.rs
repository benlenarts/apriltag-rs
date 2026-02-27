use thiserror::Error;

#[derive(Debug, Error)]
pub enum LayoutError {
    #[error("layout string length {0} is not a perfect square")]
    NotSquare(usize),

    #[error("invalid character '{0}' in layout string at position {1}")]
    InvalidChar(char, usize),

    #[error("layout is not rotationally symmetric")]
    NotSymmetric,

    #[error("no valid border detected in layout")]
    NoBorder,

    #[error("invalid border: {0}")]
    InvalidBorder(String),
}
