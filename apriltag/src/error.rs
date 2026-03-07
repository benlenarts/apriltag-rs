use thiserror::Error;

/// Errors produced when parsing or validating a tag layout.
///
/// ```
/// use apriltag::layout::Layout;
/// use apriltag::error::LayoutError;
///
/// // Non-square string length
/// let err = Layout::from_data_string("ddd").unwrap_err();
/// assert!(matches!(err, LayoutError::NotSquare(3)));
///
/// // Invalid character
/// let err = Layout::from_data_string("dddZddddd").unwrap_err();
/// assert!(matches!(err, LayoutError::InvalidChar('Z', 3)));
/// ```
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
