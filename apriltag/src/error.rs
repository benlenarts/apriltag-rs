use std::fmt;

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
#[derive(Debug)]
pub enum LayoutError {
    NotSquare(usize),
    InvalidChar(char, usize),
    NotSymmetric,
    NoBorder,
    InvalidBorder(String),
}

impl fmt::Display for LayoutError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NotSquare(len) => write!(f, "layout string length {len} is not a perfect square"),
            Self::InvalidChar(ch, pos) => {
                write!(
                    f,
                    "invalid character '{ch}' in layout string at position {pos}"
                )
            }
            Self::NotSymmetric => write!(f, "layout is not rotationally symmetric"),
            Self::NoBorder => write!(f, "no valid border detected in layout"),
            Self::InvalidBorder(msg) => write!(f, "invalid border: {msg}"),
        }
    }
}

impl std::error::Error for LayoutError {}
