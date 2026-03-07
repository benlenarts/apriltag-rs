/// The type of a cell in a tag layout grid.
///
/// ```
/// use apriltag::layout::Layout;
/// use apriltag::types::CellType;
///
/// let layout = Layout::classic(8).unwrap();
/// assert_eq!(layout.cell(0, 0), CellType::White);  // outer border
/// assert_eq!(layout.cell(1, 1), CellType::Black);  // inner border
/// assert_eq!(layout.cell(3, 3), CellType::Data);   // data region
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CellType {
    /// A data bit cell — carries one bit of the tag code.
    Data,
    /// Always-black border cell.
    Black,
    /// Always-white border cell.
    White,
    /// Ignored / transparent — outside the tag boundary.
    Ignored,
}

/// A rendered pixel value.
///
/// ```
/// use apriltag::family;
/// use apriltag::types::Pixel;
///
/// let f = family::tag16h5();
/// let tag = f.render(0);
/// assert_eq!(tag.pixel(0, 0), Pixel::White);
/// assert_eq!(tag.pixel(1, 1), Pixel::Black);
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Pixel {
    Black,
    White,
    Transparent,
}
