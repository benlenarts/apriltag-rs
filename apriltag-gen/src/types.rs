/// The type of a cell in a tag layout grid.
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
