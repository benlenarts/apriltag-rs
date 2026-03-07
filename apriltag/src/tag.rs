use crate::family::TagFamily;
use crate::render::{self, RenderedTag};

/// A reference to a single tag within a family.
///
/// Provides per-tag operations (rendering, code access) without mixing
/// responsibilities into [`TagFamily`].
pub struct Tag<'a> {
    family: &'a TagFamily,
    index: usize,
}

impl<'a> Tag<'a> {
    /// Create a new `Tag` referencing the given family and index.
    pub(crate) fn new(family: &'a TagFamily, index: usize) -> Self {
        Self { family, index }
    }

    /// The family this tag belongs to.
    pub fn family(&self) -> &'a TagFamily {
        self.family
    }

    /// The index of this tag within its family's code list.
    pub fn index(&self) -> usize {
        self.index
    }

    /// The bit-pattern code for this tag.
    pub fn code(&self) -> u64 {
        self.family.codes[self.index]
    }

    /// Render this tag as a pixel grid.
    pub fn render(&self) -> RenderedTag {
        render::render(&self.family.layout, self.code())
    }
}

#[cfg(test)]
mod tests {
    use crate::family::tag16h5;

    #[test]
    fn tag_code_returns_correct_value() {
        let family = tag16h5();
        let tag = family.tag(0);
        assert_eq!(tag.code(), family.codes[0]);
    }

    #[test]
    fn tag_index_round_trips() {
        let family = tag16h5();
        let tag = family.tag(3);
        assert_eq!(tag.index(), 3);
    }

    #[test]
    fn tag_family_reference() {
        let family = tag16h5();
        let tag = family.tag(0);
        assert_eq!(tag.family().config.name, "tag16h5");
    }

    #[test]
    fn tag_render_matches_direct_render() {
        let family = tag16h5();
        let rendered_via_tag = family.tag(0).render();
        let rendered_direct = crate::render::render(&family.layout, family.codes[0]);
        assert_eq!(rendered_via_tag.pixels, rendered_direct.pixels);
        assert_eq!(rendered_via_tag.grid_size, rendered_direct.grid_size);
    }
}
