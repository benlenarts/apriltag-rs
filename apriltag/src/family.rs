use serde::Deserialize;

use crate::bits::{self, BitLocation};
use crate::error::LayoutError;
use crate::layout::Layout;

/// Serde-driven family configuration matching the TOML format.
#[derive(Debug, Clone, Deserialize)]
pub struct FamilyConfig {
    pub name: String,
    pub min_hamming: u32,
    /// Per-family complexity parameter used in the LCG seed computation.
    /// Required for Era 2 code generation; optional for classic families.
    #[serde(default)]
    pub min_complexity: Option<u32>,
    pub layout: LayoutConfig,
}

/// Layout configuration variant.
#[derive(Debug, Clone, Deserialize)]
#[serde(tag = "type", rename_all = "lowercase")]
pub enum LayoutConfig {
    Classic { grid_size: usize },
    Standard { grid_size: usize },
    Circle { grid_size: usize },
    Custom { grid_size: usize, data: String },
}

/// A fully loaded tag family with config, layout, codes, and computed fields.
#[derive(Debug, Clone)]
pub struct TagFamily {
    pub config: FamilyConfig,
    pub layout: Layout,
    pub codes: Vec<u64>,
    pub bit_locations: Vec<BitLocation>,
}

impl TagFamily {
    /// Construct a family from a parsed config and a slice of codes.
    pub fn from_config_and_codes(
        config: FamilyConfig,
        codes: Vec<u64>,
    ) -> Result<TagFamily, LayoutError> {
        let layout = build_layout(&config.layout)?;
        let bit_locations = bits::bit_locations(&layout);
        Ok(TagFamily {
            config,
            layout,
            codes,
            bit_locations,
        })
    }

    /// Parse a TOML config string and binary code data into a TagFamily.
    pub fn from_toml_and_bin(toml_str: &str, bin_data: &[u8]) -> Result<TagFamily, FamilyError> {
        let config: FamilyConfig =
            toml::from_str(toml_str).map_err(|e| FamilyError::Config(e.to_string()))?;
        let codes = parse_bin_codes(bin_data)?;
        TagFamily::from_config_and_codes(config, codes).map_err(FamilyError::Layout)
    }
}

/// Build a Layout from a LayoutConfig.
fn build_layout(config: &LayoutConfig) -> Result<Layout, LayoutError> {
    match config {
        LayoutConfig::Classic { grid_size } => Layout::classic(*grid_size),
        LayoutConfig::Standard { grid_size } => Layout::standard(*grid_size),
        LayoutConfig::Circle { grid_size } => Layout::circle(*grid_size),
        LayoutConfig::Custom { data, .. } => Layout::from_data_string(data),
    }
}

/// Parse a binary code file (flat array of little-endian u64).
fn parse_bin_codes(data: &[u8]) -> Result<Vec<u64>, FamilyError> {
    if !data.len().is_multiple_of(8) {
        return Err(FamilyError::InvalidBin(format!(
            "binary data length {} is not a multiple of 8",
            data.len()
        )));
    }
    Ok(data
        .chunks_exact(8)
        .map(|chunk| u64::from_le_bytes(chunk.try_into().unwrap()))
        .collect())
}

#[derive(Debug, thiserror::Error)]
pub enum FamilyError {
    #[error("config error: {0}")]
    Config(String),
    #[error("layout error: {0}")]
    Layout(#[from] LayoutError),
    #[error("invalid binary data: {0}")]
    InvalidBin(String),
}

// --- Built-in families ---

macro_rules! builtin_family {
    ($name:ident, $toml:expr, $bin:expr) => {
        pub fn $name() -> TagFamily {
            TagFamily::from_toml_and_bin(
                include_str!(concat!("../families/", $toml)),
                include_bytes!(concat!("../families/", $bin)),
            )
            .expect(concat!("built-in family ", $toml, " should be valid"))
        }
    };
}

builtin_family!(tag16h5, "tag16h5.toml", "tag16h5.bin");
builtin_family!(tag25h9, "tag25h9.toml", "tag25h9.bin");
builtin_family!(tag36h11, "tag36h11.toml", "tag36h11.bin");
builtin_family!(tag_circle21h7, "tagCircle21h7.toml", "tagCircle21h7.bin");
builtin_family!(tag_circle49h12, "tagCircle49h12.toml", "tagCircle49h12.bin");
builtin_family!(tag_custom48h12, "tagCustom48h12.toml", "tagCustom48h12.bin");
builtin_family!(tag_standard41h12, "tagStandard41h12.toml", "tagStandard41h12.bin");
builtin_family!(tag_standard52h13, "tagStandard52h13.toml", "tagStandard52h13.bin");

/// List of all built-in family names.
pub const BUILTIN_NAMES: &[&str] = &[
    "tag16h5",
    "tag25h9",
    "tag36h11",
    "tagCircle21h7",
    "tagCircle49h12",
    "tagCustom48h12",
    "tagStandard41h12",
    "tagStandard52h13",
];

/// Load a built-in family by name.
pub fn builtin_family(name: &str) -> Option<TagFamily> {
    match name {
        "tag16h5" => Some(tag16h5()),
        "tag25h9" => Some(tag25h9()),
        "tag36h11" => Some(tag36h11()),
        "tagCircle21h7" => Some(tag_circle21h7()),
        "tagCircle49h12" => Some(tag_circle49h12()),
        "tagCustom48h12" => Some(tag_custom48h12()),
        "tagStandard41h12" => Some(tag_standard41h12()),
        "tagStandard52h13" => Some(tag_standard52h13()),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn load_tag16h5() {
        let f = tag16h5();
        assert_eq!(f.config.name, "tag16h5");
        assert_eq!(f.config.min_hamming, 5);
        assert_eq!(f.layout.nbits, 16);
        assert_eq!(f.layout.grid_size, 8);
        assert_eq!(f.codes.len(), 30);
        assert_eq!(f.codes[0], 0x27c8);
        assert_eq!(f.codes[29], 0xb57a);
        assert!(!f.layout.reversed_border);
        assert_eq!(f.layout.border_width, 6);
    }

    #[test]
    fn load_tag25h9() {
        let f = tag25h9();
        assert_eq!(f.codes.len(), 35);
        assert_eq!(f.layout.nbits, 25);
        assert_eq!(f.codes[0], 0x156f1f4);
    }

    #[test]
    fn load_tag36h11() {
        let f = tag36h11();
        assert_eq!(f.codes.len(), 587);
        assert_eq!(f.layout.nbits, 36);
        assert_eq!(f.layout.grid_size, 10);
        assert_eq!(f.codes[0], 0xd7e00984b);
        assert!(!f.layout.reversed_border);
        assert_eq!(f.layout.border_width, 8);
    }

    #[test]
    fn load_tag_circle21h7() {
        let f = tag_circle21h7();
        assert_eq!(f.codes.len(), 38);
        assert_eq!(f.layout.nbits, 21);
        assert_eq!(f.layout.grid_size, 9);
        assert!(f.layout.reversed_border);
        assert_eq!(f.layout.border_width, 5);
        assert_eq!(f.codes[0], 0x157863);
    }

    #[test]
    fn load_tag_standard41h12() {
        let f = tag_standard41h12();
        assert_eq!(f.codes.len(), 2115);
        assert_eq!(f.layout.nbits, 41);
        assert!(f.layout.reversed_border);
        assert_eq!(f.layout.border_width, 5);
        assert_eq!(f.codes[0], 0x1bd8a64ad10);
    }

    #[test]
    fn load_tag_standard52h13() {
        let f = tag_standard52h13();
        assert_eq!(f.codes.len(), 48714);
        assert_eq!(f.layout.nbits, 52);
        assert_eq!(f.codes[0], 0x4064a19651ff1);
    }

    #[test]
    fn load_tag_circle49h12() {
        let f = tag_circle49h12();
        assert_eq!(f.codes.len(), 65535);
        assert_eq!(f.layout.nbits, 49);
        assert_eq!(f.codes[0], 0xc6c921d8614a);
    }

    #[test]
    fn load_tag_custom48h12() {
        let f = tag_custom48h12();
        assert_eq!(f.codes.len(), 42211);
        assert_eq!(f.layout.nbits, 48);
        assert_eq!(f.codes[0], 0xd6c8ae76dff0);
    }

    #[test]
    fn builtin_family_lookup() {
        assert!(builtin_family("tag36h11").is_some());
        assert!(builtin_family("tagCircle21h7").is_some());
        assert!(builtin_family("nonexistent").is_none());
    }

    #[test]
    fn tag16h5_bit_locations_match_c_reference() {
        let f = tag16h5();
        assert_eq!(f.bit_locations.len(), 16);
        assert_eq!(f.bit_locations[0].x, 1);
        assert_eq!(f.bit_locations[0].y, 1);
    }

    #[test]
    fn tag_circle21h7_bit_locations_match_c_reference() {
        let f = tag_circle21h7();
        assert_eq!(f.bit_locations.len(), 21);
        // First bit at (1, -2) — negative coordinate
        assert_eq!(f.bit_locations[0].x, 1);
        assert_eq!(f.bit_locations[0].y, -2);
    }
}
