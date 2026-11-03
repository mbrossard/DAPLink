//! The firmware row a bench record names: the image's answer reconciled with the operator's value.

// One table: this compiles the file `xtask` owns.
#[allow(dead_code)]
#[path = "../../../xtask/src/registry.rs"]
mod registry;

/// `DAP_Info` id of the product firmware version, whose format is the vendor's; ours is the row.
pub const DAP_INFO_ID: u8 = 0x09;

/// What every probe row starts with, so another vendor's version string is not read as a row.
const ROW_PREFIX: &str = "firmware/";
/// Names an image the registry has no row for.
const EXTERNAL_PREFIX: &str = "external:";
const UNKNOWN: &str = "unknown";
const NAMING: &str =
    "`cargo xtask list` names the rows, and `external:<name>` names an image outside them";

/// What named a record's row.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Source {
    Image,
    Operator,
    None,
}

impl Source {
    /// The record's `firmware_row_source` value.
    pub fn key(self) -> &'static str {
        match self {
            Source::Image => "image",
            Source::Operator => "operator",
            Source::None => "none",
        }
    }
}

/// The row to record, what named it, and what the operator should hear about it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Resolved {
    pub row: Option<String>,
    pub source: Source,
    pub warnings: Vec<String>,
}

impl Resolved {
    /// The row and its source, for a run header.
    pub fn describe(&self) -> String {
        format!(
            "{} ({})",
            self.row.as_deref().unwrap_or("-"),
            self.source.key()
        )
    }
}

/// The image's answer wins over the operator's value; nothing here stops a run.
pub fn resolve(image: Option<&str>, operator: Option<&str>) -> Resolved {
    let operator = operator
        .filter(|row| !row.is_empty() && *row != UNKNOWN)
        .map(canonical);
    let image = image.filter(|row| row.starts_with(ROW_PREFIX));
    let mut warnings = Vec::new();
    let repeated = operator.as_deref().filter(|row| Some(*row) != image);
    for row in [image, repeated].into_iter().flatten() {
        if !is_known(row) {
            warnings.push(format!(
                "`{row}` is not a firmware row in this tree, though a retired row stays valid; \
                 {NAMING}"
            ));
        }
    }
    let (row, source) = match (image, operator.as_deref()) {
        (Some(image), Some(operator)) => {
            if image != operator {
                warnings.push(format!(
                    "the probe's image is firmware row `{image}`, but --firmware-row says \
                     `{operator}`; recording the image's answer"
                ));
            }
            (Some(image), Source::Image)
        }
        (Some(image), None) => (Some(image), Source::Image),
        (None, Some(operator)) => (Some(operator), Source::Operator),
        (None, None) => {
            warnings.push(format!(
                "nothing names this probe's firmware row, so the record's is null; pass \
                 --firmware-row: {NAMING}"
            ));
            (None, Source::None)
        }
    };
    Resolved {
        row: row.map(str::to_string),
        source,
        warnings,
    }
}

fn probe_rows() -> impl Iterator<Item = &'static str> {
    registry::FIRMWARE
        .iter()
        .filter(|fw| fw.profile.is_some())
        .map(|fw| fw.name)
}

fn is_known(row: &str) -> bool {
    row.strip_prefix(EXTERNAL_PREFIX)
        .is_some_and(|name| !name.is_empty())
        || probe_rows().any(|known| known == row)
}

/// An unambiguous basename as the registry spells it; anything else stays as the operator wrote it.
fn canonical(name: &str) -> String {
    let mut rows = probe_rows().filter(|row| row.rsplit('/').next() == Some(name));
    match (rows.next(), rows.next()) {
        (Some(row), None) => row.to_string(),
        _ => name.to_string(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn row_of(resolved: &Resolved) -> (Option<&str>, Source) {
        (resolved.row.as_deref(), resolved.source)
    }

    /// `DAP_Info` truncates to a 64-byte packet, and a row outside the prefix would be ignored.
    #[test]
    fn every_probe_row_survives_the_wire_and_is_read_as_a_row() {
        for row in probe_rows() {
            assert!(row.len() <= 61, "{row}");
            let image = resolve(Some(row), None);
            assert_eq!(row_of(&image), (Some(row), Source::Image));
            assert!(image.warnings.is_empty(), "{:?}", image.warnings);
            let operator = resolve(None, Some(row));
            assert_eq!(row_of(&operator), (Some(row), Source::Operator));
            assert!(operator.warnings.is_empty(), "{:?}", operator.warnings);
        }
    }

    #[test]
    fn no_row_records_null_and_says_how_to_fix_it() {
        for (image, operator) in [
            (None, None),
            (Some(""), None),
            (Some(UNKNOWN), None),
            (Some(UNKNOWN), Some(UNKNOWN)),
            (None, Some("")),
        ] {
            let resolved = resolve(image, operator);
            assert_eq!(row_of(&resolved), (None, Source::None));
            let [warning] = &resolved.warnings[..] else {
                panic!("{:?}", resolved.warnings);
            };
            assert!(warning.contains("--firmware-row"), "{warning}");
            assert!(warning.contains("cargo xtask list"), "{warning}");
        }
    }

    #[test]
    fn a_resident_image_takes_the_operators_value() {
        for image in [None, Some(""), Some(UNKNOWN)] {
            let resolved = resolve(image, Some("firmware/mculink-flash"));
            assert_eq!(
                row_of(&resolved),
                (Some("firmware/mculink-flash"), Source::Operator)
            );
            assert!(resolved.warnings.is_empty());
        }
    }

    #[test]
    fn on_disagreement_the_image_wins_and_the_warning_names_both() {
        let resolved = resolve(Some("firmware/mculink"), Some("firmware/mculink-flash"));
        assert_eq!(row_of(&resolved), (Some("firmware/mculink"), Source::Image));
        let [warning] = &resolved.warnings[..] else {
            panic!("{:?}", resolved.warnings);
        };
        assert!(warning.contains("`firmware/mculink`"), "{warning}");
        assert!(warning.contains("`firmware/mculink-flash`"), "{warning}");
    }

    #[test]
    fn a_basename_is_recorded_under_the_full_row() {
        let agreed = resolve(Some("firmware/mculink-flash"), Some("mculink-flash"));
        assert_eq!(
            row_of(&agreed),
            (Some("firmware/mculink-flash"), Source::Image)
        );
        assert!(agreed.warnings.is_empty());
        assert_eq!(
            row_of(&resolve(None, Some("mculink-flash"))),
            (Some("firmware/mculink-flash"), Source::Operator)
        );
    }

    /// A row retired since the image was built, or a typo: recorded as given, with a warning.
    #[test]
    fn a_value_outside_the_registry_is_recorded_and_warned_about() {
        for (image, operator, source) in [
            (None, Some("firmware/retired-row"), Source::Operator),
            (Some("firmware/retired-row"), None, Source::Image),
            (
                Some("firmware/retired-row"),
                Some("firmware/retired-row"),
                Source::Image,
            ),
        ] {
            let resolved = resolve(image, operator);
            assert_eq!(row_of(&resolved), (Some("firmware/retired-row"), source));
            assert_eq!(resolved.warnings.len(), 1, "{:?}", resolved.warnings);
        }
        for bad in ["mculink-fash", "tests/firmwares/test-rp2040", "external:"] {
            let resolved = resolve(None, Some(bad));
            assert_eq!(row_of(&resolved), (Some(bad), Source::Operator));
            assert_eq!(resolved.warnings.len(), 1, "{:?}", resolved.warnings);
        }
    }

    /// Another vendor's image answers this id with its own version string.
    #[test]
    fn an_external_image_is_named_by_the_operator_alone() {
        assert_eq!(row_of(&resolve(Some("3.1.0"), None)), (None, Source::None));
        let named = resolve(Some("3.1.0"), Some("external:vendor-3.1.0"));
        assert_eq!(
            row_of(&named),
            (Some("external:vendor-3.1.0"), Source::Operator)
        );
        assert!(named.warnings.is_empty());
    }

    #[test]
    fn the_source_keys_are_the_record_values() {
        assert_eq!(Source::Image.key(), "image");
        assert_eq!(Source::Operator.key(), "operator");
        assert_eq!(Source::None.key(), "none");
    }

    #[test]
    fn a_header_says_the_row_and_what_named_it() {
        assert_eq!(
            resolve(Some("firmware/mculink"), None).describe(),
            "firmware/mculink (image)"
        );
        assert_eq!(resolve(None, None).describe(), "- (none)");
    }
}
