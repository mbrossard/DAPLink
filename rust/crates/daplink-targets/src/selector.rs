//! The target-selection seam: choose a [`Target`] from a registry table at runtime.

use crate::Target;

/// An image hint that the selected target must match.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ImageHint {
    /// The declared UF2 family, or `None` when no family flag is present.
    pub uf2_family: Option<u32>,
}

impl ImageHint {
    /// An empty hint, accepted only when the selection policy permits it.
    pub const NONE: ImageHint = ImageHint { uf2_family: None };

    /// A hint carrying the dropped UF2 image's family id.
    pub const fn uf2(family: u32) -> ImageHint {
        ImageHint {
            uf2_family: Some(family),
        }
    }
}

/// Why a selector refused to resolve a target.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SelectError {
    /// No composed target matched (or the table is empty).
    NoTarget,
    /// The family is missing, unsupported, or matches more than one composed entry.
    ImageMismatch,
}

/// Strategy for picking a target from a registry table.
pub trait TargetSelector {
    /// Select a target from `table`, cross-checking the image hint.
    fn select<'t>(
        &mut self,
        hint: ImageHint,
        table: &'t [&'t dyn Target],
    ) -> Result<&'t dyn Target, SelectError>;
}

/// With no family hint, the sole composed target; with one, the unique target of that UF2 family.
#[derive(Clone, Copy, Debug, Default)]
pub struct FixedTarget;

impl TargetSelector for FixedTarget {
    fn select<'t>(
        &mut self,
        hint: ImageHint,
        table: &'t [&'t dyn Target],
    ) -> Result<&'t dyn Target, SelectError> {
        let first = *table.first().ok_or(SelectError::NoTarget)?;
        let Some(family) = hint.uf2_family else {
            return if table.len() == 1 {
                Ok(first)
            } else {
                Err(SelectError::ImageMismatch)
            };
        };
        let mut matches = table
            .iter()
            .copied()
            .filter(|t| t.accepts_uf2_family(family));
        let target = matches.next().ok_or(SelectError::ImageMismatch)?;
        if matches.next().is_some() {
            return Err(SelectError::ImageMismatch);
        }
        Ok(target)
    }
}
