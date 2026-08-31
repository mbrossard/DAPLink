//! The target-selection seam: choose a [`Target`] from a registry table at runtime.

use crate::Target;

/// A hint derived from the dropped image: the resolved target must match it, and a mismatch is a
/// hard rejection rather than a guess.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ImageHint {
    /// The UF2 family id declared by the dropped image, or `None` for `.bin`/`.hex` (or a UF2
    /// with no family id).
    pub uf2_family: Option<u32>,
}

impl ImageHint {
    /// An empty hint (no image family known): no cross-check.
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
    /// A target was resolved, but it disagrees with the dropped image -- the image is for a
    /// different chip than the one composed.
    ImageMismatch,
}

/// Strategy for picking a target from a registry table.
pub trait TargetSelector {
    /// Select a target from `table`, consulting `hint` for any image-format cross-check. Returns
    /// the resolved target, or a [`SelectError`] describing why none was chosen.
    fn select<'t>(
        &mut self,
        hint: ImageHint,
        table: &'t [&'t dyn Target],
    ) -> Result<&'t dyn Target, SelectError>;
}

/// A selector that returns the first table entry, still cross-checking any image hint against it.
#[derive(Clone, Copy, Debug, Default)]
pub struct FixedTarget;

impl TargetSelector for FixedTarget {
    fn select<'t>(
        &mut self,
        hint: ImageHint,
        table: &'t [&'t dyn Target],
    ) -> Result<&'t dyn Target, SelectError> {
        let target = *table.first().ok_or(SelectError::NoTarget)?;
        if let Some(family) = hint.uf2_family {
            // A single-entry table still cross-checks the image family against the sole target.
            if target.uf2_family() != Some(family) {
                return Err(SelectError::ImageMismatch);
            }
        }
        Ok(target)
    }
}
