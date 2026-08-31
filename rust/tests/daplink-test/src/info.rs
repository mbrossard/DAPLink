//! The info case.

use crate::{Ctx, Failure};

/// Check the probe's `DAP_Info` surface over raw CMSIS-DAP v2 bulk.
pub(crate) fn run(cx: &mut Ctx, full_run: bool) -> Result<(), Failure> {
    inner(cx, full_run).map_err(Failure::Fatal)
}

fn inner(cx: &mut Ctx, _full_run: bool) -> Result<(), String> {
    let mut dap = crate::dap::open(&cx.selector)?;

    let version = dap.dap_info(0x04)?;
    let version = core::str::from_utf8(&version).map_err(|_| "version is not UTF-8")?;
    let version = version.trim_end_matches('\0');
    let triple_ok = version.split('.').take(3).filter(|p| !p.is_empty()).count() == 3
        && version
            .split('.')
            .take(3)
            .all(|p| p.bytes().all(|b| b.is_ascii_digit()));
    if !triple_ok {
        return Err(format!(
            "version `{version}` does not open with a x.y.z triple"
        ));
    }

    let caps = dap.dap_info(0xF0)?;
    if caps.first().is_none_or(|b| b & 1 == 0) {
        return Err(format!("capabilities {caps:02x?} do not advertise SWD"));
    }

    let size = dap.dap_info(0xFF)?;
    // Both 64 (full speed) and 512 (high speed) are valid; range check only.
    let size = match size[..] {
        [lo, hi] => u16::from_le_bytes([lo, hi]),
        _ => return Err(format!("packet size {size:02x?} is not a u16")),
    };
    if !packet_size_ok(size) {
        return Err(format!("packet size {size} out of range 64..=512"));
    }

    let count = dap.dap_info(0xFE)?;
    if count.first().is_none_or(|&c| c < 1) {
        return Err(format!("packet count {count:02x?} is below 1"));
    }
    Ok(())
}

fn packet_size_ok(size: u16) -> bool {
    (64..=512).contains(&size)
}

#[cfg(test)]
mod tests {
    use super::packet_size_ok;

    #[test]
    fn packet_size_range() {
        assert!(packet_size_ok(64));
        assert!(packet_size_ok(512));
        assert!(!packet_size_ok(63));
        assert!(!packet_size_ok(513));
        assert!(!packet_size_ok(0));
    }
}
