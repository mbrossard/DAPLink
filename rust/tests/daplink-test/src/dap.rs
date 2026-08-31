//! Raw CMSIS-DAP v2 bulk transport, independent of probe-rs, shared by the cases.

use std::time::Duration;

use nusb::transfer::{Buffer, Bulk, Direction, In, Out};
use nusb::{Endpoint, MaybeFuture};
use probe_rs::probe::DebugProbeSelector;

const TIMEOUT: Duration = Duration::from_secs(2);

/// Vendor-specific interface class, as CMSIS-DAP v2 advertises.
const CLASS_VENDOR: u8 = 0xFF;

/// `DAP_SWJ_Pins` bit for nRESET.
const PIN_NRESET: u8 = 1 << 7;

/// Response-size hint; reads round it up to a whole number of max-size packets.
const RESPONSE_BUFFER_HINT: usize = 64;

/// The probe's claimed CMSIS-DAP v2 bulk interface and its endpoint pair.
pub(crate) struct RawDap {
    _interface: nusb::Interface,
    out: Endpoint<Bulk, Out>,
    inp: Endpoint<Bulk, In>,
}

impl RawDap {
    pub(crate) fn command(&mut self, request: &[u8]) -> Result<Vec<u8>, String> {
        self.out
            .transfer_blocking(Buffer::from(request.to_vec()), TIMEOUT)
            .into_result()
            .map_err(|e| format!("bulk write failed: {e}"))?;
        // Bulk IN reads must be a whole number of the endpoint's max packet size.
        let read_len = round_up(RESPONSE_BUFFER_HINT, self.inp.max_packet_size().max(1));
        Ok(self
            .inp
            .transfer_blocking(Buffer::new(read_len), TIMEOUT)
            .into_result()
            .map_err(|e| format!("bulk read failed: {e}"))?
            .into_vec())
    }

    /// One `DAP_Info` exchange, returning the payload behind the `[0x00, len]` framing.
    pub(crate) fn dap_info(&mut self, id: u8) -> Result<Vec<u8>, String> {
        let resp = self.command(&[0x00, id])?;
        if resp.first() != Some(&0x00) {
            return Err(format!(
                "DAP_Info({id:#04x}) reply {resp:02x?} echoes wrongly"
            ));
        }
        let len = usize::from(*resp.get(1).ok_or("reply too short")?);
        resp.get(2..2 + len)
            .map(<[u8]>::to_vec)
            .ok_or_else(|| format!("DAP_Info({id:#04x}) reply shorter than its length byte"))
    }
}

/// Open the probe's vendor bulk interface by its selector.
pub(crate) fn open(selector: &DebugProbeSelector) -> Result<RawDap, String> {
    let device_info = nusb::list_devices()
        .wait()
        .map_err(|e| e.to_string())?
        .find(|d| {
            d.vendor_id() == selector.vendor_id
                && d.product_id() == selector.product_id
                && selector
                    .serial_number
                    .as_deref()
                    .is_none_or(|s| d.serial_number() == Some(s))
        })
        .ok_or("probe not found on the bus")?;
    let device = device_info.open().wait().map_err(|e| e.to_string())?;
    let config = device.active_configuration().map_err(|e| e.to_string())?;

    // The vendor interface with a bulk OUT/IN pair is the CMSIS-DAP v2 interface.
    let mut found = None;
    for group in config.interfaces() {
        let alt = group.first_alt_setting();
        if alt.class() != CLASS_VENDOR {
            continue;
        }
        let (mut out, mut inp) = (None, None);
        for ep in alt.endpoints() {
            match ep.direction() {
                Direction::Out => out = out.or(Some(ep.address())),
                Direction::In => inp = inp.or(Some(ep.address())),
            }
        }
        if let (Some(o), Some(i)) = (out, inp) {
            found = Some((alt.interface_number(), o, i));
            break;
        }
    }
    let (ifnum, out_addr, in_addr) = found.ok_or("no vendor bulk interface")?;
    let interface = device
        .detach_and_claim_interface(ifnum)
        .wait()
        .map_err(|e| e.to_string())?;
    let out = interface
        .endpoint::<Bulk, Out>(out_addr)
        .map_err(|e| e.to_string())?;
    let inp = interface
        .endpoint::<Bulk, In>(in_addr)
        .map_err(|e| e.to_string())?;
    Ok(RawDap {
        _interface: interface,
        out,
        inp,
    })
}

/// Pulse the target's reset line through the probe: assert nRESET low, dwell, release.
pub(crate) fn pulse_nreset(selector: &DebugProbeSelector) -> Result<(), String> {
    let mut dap = open(selector)?;
    dap.command(&[0x02, 0x01])?; // DAP_Connect, SWD
    let low = dap.command(&[0x10, 0x00, PIN_NRESET, 0, 0, 0, 0])?;
    if low.first() != Some(&0x10) {
        return Err(format!("DAP_SWJ_Pins reply {low:02x?} echoes wrongly"));
    }
    std::thread::sleep(Duration::from_millis(50));
    dap.command(&[0x10, PIN_NRESET, PIN_NRESET, 0, 0, 0, 0])?;
    dap.command(&[0x03])?; // DAP_Disconnect releases the wire
    Ok(())
}

/// Round `len` up to the next multiple of `granularity`.
fn round_up(len: usize, granularity: usize) -> usize {
    if granularity == 0 {
        return len;
    }
    len.div_ceil(granularity) * granularity
}

#[cfg(test)]
mod tests {
    use super::round_up;

    #[test]
    fn round_up_keeps_a_full_speed_read_at_one_packet() {
        assert_eq!(round_up(64, 64), 64);
    }

    #[test]
    fn round_up_pads_to_a_whole_high_speed_packet() {
        assert_eq!(round_up(64, 512), 512);
    }

    #[test]
    fn round_up_is_a_no_op_at_zero_granularity() {
        assert_eq!(round_up(64, 0), 64);
    }
}
