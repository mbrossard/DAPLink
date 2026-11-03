//! Raw CMSIS-DAP v2 bulk transport, independent of probe-rs, shared by the cases.

use std::time::Duration;

use dap_conformance::cmsis_dap::{exchange, ids, Context, Finding, Link, Reply};
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
    /// Submit one request without reading its reply, so a caller can fill the probe's queue.
    pub(crate) fn send(&mut self, request: &[u8], timeout: Duration) -> Result<(), String> {
        self.out
            .transfer_blocking(Buffer::from(request.to_vec()), timeout)
            .into_result()
            .map(|_| ())
            .map_err(|e| format!("bulk write failed: {e}"))
    }

    /// Read one reply, for a caller that queued its requests separately.
    pub(crate) fn recv(&mut self, timeout: Duration) -> Result<Vec<u8>, String> {
        // Bulk IN reads must be a whole number of the endpoint's max packet size.
        let read_len = round_up(RESPONSE_BUFFER_HINT, self.inp.max_packet_size().max(1));
        Ok(self
            .inp
            .transfer_blocking(Buffer::new(read_len), timeout)
            .into_result()
            .map_err(|e| format!("bulk read failed: {e}"))?
            .into_vec())
    }

    pub(crate) fn command(&mut self, request: &[u8]) -> Result<Vec<u8>, String> {
        self.send(request, TIMEOUT)?;
        self.recv(TIMEOUT)
    }

    /// One `DAP_Info` exchange, returning the payload behind the `[0x00, len]` framing.
    pub(crate) fn dap_info(&mut self, id: u8) -> Result<Vec<u8>, String> {
        self.dap_info_checked(id).map(|(value, _)| value)
    }

    /// [`RawDap::dap_info`], with the findings `check::exchange` makes on the value.
    pub(crate) fn dap_info_checked(&mut self, id: u8) -> Result<(Vec<u8>, Vec<Finding>), String> {
        let request = [ids::INFO, id];
        let resp = self.command(&request)?;
        let checked = exchange(&request, &resp, Context::arm(Link::Swd));
        match checked.exchange.map(|e| e.reply) {
            Some(Reply::Info(value)) => Ok((value, checked.findings)),
            _ => Err(format!(
                "DAP_Info({id:#04x}) reply {resp:02x?} is malformed: {:?}",
                checked.findings
            )),
        }
    }
}

/// The probe's bus entry, matched by its selector.
pub(crate) fn find_device(selector: &DebugProbeSelector) -> Result<nusb::DeviceInfo, String> {
    nusb::list_devices()
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
        .ok_or_else(|| "probe not found on the bus".to_string())
}

/// Open the probe's USB device without claiming an interface.
pub(crate) fn open_device(selector: &DebugProbeSelector) -> Result<nusb::Device, String> {
    find_device(selector)?
        .open()
        .wait()
        .map_err(|e| e.to_string())
}

/// Open the probe's vendor bulk interface by its selector.
pub(crate) fn open(selector: &DebugProbeSelector) -> Result<RawDap, String> {
    let device = open_device(selector)?;
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

/// One SWD idle cycle: gated by a flash session, where a zero count would answer OK regardless.
const IDLE_CYCLE_SEQUENCE: [u8; 4] = [0x1D, 1, 1, 0];

/// Whether a flash session holds the wire; once none does, the pads are left parked.
pub(crate) fn wire_held(selector: &DebugProbeSelector) -> Result<bool, String> {
    let mut dap = open(selector)?;
    let reply = dap.command(&IDLE_CYCLE_SEQUENCE)?;
    let held = sequence_refused(&reply)?;
    if !held {
        // Best effort: the remount may detach the device, and the next attach reconnects anyway.
        let _ = dap.command(&[0x03]);
    }
    Ok(held)
}

/// dap-rs marks a refusal over the command byte, status left OK; `[0x1D, 0xFF]` is Arm's form.
fn sequence_refused(reply: &[u8]) -> Result<bool, String> {
    match reply {
        [0x1D, 0x00] => Ok(false),
        [0xFF, 0x00] | [0x1D, 0xFF] => Ok(true),
        _ => Err(format!("DAP_SWD_Sequence reply {reply:02x?} is malformed")),
    }
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

fn round_up(len: usize, granularity: usize) -> usize {
    if granularity == 0 {
        return len;
    }
    len.div_ceil(granularity) * granularity
}

#[cfg(test)]
mod tests {
    use super::{round_up, sequence_refused};

    /// During a session `DAP_SWJ_Clock` answers OK, so a poll on it passes; the sequence does not.
    #[test]
    fn a_held_session_passes_the_clock_poll_and_fails_the_sequence_poll() {
        let clock_poll_passes = |reply: &[u8]| matches!(reply, [0x11, 0x00]);
        assert!(clock_poll_passes(&[0x11, 0x00]));
        assert_eq!(sequence_refused(&[0xFF, 0x00]), Ok(true));
    }

    #[test]
    fn a_released_wire_answers_the_sequence_ok() {
        assert_eq!(sequence_refused(&[0x1D, 0x00]), Ok(false));
    }

    #[test]
    fn the_spec_form_of_a_refusal_also_reads_as_held() {
        assert_eq!(sequence_refused(&[0x1D, 0xFF]), Ok(true));
    }

    /// A reply that is neither answer never reads as a release.
    #[test]
    fn an_unrecognised_reply_is_an_error() {
        for reply in [&[0xFF][..], &[0x11, 0x00], &[0x1D], &[0x1D, 0x00, 0x00]] {
            assert!(sequence_refused(reply).is_err(), "{reply:02x?}");
        }
    }

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
