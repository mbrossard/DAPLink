//! The jtag-chain case: a raw IDCODE scan of every TAP, then `DAP_JTAG_IDCODE` at the ARM debug port.

use crate::{Ctx, Failure};

const CONNECT_JTAG: [u8; 2] = [0x02, 0x02];
/// `DAP_SWJ_Clock`, 1 MHz: slow enough that the chain is never the reason a TAP is silent.
const SWJ_CLOCK_HZ: u32 = 1_000_000;
const SWJ_SEQUENCE: u8 = 0x12;
/// The SWD-to-JTAG select sequence, `0xE73C` on the wire least-significant bit first.
const SWD_TO_JTAG: [u8; 2] = [0x3C, 0xE7];
/// TMS-high clocks that put any TAP state machine into Test-Logic-Reset.
const TMS_RESET_BITS: u8 = 51;
const JTAG_SEQUENCE: u8 = 0x14;
/// `DAP_JTAG_Sequence` info byte: TCK count in bits 5:0, TMS level in bit 6, TDO capture in 7.
const SEQ_TMS_HIGH: u8 = 0x40;
const SEQ_CAPTURE: u8 = 0x80;
/// Bits in one IDCODE, and the TCK count of one ground-truth capture sequence.
const IDCODE_BITS: u8 = 32;
/// TMS-high clocks that reach Test-Logic-Reset from any state, with one of margin.
const TLR_CLOCKS: u8 = 6;
const JTAG_CONFIGURE: u8 = 0x15;
const JTAG_IDCODE: u8 = 0x16;
const DAP_OK: u8 = 0x00;

/// The ARM JTAG-DP IDCODE with the version nibble masked off.
const ARM_JTAG_DP: u32 = 0x0BA0_0477;
const IDCODE_PART_MASK: u32 = 0x0FFF_FFFF;
/// The IR width of an ARM JTAG-DP, and the width that makes 0x0E mean IDCODE.
const ARM_IR_BITS: u8 = 4;

/// Configure the chain the target actually has and read an IDCODE from each of its TAPs.
pub(crate) fn run(cx: &mut Ctx, _full_run: bool) -> Result<(), Failure> {
    let ir_lens = cx.jtag_ir_lens;
    if ir_lens.len() < 2 {
        return Err(Failure::Skip(
            "jtag-chain: no multi-TAP chain declared for this target".into(),
        ));
    }

    // The trace pin the swo case routes is JTDO; leaving it routed breaks the chain.
    let mut session = cx
        .session
        .take()
        .ok_or_else(|| Failure::Fatal("jtag-chain needs a successful connect".into()))?;
    let unrouted =
        crate::jtag::unroute_trace_pin(&mut session, cx.dbgmcu_cr).map_err(Failure::Failed);
    drop(session);

    let result = unrouted.and_then(|()| attempt(cx, ir_lens));

    // Later cases need the SWD substrate back whatever this case decided.
    match crate::jtag::reopen_swd(cx) {
        Ok(session) => cx.session = Some(session),
        Err(e) => return Err(Failure::Fatal(format!("re-open SWD: {e}"))),
    }
    result
}

/// Drive the chain over raw bulk: connect, reset the TAPs, declare the IR lengths, read IDCODEs.
fn attempt(cx: &Ctx, ir_lens: &[u8]) -> Result<(), Failure> {
    let mut dap = crate::dap::open(&cx.selector).map_err(Failure::Fatal)?;

    let connect = dap.command(&CONNECT_JTAG).map_err(Failure::Fatal)?;
    if connect != [0x02, 0x02] {
        return Err(Failure::Failed(format!(
            "DAP_Connect(JTAG) answered {connect:02x?}: the probe did not select JTAG"
        )));
    }
    let mut clock = vec![0x11];
    clock.extend_from_slice(&SWJ_CLOCK_HZ.to_le_bytes());
    dap.command(&clock).map_err(Failure::Fatal)?;

    // Line reset, the SWD-to-JTAG select word, then another reset, leaving every TAP in Test-Logic-Reset.
    for seq in [
        tms_high(TMS_RESET_BITS),
        swj_bits(16, &SWD_TO_JTAG),
        tms_high(TMS_RESET_BITS),
    ] {
        dap.command(&seq).map_err(Failure::Fatal)?;
    }

    let mut configure = vec![JTAG_CONFIGURE, ir_lens.len() as u8];
    configure.extend_from_slice(ir_lens);
    let configured = dap.command(&configure).map_err(Failure::Fatal)?;
    if configured != [JTAG_CONFIGURE, DAP_OK] {
        return Err(Failure::Failed(format!(
            "DAP_JTAG_Configure with IR lengths {ir_lens:?} answered {configured:02x?}"
        )));
    }

    // Ground truth; ends in Run-Test/Idle, where `DAP_JTAG_IDCODE` expects to start.
    let taps = ir_lens.len();
    let scanned = dap.command(&scan_chain(taps)).map_err(Failure::Fatal)?;
    let truth = parse_scan(&scanned, taps).map_err(Failure::Failed)?;
    if let Some(bad) = truth.iter().position(|id| !is_plausible(*id)) {
        return Err(Failure::Failed(format!(
            "the raw DR scan reads {:?} and position {bad} of {taps} is not an IDCODE: the chain \
             itself does not answer with {taps} TAPs, so the IR lengths {ir_lens:?} in the target \
             registry describe a chain this target does not have",
            hex(&truth)
        )));
    }

    for (index, idcode) in truth.iter().enumerate() {
        if truth[..index].contains(idcode) {
            return Err(Failure::Failed(format!(
                "TAP {index} reads IDCODE {idcode:#010x}, which an earlier TAP already \
                 answered: the chain is being read as one TAP, not {taps}"
            )));
        }
    }

    // Exactly one debug port, identified by part rather than position.
    let arm = arm_index(&truth).map_err(Failure::Failed)?;
    if ir_lens[arm] != ARM_IR_BITS {
        return Err(Failure::Failed(format!(
            "the raw DR scan puts the ARM JTAG-DP at index {arm}, where the registry declares an \
             IR length of {} rather than {ARM_IR_BITS}: the declared chain and the chain the \
             target presents do not line up",
            ir_lens[arm]
        )));
    }

    // Only the ARM TAP is asked: 0x0E is IDCODE on its four-bit IR alone.
    let reply = dap
        .command(&[JTAG_IDCODE, arm as u8])
        .map_err(Failure::Fatal)?;
    let seen = parse_idcode(&reply).map_err(Failure::Failed)?;
    if seen != truth[arm] {
        let elsewhere = truth.iter().position(|id| *id == seen);
        let diagnosis = match elsewhere {
            Some(other) => format!(
                "that is the IDCODE the raw scan reads at index {other}, so the two disagree \
                 about which end of the chain index 0 names -- index 0 is the TAP nearest TDO, \
                 the one the raw DR scan presents first"
            ),
            None => "that is an IDCODE the chain never presented, so the probe's per-index \
                 addressing -- its bypass accounting for a TAP that is not first -- is what is \
                 out of step, not the chain"
                .to_string(),
        };
        return Err(Failure::Failed(format!(
            "DAP_JTAG_IDCODE at index {arm} reads {seen:#010x} while the raw DR scan of the same \
             chain reads {:#010x} there: {diagnosis}. Raw scan {:?}, IR lengths {ir_lens:?}, TDO \
             first",
            truth[arm],
            hex(&truth)
        )));
    }
    Ok(())
}

/// The index of the chain's one ARM JTAG-DP, the only TAP `DAP_JTAG_IDCODE` can address.
fn arm_index(truth: &[u32]) -> Result<usize, String> {
    let mut found = truth
        .iter()
        .enumerate()
        .filter(|(_, id)| *id & IDCODE_PART_MASK == ARM_JTAG_DP);
    match (found.next(), found.next()) {
        (Some((index, _)), None) => Ok(index),
        _ => {
            let ids = hex(truth);
            Err(format!(
                "the chain reads {ids:?} and none or several of them are the ARM JTAG-DP part \
                 {ARM_JTAG_DP:#010x}; a {}-TAP chain must carry exactly one debug port",
                truth.len()
            ))
        }
    }
}

/// A `DAP_JTAG_Sequence` that resets the TAPs, shifts out every IDCODE, and ends in Run-Test/Idle.
fn scan_chain(taps: usize) -> Vec<u8> {
    let mut v = vec![JTAG_SEQUENCE, (taps + 5) as u8];
    // Test-Logic-Reset, then one TMS-low clock into Run-Test/Idle.
    v.extend_from_slice(&[SEQ_TMS_HIGH | TLR_CLOCKS, 0x00, 1, 0x00]);
    // Select-DR-Scan, then Capture-DR and Shift-DR.
    v.extend_from_slice(&[SEQ_TMS_HIGH | 1, 0x00, 2, 0x00]);
    // One capture sequence per TAP, TDI held high so every stage behind sees ones.
    for _ in 0..taps {
        v.push(SEQ_CAPTURE | IDCODE_BITS);
        v.extend_from_slice(&[0xFF; 4]);
    }
    // Exit1-DR and Update-DR, then back to Run-Test/Idle for DAP_JTAG_IDCODE.
    v.extend_from_slice(&[SEQ_TMS_HIGH | 2, 0x00, 1, 0x00]);
    v
}

/// The IDCODEs in a [`scan_chain`] reply, four little-endian bytes per TAP.
fn parse_scan(reply: &[u8], taps: usize) -> Result<Vec<u32>, String> {
    match reply.get(..2) {
        Some([JTAG_SEQUENCE, DAP_OK]) => {}
        Some([JTAG_SEQUENCE, status]) => {
            return Err(format!("DAP_JTAG_Sequence status {status:#04x}"))
        }
        _ => return Err(format!("DAP_JTAG_Sequence reply {reply:02x?} is malformed")),
    }
    let want = 2 + taps * 4;
    let body = reply.get(2..want).ok_or_else(|| {
        format!(
            "the raw DR scan returned {} bytes, want {want}: the probe captured fewer TAPs than \
             the chain declares",
            reply.len()
        )
    })?;
    Ok(body
        .chunks_exact(4)
        .map(|w| u32::from_le_bytes([w[0], w[1], w[2], w[3]]))
        .collect())
}

/// Bit 0 is read-as-one, and the two rail values are a silent TAP.
fn is_plausible(idcode: u32) -> bool {
    idcode & 1 == 1 && idcode != u32::MAX
}

/// IDCODEs as hex, for a failure that has to show two lists side by side.
fn hex(ids: &[u32]) -> Vec<String> {
    ids.iter().map(|id| format!("{id:#010x}")).collect()
}

/// A `DAP_SWJ_Sequence` command of `bits` clocks with TMS held high.
fn tms_high(bits: u8) -> Vec<u8> {
    let bytes = vec![0xFF; usize::from(bits).div_ceil(8)];
    swj_bits(bits, &bytes)
}

/// A `DAP_SWJ_Sequence` command carrying `bits` clocks of `data`.
fn swj_bits(bits: u8, data: &[u8]) -> Vec<u8> {
    let mut v = vec![SWJ_SEQUENCE, bits];
    v.extend_from_slice(data);
    v
}

/// The IDCODE in a `DAP_JTAG_IDCODE` reply: `[0x16, status, id32 LE]`.
fn parse_idcode(reply: &[u8]) -> Result<u32, String> {
    match reply {
        [JTAG_IDCODE, DAP_OK, a, b, c, d] => {
            let idcode = u32::from_le_bytes([*a, *b, *c, *d]);
            if !is_plausible(idcode) {
                return Err(format!("IDCODE reads {idcode:#010x}: no TAP answered"));
            }
            Ok(idcode)
        }
        [JTAG_IDCODE, status, ..] => Err(format!("DAP_JTAG_IDCODE status {status:#04x}")),
        other => Err(format!("DAP_JTAG_IDCODE reply {other:02x?} is malformed")),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The wire form: status byte, then the IDCODE little-endian.
    #[test]
    fn parse_idcode_reads_the_little_endian_word() {
        assert_eq!(
            parse_idcode(&[0x16, 0x00, 0x77, 0x04, 0xA0, 0x4B]),
            Ok(0x4BA0_0477)
        );
    }

    /// The two rail values are the shapes a missing TAP produces, and neither is an identity.
    #[test]
    fn parse_idcode_rejects_an_absent_tap() {
        assert!(parse_idcode(&[0x16, 0x00, 0xFF, 0xFF, 0xFF, 0xFF]).is_err());
        assert!(parse_idcode(&[0x16, 0x00, 0x00, 0x00, 0x00, 0x00]).is_err());
    }

    #[test]
    fn parse_idcode_rejects_a_failed_or_truncated_reply() {
        assert!(parse_idcode(&[0x16, 0xFF, 0x77, 0x04, 0xA0, 0x4B]).is_err());
        assert!(parse_idcode(&[0x16, 0x00, 0x77]).is_err());
        assert!(parse_idcode(&[]).is_err());
    }

    /// The scan resets, walks to Shift-DR, captures one IDCODE per TAP, and ends in Run-Test/Idle.
    #[test]
    fn the_scan_walks_to_shift_dr_and_back_to_idle() {
        let seq = scan_chain(2);
        assert_eq!(seq[0], JTAG_SEQUENCE);
        assert_eq!(seq[1], 7, "five state moves plus one capture per TAP");
        assert_eq!(seq[2] & SEQ_TMS_HIGH, SEQ_TMS_HIGH);
        assert!(
            seq[2] & 0x3F >= 5,
            "fewer than five clocks may not reach TLR"
        );
        assert_eq!(seq[4], 1, "one TMS-low clock leaves TLR for Run-Test/Idle");
        assert_eq!(seq[6], SEQ_TMS_HIGH | 1, "Select-DR-Scan");
        assert_eq!(seq[8], 2, "Capture-DR then Shift-DR");
        assert_eq!(seq[10], SEQ_CAPTURE | IDCODE_BITS);
        assert_eq!(seq[11..15], [0xFF; 4]);
        assert_eq!(seq[15], SEQ_CAPTURE | IDCODE_BITS);
        assert_eq!(seq[10] & SEQ_TMS_HIGH, 0, "the chain stays in Shift-DR");
        assert_eq!(seq[20], SEQ_TMS_HIGH | 2, "Exit1-DR then Update-DR");
        assert_eq!(seq[22], 1, "back to Run-Test/Idle");
    }

    /// Each TAP's four bytes are its IDCODE little-endian, in TDO order.
    #[test]
    fn the_scan_reply_reads_one_idcode_per_tap() {
        let reply = [0x14, 0x00, 0x77, 0x04, 0xA0, 0x4B, 0x41, 0x00, 0x41, 0x06];
        assert_eq!(parse_scan(&reply, 2), Ok(vec![0x4BA0_0477, 0x0641_0041]));
    }

    /// A short reply must not read as a chain that answered with zeros.
    #[test]
    fn the_scan_reply_refuses_to_invent_taps() {
        assert!(parse_scan(&[0x14, 0x00, 0x77, 0x04, 0xA0, 0x4B], 2).is_err());
        assert!(parse_scan(&[0x14, 0xFF], 1).is_err());
        assert!(parse_scan(&[], 1).is_err());
    }

    /// The debug port is identified by its part, not by where it sits.
    #[test]
    fn the_arm_debug_part_matches_regardless_of_the_version_nibble() {
        assert_eq!(0x4BA0_0477u32 & IDCODE_PART_MASK, ARM_JTAG_DP);
        assert_eq!(0x3BA0_0477u32 & IDCODE_PART_MASK, ARM_JTAG_DP);
        assert_ne!(0x0641_0041u32 & IDCODE_PART_MASK, ARM_JTAG_DP);
    }

    /// The ARM TAP is found at either end; none or two is not a checkable chain.
    #[test]
    fn the_arm_tap_is_located_by_part_at_either_end() {
        assert_eq!(arm_index(&[0x4BA0_0477, 0x0643_1041]), Ok(0));
        assert_eq!(arm_index(&[0x0643_1041, 0x4BA0_0477]), Ok(1));
        assert!(arm_index(&[0x0643_1041, 0x0641_0041]).is_err());
        assert!(arm_index(&[0x4BA0_0477, 0x3BA0_0477]).is_err());
    }

    /// A TMS-high reset is whole bytes of ones, and its bit count is what the probe clocks.
    #[test]
    fn tms_high_pads_to_whole_bytes() {
        assert_eq!(
            tms_high(51),
            vec![0x12, 51, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]
        );
        assert_eq!(swj_bits(16, &SWD_TO_JTAG), vec![0x12, 16, 0x3C, 0xE7]);
    }
}
