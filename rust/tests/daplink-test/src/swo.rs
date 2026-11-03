//! The swo case: the DUT's ITM emits on port 0, the host reads it back through `DAP_SWO_*`.

use std::time::{Duration, Instant};

use daplink_test_protocol as proto;
use probe_rs::architecture::arm::swo::SwoConfig;

use crate::load::DapLoad;
use crate::util::chain;
use crate::{rtt, Ctx, Failure};

/// SWO/ITM bauds to sweep; each divides the DUT's trace clock cleanly.
const SWO_BAUDS: &[u32] = &[500_000, 1_000_000, 2_000_000, 4_000_000];
/// TPIU clock hint for [`SwoConfig`], informational only: the DUT sets its own trace clock.
const TPIU_CLK_HINT: u32 = 84_000_000;
/// How far the DUT's realized trace rate may sit from the requested one.
const RATE_TOLERANCE_PCT: u32 = 2;
/// ITM software-trace header for an 8-bit write to stimulus port 0: `(port << 3) | 0b01`.
const ITM_PORT0_HEADER: u8 = 0x01;
/// Emissions issued before one rate's window ends; each ordinal is a distinct tag byte.
const MAX_EMISSIONS: u8 = 24;
/// Emissions after the first captured one that must also arrive intact before a rate passes.
const CONFIRM_EMISSIONS: usize = 3;
/// Wall-clock bound on one rate's emit/capture window.
const EMIT_WINDOW: Duration = Duration::from_secs(6);
/// How long one emission's framed payload is waited for before it counts as not captured.
const CAPTURE_DEADLINE: Duration = Duration::from_millis(750);
/// Final drain after the last emission, for a buffered transport to hand over captured bytes.
const FINAL_DRAIN: Duration = Duration::from_millis(500);
const POLL_INTERVAL: Duration = Duration::from_millis(25);
/// The one rate the loaded variant runs; finding the ceiling is the unloaded sweep's job.
const LOAD_BAUD: u32 = 1_000_000;
/// Reads the loaded variant requires before its capture result counts as taken under load.
const MIN_LOAD_READS: u64 = 4;

/// Run one emit/capture window per rate; never `setup_tracing`, which reprograms the DUT's TPIU.
pub(crate) fn run(cx: &mut Ctx, _full_run: bool) -> Result<(), Failure> {
    inner(cx, None)
}

/// One window with a `read_32` loop running throughout; capture and the loop must both hold.
pub(crate) fn run_loaded(cx: &mut Ctx, _full_run: bool) -> Result<(), Failure> {
    let mut load = DapLoad::new(cx.read_addr);
    let outcome = inner(cx, Some(&mut load));
    // The load's own verdict only matters once capture has been judged.
    outcome.and_then(|()| load.progress(MIN_LOAD_READS).map_err(Failure::Failed))
}

fn inner(cx: &mut Ctx, mut load: Option<&mut DapLoad>) -> Result<(), Failure> {
    let dut_elf = cx.dut_elf.clone();
    // DBGMCU_CR: TRACE_IOEN (bit 5) routes the trace pin to the TPIU, TRACE_MODE (7:6) = async.
    let dbgmcu_cr = cx
        .dbgmcu_cr
        .ok_or_else(|| Failure::Skip("no trace-routing register on this target".into()))?;
    let session = cx
        .session
        .as_mut()
        .ok_or_else(|| Failure::Fatal("swo needs a successful connect".into()))?;

    // probe-rs rewrites debugger-domain state, so assert async-SWO routing here, before the sweep.
    {
        use probe_rs::MemoryInterface;
        let mut core = session.core(0).map_err(|e| Failure::Fatal(chain(e)))?;
        let cr = core
            .read_word_32(dbgmcu_cr)
            .map_err(|e| Failure::Failed(format!("route trace pin: {}", chain(e))))?;
        core.write_word_32(dbgmcu_cr, (cr & !(0b11 << 6)) | (1 << 5))
            .map_err(|e| Failure::Failed(format!("route trace pin: {}", chain(e))))?;
    }

    let bauds: &[u32] = if load.is_some() {
        &[LOAD_BAUD]
    } else {
        SWO_BAUDS
    };
    for &baud in bauds {
        run_at_rate(session, dut_elf.as_deref(), baud, load.as_deref_mut()).map_err(
            |f| match f {
                Failure::Failed(msg) => Failure::Failed(format!("SWO at {baud} baud: {msg}")),
                Failure::Fatal(msg) => Failure::Fatal(format!("SWO at {baud} baud: {msg}")),
                Failure::Skip(reason) => Failure::Skip(reason),
            },
        )?;
    }
    Ok(())
}

/// Reconfigure the DUT and probe to `baud`, then run and judge one emit/capture window.
fn run_at_rate(
    session: &mut probe_rs::Session,
    dut_elf: Option<&str>,
    baud: u32,
    load: Option<&mut DapLoad>,
) -> Result<(), Failure> {
    // Reprogram the DUT's TPIU over RTT first; the reply carries the realized rate.
    {
        let mut core = session.core(0).map_err(|e| Failure::Fatal(chain(e)))?;
        let mut rtt = rtt::attach(&mut core, dut_elf).map_err(Failure::Fatal)?;
        let seen = rtt::exchange(
            &mut rtt,
            &mut core,
            format!("SWO CFG {baud}\n").as_bytes(),
            "OK SWO CFG",
        )
        .map_err(|e| Failure::Failed(format!("set DUT baud: {e}")))?;
        let realized = seen
            .lines()
            .rev()
            .find_map(proto::swo_cfg_realized)
            .ok_or_else(|| Failure::Failed("no realized rate in the DUT reply".into()))?;
        if u64::from(realized.abs_diff(baud)) * 100
            > u64::from(baud) * u64::from(RATE_TOLERANCE_PCT)
        {
            return Err(Failure::Failed(format!(
                "DUT realized {realized}, outside the {RATE_TOLERANCE_PCT}% budget"
            )));
        }
    }

    // `enable_swo` re-issues transport/mode/baudrate/start, so disable first for a clean start.
    if let Ok(interface) = session.get_arm_interface() {
        let _ = interface.disable_swo();
    }
    {
        let interface = session
            .get_arm_interface()
            .map_err(|e| Failure::Fatal(chain(e)))?;
        let config = SwoConfig::new(TPIU_CLK_HINT).set_baud(baud).set_mode_uart();
        interface
            .enable_swo(&config)
            .map_err(|e| Failure::Failed(format!("enable SWO capture: {}", chain(e))))?;
    }

    let outcome = window(session, dut_elf, load);

    if let Ok(interface) = session.get_arm_interface() {
        let _ = interface.disable_swo();
    }

    let delivered = outcome?;
    decide(&delivered).map_err(Failure::Failed)
}

/// Emit each tag over RTT and poll for its framed bytes; returns the flags in issue order.
fn window(
    session: &mut probe_rs::Session,
    dut_elf: Option<&str>,
    mut load: Option<&mut DapLoad>,
) -> Result<Vec<bool>, Failure> {
    let mut delivered = Vec::new();
    let mut captured: Vec<u8> = Vec::new();
    let start = Instant::now();

    for tag in 1..=MAX_EMISSIONS {
        // Only issue an emission that can still be judged inside the window.
        if start.elapsed() + CAPTURE_DEADLINE >= EMIT_WINDOW {
            break;
        }

        {
            let mut core = session.core(0).map_err(|e| Failure::Fatal(chain(e)))?;
            let mut rtt = rtt::attach(&mut core, dut_elf).map_err(Failure::Fatal)?;
            let marker = format!("OK SWO EMIT {tag}");
            rtt::exchange(
                &mut rtt,
                &mut core,
                format!("SWO EMIT {tag}\n").as_bytes(),
                &marker,
            )
            .map_err(|e| Failure::Failed(format!("emit tag {tag}: {e}")))?;
        }

        // The competing DAP traffic the loaded variant is named for, between emissions.
        if let Some(load) = load.as_deref_mut() {
            load.tick_session(session);
        }

        let framed = framed(&payload_for(tag));
        let waited = Instant::now();
        let got = await_capture(session, &mut captured, &framed)?;
        // A rate past the ceiling always spends the deadline, so only a capture says how close it came.
        if got {
            crate::wait::note("swo.capture", CAPTURE_DEADLINE, waited.elapsed());
        }
        delivered.push(got);

        if let Some(first) = delivered.iter().position(|d| *d) {
            if delivered.len() - (first + 1) >= CONFIRM_EMISSIONS {
                break;
            }
        }
    }

    // A buffered transport may still hand bytes over: drain once more, then re-scan the misses.
    drain(session, &mut captured, FINAL_DRAIN)?;
    for (i, d) in delivered.iter_mut().enumerate() {
        if !*d && contains(&captured, &framed(&payload_for(i as u8 + 1))) {
            *d = true;
        }
    }
    Ok(delivered)
}

/// Poll `read_swo` until `target` appears or [`CAPTURE_DEADLINE`] elapses.
fn await_capture(
    session: &mut probe_rs::Session,
    captured: &mut Vec<u8>,
    target: &[u8],
) -> Result<bool, Failure> {
    let start = Instant::now();
    loop {
        read_chunk(session, captured)?;
        if contains(captured, target) {
            return Ok(true);
        }
        if start.elapsed() >= CAPTURE_DEADLINE {
            return Ok(false);
        }
        std::thread::sleep(POLL_INTERVAL);
    }
}

/// Read the probe's SWO buffer into `captured` for `budget`, then stop.
fn drain(
    session: &mut probe_rs::Session,
    captured: &mut Vec<u8>,
    budget: Duration,
) -> Result<(), Failure> {
    let start = Instant::now();
    loop {
        read_chunk(session, captured)?;
        if start.elapsed() >= budget {
            return Ok(());
        }
        std::thread::sleep(POLL_INTERVAL);
    }
}

fn read_chunk(session: &mut probe_rs::Session, captured: &mut Vec<u8>) -> Result<(), Failure> {
    let interface = session
        .get_arm_interface()
        .map_err(|e| Failure::Fatal(chain(e)))?;
    let chunk = interface
        .read_swo()
        .map_err(|e| Failure::Failed(format!("read SWO: {}", chain(e))))?;
    captured.extend_from_slice(&chunk);
    Ok(())
}

/// Passes when capture began and the next [`CONFIRM_EMISSIONS`] emissions arrived intact.
fn decide(delivered: &[bool]) -> Result<(), String> {
    if delivered.is_empty() {
        return Err("no emission was issued in the window".into());
    }
    let Some(first) = delivered.iter().position(|d| *d) else {
        return Err(format!(
            "no framed ITM payload captured from {} emission(s)",
            delivered.len()
        ));
    };
    let after = &delivered[first + 1..];
    if after.len() < CONFIRM_EMISSIONS {
        return Err(format!(
            "first capture at emission {} left only {} of {CONFIRM_EMISSIONS} confirmation \
             emission(s): the window ended before capture could be shown to hold",
            first + 1,
            after.len()
        ));
    }
    let missed = after[..CONFIRM_EMISSIONS].iter().filter(|d| !**d).count();
    if missed > 0 {
        return Err(format!(
            "capture is intermittent: {missed} of the {CONFIRM_EMISSIONS} emission(s) after the \
             first capture never arrived, with the receiver already listening"
        ));
    }
    Ok(())
}

/// The payload for emission `tag`: the shared fixed head plus `tag` as the final byte.
fn payload_for(tag: u8) -> [u8; proto::SWO_PAYLOAD_LEN] {
    let mut p = [0u8; proto::SWO_PAYLOAD_LEN];
    p[..proto::SWO_PAYLOAD_HEAD.len()].copy_from_slice(&proto::SWO_PAYLOAD_HEAD);
    p[proto::SWO_PAYLOAD_LEN - 1] = tag;
    p
}

/// The expected ITM byte stream for `payload`: each byte preceded by the port-0 8-bit header.
fn framed(payload: &[u8]) -> Vec<u8> {
    let mut v = Vec::with_capacity(payload.len() * 2);
    for &b in payload {
        v.push(ITM_PORT0_HEADER);
        v.push(b);
    }
    v
}

fn contains(haystack: &[u8], needle: &[u8]) -> bool {
    !needle.is_empty() && haystack.windows(needle.len()).any(|w| w == needle)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn framed_interleaves_header_and_tags_are_distinct() {
        let a = framed(&payload_for(1));
        let b = framed(&payload_for(2));
        assert_eq!(a.len(), proto::SWO_PAYLOAD_LEN * 2);
        assert_eq!(a[0], ITM_PORT0_HEADER);
        assert_ne!(a, b, "distinct tags must frame to distinct sequences");
    }

    #[test]
    fn contains_tolerates_noise_and_rejects_absent() {
        let f = framed(&payload_for(1));
        let mut stream = vec![0x00, 0xff, 0x01];
        stream.extend_from_slice(&f);
        stream.extend_from_slice(&[0x77]);
        assert!(contains(&stream, &f));
        assert!(!contains(&payload_for(1), &f), "header dropped -> no match");
        assert!(!contains(&[], &f));
    }

    #[test]
    fn decide_pass_and_fail_shapes() {
        // Late start then reliable delivery passes.
        assert!(decide(&[false, false, true, true, true, true]).is_ok());
        // One lucky capture among losses is intermittent, not a pass.
        assert!(decide(&[false, true, false, false, false]).is_err());
        // A window ending before confirmation is inconclusive, not a pass.
        assert!(decide(&[false, true, true]).is_err());
        assert!(decide(&[false; 6]).is_err());
        assert!(decide(&[]).is_err());
    }
}
