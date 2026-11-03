//! The rtt case: the boot event left by the flash case's reset, then a PING exchange.

use std::time::{Duration, Instant};

use daplink_test_protocol as proto;
use probe_rs::rtt::{Rtt, ScanRegion};
use probe_rs::Core;

use crate::util::chain;
use crate::{Ctx, Failure};

pub(crate) fn run(cx: &mut Ctx, _full_run: bool) -> Result<(), Failure> {
    // The DUT cases that follow talk over this channel.
    inner(cx).map_err(Failure::Fatal)
}

fn inner(cx: &mut Ctx) -> Result<(), String> {
    let session = cx.session.as_mut().ok_or("needs a successful connect")?;
    let mut core = session.core(0).map_err(chain)?;

    // Unless the previous case reset the target and left its boot event, regenerate the marker.
    if !std::mem::take(&mut cx.boot_pending) {
        crate::reset(&mut core)?;
    }

    let mut rtt = attach(&mut core, cx.dut_elf.as_deref())?;

    expect(&mut rtt, &mut core, proto::EVT_BOOT).map_err(|e| format!("boot event: {e}"))?;

    exchange(&mut rtt, &mut core, b"PING\n", proto::OK_PONG)
        .map_err(|e| format!("after PING: {e}"))?;
    Ok(())
}

/// Read the up channel until `marker` appears, returning everything seen. Shared with nreset.
pub(crate) fn expect(rtt: &mut Rtt, core: &mut Core, marker: &str) -> Result<String, String> {
    let channel = rtt.up_channel(0).ok_or("no RTT up channel")?;
    let budget = Duration::from_secs(3);
    crate::wait::timed("rtt.expect", budget, || {
        let deadline = Instant::now() + budget;
        accumulate_until_marker(deadline, Duration::from_millis(50), marker, || {
            let mut buf = [0u8; 256];
            let n = channel.read(core, &mut buf).map_err(chain)?;
            Ok(buf[..n].to_vec())
        })
    })
}

/// Wait for `marker`, rejecting complete DMA-abort errors before accepting success.
fn accumulate_until_marker(
    deadline: Instant,
    interval: Duration,
    marker: &str,
    mut read_more: impl FnMut() -> Result<Vec<u8>, String>,
) -> Result<String, String> {
    let mut seen = Vec::new();
    while Instant::now() < deadline {
        seen.extend_from_slice(&read_more()?);
        let text = String::from_utf8_lossy(&seen);
        if let Some(end) = text.rfind('\n') {
            if text[..=end]
                .lines()
                .any(|line| line == proto::ERR_DMA_ABORT_TIMEOUT)
            {
                return Err(format!(
                    "DUT rejected `{marker}`: {}; transfer ownership is unresolved",
                    proto::ERR_DMA_ABORT_TIMEOUT
                ));
            }
        }
        if text.contains(marker) {
            return Ok(text.into_owned());
        }
        std::thread::sleep(interval);
    }
    Err(format!(
        "no `{marker}` within the window (saw {:?})",
        String::from_utf8_lossy(&seen)
    ))
}

/// Send one command line and read until `marker` appears, returning everything seen.
pub(crate) fn exchange(
    rtt: &mut Rtt,
    core: &mut Core,
    line: &[u8],
    marker: &str,
) -> Result<String, String> {
    let down = rtt.down_channel(0).ok_or("no RTT down channel")?;
    let written = down.write(core, line).map_err(chain)?;
    // A truncated RTT write must not be mistaken for a DUT command rejection.
    full_write(written, line.len())?;
    expect(rtt, core, marker)
}

/// Reject a down-channel write the ring only partly took.
fn full_write(written: usize, len: usize) -> Result<(), String> {
    if written == len {
        return Ok(());
    }
    Err(format!(
        "the down channel took {written} of {len} command bytes, truncating it"
    ))
}

/// Attach RTT at the ELF's `_SEGGER_RTT` symbol when a DUT image is at hand, avoiding a RAM scan.
pub(crate) fn attach(core: &mut Core, dut_elf: Option<&str>) -> Result<Rtt, String> {
    attach_within(core, dut_elf, Duration::from_secs(15))
}

/// [`attach`] with a caller-bounded wait.
pub(crate) fn attach_within(
    core: &mut Core,
    dut_elf: Option<&str>,
    within: Duration,
) -> Result<Rtt, String> {
    let region = dut_elf
        .and_then(|elf| rtt_symbol(elf).ok())
        .map_or(ScanRegion::default(), ScanRegion::Exact);
    let deadline = Instant::now() + within;
    poll_until_deadline(deadline, Duration::from_millis(100), || {
        Rtt::attach_region(core, &region)
    })
    .map_err(|e| format!("RTT attach failed: {}", chain(e)))
}

/// Attach for an up-only stream, waiting for `up[0]` itself: a defmt control block has no down channel.
pub(crate) fn attach_up_only(
    core: &mut Core,
    dut_elf: Option<&str>,
    deadline: Duration,
) -> Result<Rtt, String> {
    let region = dut_elf
        .and_then(|elf| rtt_symbol(elf).ok())
        .map_or(ScanRegion::default(), ScanRegion::Exact);
    let bound = Instant::now() + deadline;
    poll_until_deadline(bound, Duration::from_millis(50), || {
        let mut rtt = Rtt::attach_region(core, &region).map_err(chain)?;
        if rtt.up_channel(0).is_some() {
            Ok(rtt)
        } else {
            Err("attached but up-channel 0 not yet published".to_string())
        }
    })
    .map_err(|e| format!("RTT attach failed: {e}"))
}

/// Retry `attempt` until success or `deadline`, returning the last error as-is.
fn poll_until_deadline<T, E>(
    deadline: Instant,
    interval: Duration,
    mut attempt: impl FnMut() -> Result<T, E>,
) -> Result<T, E> {
    loop {
        let err = match attempt() {
            Ok(v) => return Ok(v),
            Err(e) => e,
        };
        if Instant::now() >= deadline {
            return Err(err);
        }
        std::thread::sleep(interval);
    }
}

/// The `_SEGGER_RTT` control-block address in the ELF at `path`.
pub(crate) fn rtt_symbol(path: &str) -> Result<u64, String> {
    use object::{Object, ObjectSymbol};
    let bytes = std::fs::read(path).map_err(|e| format!("read ELF {path}: {e}"))?;
    let elf = object::File::parse(bytes.as_slice()).map_err(chain)?;
    elf.symbols()
        .find(|symbol| symbol.name() == Ok("_SEGGER_RTT"))
        .map(|symbol| symbol.address())
        .ok_or_else(|| format!("ELF {path} has no _SEGGER_RTT symbol"))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Short bound + fast poll so the timeout-shaped tests below cost milliseconds.
    const TEST_DEADLINE: Duration = Duration::from_millis(60);
    const TEST_INTERVAL: Duration = Duration::from_millis(1);

    #[test]
    fn poll_until_deadline_returns_the_first_success_after_transient_errors() {
        let mut attempts = 0;
        let got = poll_until_deadline(Instant::now() + TEST_DEADLINE, TEST_INTERVAL, || {
            attempts += 1;
            if attempts < 4 {
                Err("not yet")
            } else {
                Ok(attempts)
            }
        });
        assert_eq!(got, Ok(4));
    }

    #[test]
    fn poll_until_deadline_fails_at_the_bound_when_it_never_succeeds() {
        let start = Instant::now();
        let deadline = start + TEST_DEADLINE;
        let err = poll_until_deadline(deadline, TEST_INTERVAL, || Err::<(), _>("still failing"))
            .expect_err("an attempt that never succeeds must fail, not hang");
        assert_eq!(err, "still failing");
        assert!(
            start.elapsed() >= TEST_DEADLINE,
            "returned before the bound"
        );
        assert!(
            start.elapsed() < TEST_DEADLINE * 10,
            "the wait is unbounded"
        );
    }

    #[test]
    fn poll_until_deadline_retries_every_error_alike() {
        // Every attach error is retried alike, so differing errors must all reach the success.
        let errors = ["a", "b", "c"];
        let mut i = 0;
        let got = poll_until_deadline(Instant::now() + TEST_DEADLINE, TEST_INTERVAL, || {
            if i < errors.len() {
                let e = errors[i];
                i += 1;
                Err(e)
            } else {
                Ok(i)
            }
        });
        assert_eq!(got, Ok(errors.len()));
    }

    #[test]
    fn accumulate_until_marker_finds_a_marker_split_across_reads() {
        // Reads are unaligned to line boundaries, so a marker routinely lands split across calls.
        let mut chunks = vec![b"EVT BO".to_vec(), b"OT rp2040\n".to_vec()].into_iter();
        let got = accumulate_until_marker(
            Instant::now() + TEST_DEADLINE,
            TEST_INTERVAL,
            "EVT BOOT",
            || Ok(chunks.next().unwrap_or_default()),
        )
        .expect("the marker spans two reads but is still present");
        assert_eq!(got, "EVT BOOT rp2040\n");
    }

    #[test]
    fn accumulate_until_marker_rejects_fragmented_dma_error_before_success_or_another_read() {
        let mut chunks = [b"ERR dma_abort_timeout".as_slice(), b"\nOK BAUD 115200\n"].into_iter();
        let mut reads = 0;
        let error = accumulate_until_marker(
            Instant::now() + TEST_DEADLINE,
            Duration::ZERO,
            "OK BAUD",
            || {
                reads += 1;
                Ok(chunks.next().expect("no read after rejection").to_vec())
            },
        )
        .unwrap_err();
        assert_eq!(reads, 2);
        assert!(error.contains("ERR dma_abort_timeout"));
        assert!(error.contains("ownership is unresolved"));
        assert!(!error.contains("within the window"));
    }

    #[test]
    fn accumulate_until_marker_does_not_reinterpret_other_text_as_dma_rejection() {
        let reply = b"note ERR dma_abort_timeout\nOK PONG\n";
        assert!(accumulate_until_marker(
            Instant::now() + TEST_DEADLINE,
            Duration::ZERO,
            "OK PONG",
            || Ok(reply.to_vec()),
        )
        .is_ok());
    }

    #[test]
    fn accumulate_until_marker_times_out_reporting_what_it_saw() {
        let start = Instant::now();
        let deadline = start + TEST_DEADLINE;
        let err = accumulate_until_marker(deadline, TEST_INTERVAL, "OK PONG", || {
            Ok(b"garbage".to_vec())
        })
        .expect_err("a marker that never arrives must fail, not hang");
        assert!(err.contains("OK PONG"), "got {err:?}");
        assert!(err.contains("garbage"), "got {err:?}");
        assert!(
            start.elapsed() >= TEST_DEADLINE,
            "returned before the bound"
        );
        assert!(
            start.elapsed() < TEST_DEADLINE * 10,
            "the wait is unbounded"
        );
    }

    #[test]
    fn accumulate_until_marker_propagates_a_read_error_immediately() {
        // A channel-read error ends the loop at once; it is not an unmatched marker.
        let mut calls = 0;
        let err =
            accumulate_until_marker(Instant::now() + TEST_DEADLINE, TEST_INTERVAL, "X", || {
                calls += 1;
                Err("read failed".to_string())
            })
            .expect_err("a read error must not be swallowed by the retry");
        assert_eq!(err, "read failed");
        assert_eq!(calls, 1, "the read error was retried instead of propagated");
    }

    #[test]
    fn full_write_accepts_only_a_write_the_ring_took_whole() {
        assert_eq!(full_write(53, 53), Ok(()));
        let err = full_write(0, 53).expect_err("a refused write is not a sent command");
        assert!(err.contains("0 of 53"), "got {err:?}");
        // The RP images' down channel is 64 bytes, so a wider command is the exposed case.
        let err = full_write(64, 70).expect_err("a truncated command must not look like a reply");
        assert!(err.contains("64 of 70"), "got {err:?}");
    }

    #[test]
    fn rtt_symbol_fails_for_bytes_that_are_not_an_object_file() {
        let path = std::env::temp_dir().join("daplink-test-rtt-symbol-not-an-elf.bin");
        std::fs::write(&path, b"not an object file").expect("scratch file write");
        let got = rtt_symbol(path.to_str().expect("utf8 scratch path"));
        let _ = std::fs::remove_file(&path);
        assert!(got.is_err(), "got {got:?}");
    }

    #[test]
    fn rtt_symbol_fails_for_a_missing_file() {
        let err = rtt_symbol("/nonexistent/daplink-test-rtt-symbol.elf").expect_err("no file");
        assert!(err.starts_with("read ELF"), "got {err:?}");
    }
}
