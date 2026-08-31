//! Wire-clock throughput sweep for the two speed cases: an instrument, never a gate.

use std::time::{Duration, Instant};

use probe_rs::probe::list::Lister;
use probe_rs::probe::{DebugProbeSelector, WireProtocol};
use probe_rs::{MemoryInterface, Permissions};

use crate::util::chain;
use crate::Failure;

/// Requested wire clocks swept at each step, in Hz. Fixed so results compare across runs.
pub(crate) const CLOCKS_HZ: [u32; 4] = [1_000_000, 4_000_000, 12_000_000, 25_000_000];
/// Bytes read per chunk.
const CHUNK: usize = 4096;
/// Wall-clock budget per step.
const BUDGET: Duration = Duration::from_millis(500);
/// Minimum wall-clock floor per step, alongside `MIN_CHUNKS`.
const MIN_TIME: Duration = Duration::from_millis(100);
/// Minimum chunk-count floor per step, alongside `MIN_TIME`.
const MIN_CHUNKS: u32 = 4;

/// One step's measured throughput.
pub(crate) struct Step {
    pub(crate) requested_hz: u32,
    pub(crate) kib_per_s: f64,
    pub(crate) chunks: u32,
    /// Elapsed nanoseconds of each timed read, raw so a clamped clock's equal-time signature
    /// survives into the record.
    pub(crate) chunk_ns: Vec<u128>,
}

/// Whether a step may stop: the budget has elapsed and both floors are met.
fn stop(elapsed: Duration, chunks: u32) -> bool {
    elapsed >= BUDGET && elapsed >= MIN_TIME && chunks >= MIN_CHUNKS
}

/// Sweep [`CLOCKS_HZ`] over `protocol`, one fresh session per step: probe-rs sets speed
/// pre-attach, so a live session cannot change it.
pub(crate) fn sweep(
    selector: &DebugProbeSelector,
    chip: &str,
    protocol: WireProtocol,
    read_addr: u64,
) -> Result<Vec<Step>, Failure> {
    let mut steps = Vec::with_capacity(CLOCKS_HZ.len());
    for &hz in &CLOCKS_HZ {
        steps.push(one_step(selector, chip, protocol, hz, read_addr)?);
    }
    Ok(steps)
}

/// Open at `hz`, attach, and read [`CHUNK`]-byte blocks from `read_addr` until [`stop`] is true.
fn one_step(
    selector: &DebugProbeSelector,
    chip: &str,
    protocol: WireProtocol,
    hz: u32,
    read_addr: u64,
) -> Result<Step, Failure> {
    let mut probe = Lister::new()
        .open(selector.clone())
        .map_err(|e| Failure::Fatal(format!("open probe: {}", chain(e))))?;
    probe
        .select_protocol(protocol)
        .map_err(|e| Failure::Failed(format!("select protocol: {}", chain(e))))?;
    // Requests above the probe's internal ~2 MHz delay clamp realize flat; that is expected.
    probe
        .set_speed(hz / 1000)
        .map_err(|e| Failure::Failed(format!("set speed: {}", chain(e))))?;
    let mut session = probe
        .attach(chip, Permissions::default())
        .map_err(|e| Failure::Failed(format!("attach at {hz} Hz: {}", chain(e))))?;
    let mut core = session
        .core(0)
        .map_err(|e| Failure::Fatal(format!("core(0) at {hz} Hz: {}", chain(e))))?;

    let mut buf = [0u32; CHUNK / 4];
    let mut chunks: u32 = 0;
    let mut elapsed = Duration::ZERO;
    let mut chunk_ns = Vec::new();
    loop {
        let started = Instant::now();
        core.read_32(read_addr, &mut buf)
            .map_err(|e| Failure::Failed(format!("read at {hz} Hz: {}", chain(e))))?;
        let step = started.elapsed();
        elapsed += step;
        chunk_ns.push(step.as_nanos());
        chunks += 1;
        if stop(elapsed, chunks) {
            break;
        }
    }
    let kib_per_s = (chunks as f64 * CHUNK as f64 / 1024.0) / elapsed.as_secs_f64();
    Ok(Step {
        requested_hz: hz,
        kib_per_s,
        chunks,
        chunk_ns,
    })
}

/// Bytes read per timed chunk -- constant across every rung, so a record needs it once.
pub(crate) const CHUNK_BYTES: usize = CHUNK;

/// Print one measured step, terse, one line, no trailing punctuation.
pub(crate) fn print_step(s: &Step) {
    println!(
        "    {:>10} Hz  {:>9.1} KiB/s  {:>4} chunks",
        s.requested_hz, s.kib_per_s, s.chunks
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stop_requires_budget_and_both_floors() {
        assert!(!stop(Duration::from_millis(50), MIN_CHUNKS)); // under budget
        assert!(!stop(Duration::from_millis(600), MIN_CHUNKS - 1)); // under MIN_CHUNKS
        assert!(stop(Duration::from_millis(600), MIN_CHUNKS)); // all satisfied
    }

    #[test]
    fn stop_keeps_reading_past_budget_for_floors() {
        // Budget met but MIN_CHUNKS not yet reached: must keep going.
        assert!(!stop(BUDGET, 1));
        // Budget and MIN_TIME met, MIN_CHUNKS now reached: may stop.
        assert!(stop(BUDGET, MIN_CHUNKS));
    }
}
