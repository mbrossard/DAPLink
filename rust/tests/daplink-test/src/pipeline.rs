//! The pipeline case: a burst is answered in order, and excess is refused or delayed, never dropped.

use std::time::{Duration, Instant};

use crate::dap::RawDap;
use crate::{Ctx, Failure};

/// Requests whose replies differ, none touching the target or needing a session.
const CANDIDATES: &[&[u8]] = &[
    // DAP_Info(FirmwareVersion): a long reply, and the only DAP_Info id carrying a string.
    &[0x00, 0x04],
    // DAP_HostStatus(Connect, false).
    &[0x01, 0x00, 0x00],
    // DAP_TransferConfigure(idle 0, wait retries 100, match retries 0).
    &[0x04, 0x00, 0x64, 0x00, 0x00, 0x00],
    // DAP_Delay(0 us).
    &[0x09, 0x00, 0x00],
    // DAP_SWJ_Clock(1 MHz).
    &[0x11, 0x40, 0x42, 0x0F, 0x00],
    // DAP_SWD_Configure(default turnaround and data phase).
    &[0x13, 0x00],
    // DAP_JTAG_Configure(no devices declared).
    &[0x15, 0x00],
    // DAP_SWO_Transport(None).
    &[0x17, 0x00],
    // DAP_SWO_Mode(Off).
    &[0x18, 0x00],
    // DAP_SWO_Baudrate(1 MHz); the reply carries the rate the probe realized.
    &[0x19, 0x40, 0x42, 0x0F, 0x00],
    // DAP_SWO_Control(Stop).
    &[0x1A, 0x00],
    // The DAP_Info ids this engine answers with something of their own.
    &[0x00, 0xF0],
    &[0x00, 0xFF],
    &[0x00, 0xFE],
];

const PACKET_COUNT: &[u8] = &[0x00, 0xFE];

/// Requests queued past both rings, to reach the probe's refusal rather than its buffering.
const EXCESS_REQUESTS: usize = 8;

/// The excess has to clear a double-buffered endpoint and the reader's parked packet.
const _: () = assert!(EXCESS_REQUESTS >= 3);

/// One queued request: the bytes sent, and the reply only they produce.
type Plan = Vec<(&'static [u8], Vec<u8>)>;

/// Bound on a submission the probe must accept, and on each reply read once the queue is full.
const TIMEOUT: Duration = Duration::from_secs(2);

/// How long a request the probe may refuse is given before its refusal is accepted.
const BACKPRESSURE_TIMEOUT: Duration = Duration::from_millis(400);

/// Burst past both rings, check in-order replies and backpressure, then a clean `DAP_Disconnect`.
pub(crate) fn run(cx: &mut Ctx, _full_run: bool) -> Result<(), Failure> {
    let mut dap = crate::dap::open(&cx.selector).map_err(Failure::Fatal)?;

    let depth = packet_count(&mut dap).map_err(Failure::Fatal)?;

    // Each queued request needs a reply nothing else can produce, so a swap is visible.
    let Some(plan) = distinct_plan(&mut dap, depth.max(2)).map_err(Failure::Fatal)? else {
        return Err(Failure::Skip(format!(
            "pipeline: only {} distinguishable requests for a queue depth of {depth}",
            CANDIDATES.len()
        )));
    };
    let burst = burst_plan(&plan, burst_len(depth));

    // An OUT that stops completing is backpressure, allowed only once the depth is covered.
    let mut accepted = 0;
    for (i, (request, _)) in burst.iter().enumerate() {
        let timeout = if i < depth {
            TIMEOUT
        } else {
            BACKPRESSURE_TIMEOUT
        };
        let sending = Instant::now();
        if let Err(e) = dap.send(request, timeout) {
            if i < depth {
                return Err(Failure::Failed(format!(
                    "the probe refused request {} with only {i} outstanding, below the queue \
                     depth {depth} it advertises: {e}",
                    i + 1
                )));
            }
            break;
        }
        // A refusal always spends the bound, so only an accepted send says how close it came.
        if i >= depth {
            crate::wait::note(
                "pipeline.backpressure",
                BACKPRESSURE_TIMEOUT,
                sending.elapsed(),
            );
        }
        accepted += 1;
    }

    // One request past the burst: refusal or delay is fine; if it completes, its reply is required.
    let (probe_request, probe_reply) = burst[0];
    let probe_accepted = dap.send(probe_request, BACKPRESSURE_TIMEOUT).is_ok();

    // Every accepted request, in the order it was issued, then the probe request if it landed.
    let mut expected = burst[..accepted].to_vec();
    if probe_accepted {
        expected.push((probe_request, probe_reply));
    }

    for (i, (request, want)) in expected.iter().enumerate() {
        let got = dap.recv(TIMEOUT).map_err(|e| {
            Failure::Failed(format!(
                "reply {} of {} never arrived ({e}); the probe accepted {request:02x?} and then \
                 dropped it, which is the one outcome backpressure must never produce",
                i + 1,
                expected.len()
            ))
        })?;
        if &got != *want {
            return Err(Failure::Failed(format!(
                "reply {} of {} is {got:02x?}, not the {request:02x?} answer {want:02x?}: the \
                 burst was reordered or corrupted",
                i + 1,
                expected.len()
            )));
        }
    }

    // Disconnect, then one plain exchange: a stale reply would surface here as the wrong echo.
    let disconnect = dap.command(&[0x03]).map_err(Failure::Failed)?;
    if disconnect.first() != Some(&0x03) {
        return Err(Failure::Failed(format!(
            "DAP_Disconnect reply {disconnect:02x?} is a stale or foreign reply"
        )));
    }
    let after = dap.command(PACKET_COUNT).map_err(Failure::Failed)?;
    if after.first() != Some(&0x00) {
        return Err(Failure::Failed(format!(
            "the exchange after DAP_Disconnect read {after:02x?}, not its own reply"
        )));
    }
    Ok(())
}

/// How many requests a burst sends: the request ring, the response ring, then the excess.
fn burst_len(depth: usize) -> usize {
    2 * depth + EXCESS_REQUESTS
}

/// The burst, cycling the distinct plan so a dropped reply shifts every one after it.
fn burst_plan(plan: &Plan, burst: usize) -> Vec<(&'static [u8], &Vec<u8>)> {
    (0..burst)
        .map(|i| {
            let (request, reply) = &plan[i % plan.len()];
            (*request, reply)
        })
        .collect()
}

/// The probe's advertised queue depth.
fn packet_count(dap: &mut RawDap) -> Result<usize, String> {
    let count = dap.dap_info(0xFE)?;
    match count.first() {
        Some(&c) if c >= 1 => Ok(usize::from(c)),
        _ => Err(format!("packet count {count:02x?} is below 1")),
    }
}

/// Keep `depth` candidates whose replies differ; `None` (a skip) when too few do.
fn distinct_plan(dap: &mut RawDap, depth: usize) -> Result<Option<Plan>, String> {
    let mut plan = Plan::new();
    for request in CANDIDATES {
        let reply = dap.command(request)?;
        if plan.iter().all(|(_, seen)| seen != &reply) {
            plan.push((request, reply));
        }
        if plan.len() == depth {
            return Ok(Some(plan));
        }
    }
    Ok(None)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Distinct command bytes make replies distinguishable; `DAP_Info` entries differ by id.
    #[test]
    fn candidates_are_distinguishable_by_construction() {
        let mut keys: Vec<&[u8]> = CANDIDATES
            .iter()
            .map(|c| if c[0] == 0x00 { &c[..2] } else { &c[..1] })
            .collect();
        keys.sort_unstable();
        let before = keys.len();
        keys.dedup();
        assert_eq!(keys.len(), before, "two candidates share an identity");
    }

    /// The candidate list must cover the deepest advertised queue; the burst cycles it.
    #[test]
    fn there_are_enough_candidates_for_the_deepest_advertised_queue() {
        assert!(CANDIDATES.len() >= 8, "{}", CANDIDATES.len());
    }

    /// The burst outruns both rings of the deepest firmware and the endpoint's held packets.
    #[test]
    fn the_burst_outruns_both_rings() {
        assert_eq!(burst_len(8), 24);
        assert!(burst_len(4) >= 2 * 4 + 3);
    }

    /// Neighbouring replies always differ, so one dropped reply shifts every later position.
    #[test]
    fn a_dropped_reply_shifts_every_later_position() {
        let plan: Plan = vec![
            (&[0x00, 0x04], vec![0x00, 0x01, 0x41]),
            (&[0x01, 0x00, 0x00], vec![0x01, 0x00]),
            (&[0x09, 0x00, 0x00], vec![0x09, 0x00]),
        ];
        let burst = burst_plan(&plan, 8);
        assert_eq!(burst.len(), 8);
        assert_eq!(burst[7].0, plan[1].0);
        for k in 0..burst.len() - 1 {
            assert_ne!(burst[k].1, burst[k + 1].1, "position {k}");
        }
    }

    /// No candidate may reset, halt or drive the target.
    #[test]
    fn no_candidate_touches_the_target() {
        const FORBIDDEN: &[u8] = &[0x02, 0x03, 0x0A, 0x10, 0x12, 0x14, 0x1D, 0x05, 0x06];
        for c in CANDIDATES {
            assert!(!FORBIDDEN.contains(&c[0]), "candidate {c:02x?}");
        }
    }
}
