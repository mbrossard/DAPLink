//! The capture window: `DAP_SWO_*` config and drain, with an optional interleaved SWD load.

use std::time::{Duration, Instant};

use anyhow::{bail, Context, Result};
use dap_bench::dap::{self, TransferItem, TransferRequest};
use dap_bench::transport::Probe;

use crate::swo;

/// MEM-AP TAR lives at `A[3:2] = 0b01`.
const AP_A_TAR: u8 = 1;
/// MEM-AP DRW lives at `A[3:2] = 0b11`.
const AP_A_DRW: u8 = 3;
/// Response length of the single-transfer TAR write: command, count, status.
const TAR_RESPONSE_LEN: usize = 3;
/// USB round-trip budget for one submitted batch.
const BATCH_TIMEOUT: Duration = Duration::from_millis(2000);
/// Round-trip budget for the one-off configuration commands.
const CONTROL_TIMEOUT: Duration = Duration::from_millis(500);
/// Polls after `Control(Stop)` that drain whatever the probe still holds.
const DRAIN_POLLS: usize = 8;

/// The concurrent SWD read the `--swd-load` mode interleaves with the capture polls.
#[derive(Debug, Clone, Copy)]
pub struct LoadPlan {
    /// Address every block read re-arms TAR to.
    pub addr: u32,
    /// Words per block read.
    pub words: usize,
    /// DAP index byte carried in each transfer (0 over SWD).
    pub dap_index: u8,
}

/// One command position in a poll batch.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Slot {
    /// Re-arm the MEM-AP TAR for the block read that follows.
    TarWrite,
    /// A `DAP_TransferBlock` read of the load plan's chunk.
    BlockRead,
    /// A `DAP_SWO_Data` drain.
    SwoData,
}

impl Slot {
    /// The CMSIS-DAP command id this slot sends, which its response must echo back.
    pub fn command_id(self) -> u8 {
        match self {
            Slot::TarWrite => dap::DAP_TRANSFER,
            Slot::BlockRead => dap::DAP_TRANSFER_BLOCK,
            Slot::SwoData => swo::DAP_SWO_DATA,
        }
    }
}

/// Confirm every response echoes its slot's command id, so a desync names the kind that slipped.
fn check_slots(slots: &[Slot], responses: &[Vec<u8>]) -> Result<()> {
    if slots.len() != responses.len() {
        bail!(
            "batch returned {} response(s) for {} slot(s)",
            responses.len(),
            slots.len()
        );
    }
    for (pos, (slot, resp)) in slots.iter().zip(responses).enumerate() {
        let expected = slot.command_id();
        match resp.first() {
            Some(&found) if found == expected => {}
            Some(&found) => bail!(
                "slot {pos} ({slot:?}) sent command 0x{expected:02X} but its response carried \
                 0x{found:02X} ({} bytes){}",
                resp.len(),
                if found == 0xFF {
                    "; 0xFF is the probe's invalid-command reply"
                } else {
                    ""
                }
            ),
            None => bail!("slot {pos} ({slot:?}) got an empty response"),
        }
    }
    Ok(())
}

/// What one baud tier's capture window produced.
#[derive(Debug, Clone)]
pub struct Capture {
    /// Baud the probe reported realizing.
    pub actual_baud: u32,
    /// Every trace byte drained, in capture order.
    pub bytes: Vec<u8>,
    /// Wall time the capture window actually ran.
    pub elapsed: Duration,
    /// `DAP_SWO_Data` exchanges issued.
    pub polls: u64,
    /// Polls that came back completely full: the probe had at least as much as one packet holds,
    /// so the host was not keeping up.
    pub full_polls: u64,
    /// Polls the probe answered with capture inactive.
    pub inactive_polls: u64,
    /// Words read by the interleaved SWD load.
    pub swd_words: u64,
    /// Load reads whose data differed from the first chunk read at this tier.
    pub swd_mismatches: u64,
    /// Load transfers the probe did not acknowledge cleanly.
    pub swd_errors: u64,
    /// `DAP_SWO_Status` read once just before capture stopped.
    pub final_status: swo::Status,
}

/// Per-tier capture knobs.
#[derive(Debug, Clone, Copy)]
pub struct Config {
    /// Requested capture baud.
    pub baud: u32,
    /// Capture window.
    pub window: Duration,
    /// Commands kept outstanding on the bulk pair.
    pub depth: usize,
    /// Bytes requested per `DAP_SWO_Data`.
    pub data_count: u16,
    /// The interleaved SWD load, when `--swd-load` is on.
    pub load: Option<LoadPlan>,
}

/// The command sequence one batch submits: a plain capture polls, a loaded capture pushes a
/// TAR/block pair in front of each poll so both share one pipelined window.
pub fn batch_slots(depth: usize, loaded: bool) -> Vec<Slot> {
    let depth = depth.max(1);
    let mut slots = Vec::with_capacity(depth * 3);
    for _ in 0..depth {
        if loaded {
            slots.push(Slot::TarWrite);
            slots.push(Slot::BlockRead);
        }
        slots.push(Slot::SwoData);
    }
    slots
}

/// Encode `slots` into request buffers and their exact response lengths.
fn build_batch(slots: &[Slot], cfg: &Config) -> (Vec<Vec<u8>>, Vec<usize>) {
    let mut requests = Vec::with_capacity(slots.len());
    let mut lens = Vec::with_capacity(slots.len());
    for slot in slots {
        match slot {
            Slot::TarWrite => {
                let load = cfg.load.expect("a TAR slot only exists under a load plan");
                requests.push(dap::encode_transfer(
                    load.dap_index,
                    &[TransferItem {
                        request: TransferRequest::new(true, false, AP_A_TAR),
                        data: Some(load.addr),
                    }],
                ));
                lens.push(TAR_RESPONSE_LEN);
            }
            Slot::BlockRead => {
                let load = cfg
                    .load
                    .expect("a block slot only exists under a load plan");
                requests.push(dap::encode_transfer_block(
                    load.dap_index,
                    load.words as u16,
                    TransferRequest::new(true, true, AP_A_DRW),
                    &[],
                ));
                lens.push(4 + load.words * 4);
            }
            Slot::SwoData => {
                requests.push(swo::encode_data(cfg.data_count));
                lens.push(swo::data_response_len(cfg.data_count));
            }
        }
    }
    (requests, lens)
}

/// Program transport, mode and baud, returning the baud the probe realized.
fn configure(probe: &mut Probe, baud: u32) -> Result<u32> {
    let resp = probe.transfer(
        &swo::encode_transport(swo::Transport::DapCommand),
        CONTROL_TIMEOUT,
    )?;
    swo::decode_transport(&resp).context("probe refused the command-based SWO transport")?;

    let resp = probe.transfer(&swo::encode_mode(swo::Mode::Uart), CONTROL_TIMEOUT)?;
    swo::decode_mode(&resp).context("probe refused SWO UART mode")?;

    let resp = probe.transfer(&swo::encode_baudrate(baud), CONTROL_TIMEOUT)?;
    let actual = swo::decode_baudrate(&resp)?;
    if actual == 0 {
        bail!("probe realized no baud for a {baud} baud request");
    }
    Ok(actual)
}

/// Send one `DAP_SWO_Control`.
fn control(probe: &mut Probe, control: swo::Control) -> Result<()> {
    let resp = probe.transfer(&swo::encode_control(control), CONTROL_TIMEOUT)?;
    swo::decode_control(&resp).with_context(|| format!("probe refused SWO {control:?}"))?;
    Ok(())
}

/// Read `DAP_SWO_Status`.
fn status(probe: &mut Probe) -> Result<swo::Status> {
    let resp = probe.transfer(&swo::encode_status(), CONTROL_TIMEOUT)?;
    Ok(swo::decode_status(&resp)?)
}

/// Configure, capture for the window, stop, and drain; a failure stops the capture first, so the
/// probe is never left capturing.
pub fn run(probe: &mut Probe, cfg: &Config) -> Result<Capture> {
    if cfg.data_count == 0 {
        bail!("the probe's packet size leaves no room for SWO payload");
    }
    let actual_baud = configure(probe, cfg.baud)?;
    control(probe, swo::Control::Start)?;

    let outcome = window(probe, cfg);
    // The stop sequence must not inherit the window's desync and report it as its own.
    if probe.is_desynced() {
        let _ = probe.resynchronize();
    }
    let final_status = match status(probe) {
        Ok(status) => status,
        Err(_) => {
            // A status that will not decode means the stream is off by one.
            probe.mark_desynced();
            swo::Status {
                active: false,
                bytes_available: 0,
            }
        }
    };
    let stop = control(probe, swo::Control::Stop);

    let mut capture = outcome?;
    stop?;
    drain(probe, cfg, &mut capture)?;
    capture.actual_baud = actual_baud;
    capture.final_status = final_status;
    Ok(capture)
}

/// Poll (and, under a load plan, read) until the window elapses.
fn window(probe: &mut Probe, cfg: &Config) -> Result<Capture> {
    let slots = batch_slots(cfg.depth, cfg.load.is_some());
    let (requests, lens) = build_batch(&slots, cfg);
    let mut cap = empty_capture();
    let mut reference: Option<Vec<u32>> = None;
    let started = Instant::now();
    let mut batches = 0u64;

    while started.elapsed() < cfg.window {
        if crate::interrupt::triggered() {
            bail!("interrupted");
        }
        batches += 1;
        let responses = probe
            .submit_pipelined(&requests, &lens, cfg.depth.max(1), BATCH_TIMEOUT)
            .with_context(|| {
                format!(
                    "SWO poll batch {batches} failed {:.1} ms into the window",
                    started.elapsed().as_secs_f64() * 1000.0
                )
            })?;
        if let Err(e) = check_slots(&slots, &responses) {
            probe.mark_desynced();
            return Err(e).with_context(|| {
                format!(
                    "SWO poll batch {batches}, {:.1} ms into the window",
                    started.elapsed().as_secs_f64() * 1000.0
                )
            });
        }
        for (slot, resp) in slots.iter().zip(&responses) {
            match slot {
                Slot::TarWrite => {
                    let decoded = dap::decode_transfer(resp)?;
                    if !decoded.status.is_ok() || decoded.transfer_count != 1 {
                        cap.swd_errors += 1;
                    }
                }
                Slot::BlockRead => {
                    let decoded = dap::decode_transfer_block(resp)?;
                    if !decoded.status.is_ok() {
                        cap.swd_errors += 1;
                        continue;
                    }
                    cap.swd_words += u64::from(decoded.transfer_count);
                    match &reference {
                        None => reference = Some(decoded.read_data),
                        Some(first) if *first != decoded.read_data => cap.swd_mismatches += 1,
                        Some(_) => {}
                    }
                }
                Slot::SwoData => {
                    let data = swo::decode_data(resp)?;
                    cap.polls += 1;
                    if !data.active {
                        cap.inactive_polls += 1;
                    }
                    if data.bytes.len() >= usize::from(cfg.data_count) {
                        cap.full_polls += 1;
                    }
                    cap.bytes.extend_from_slice(&data.bytes);
                }
            }
        }
    }

    cap.elapsed = started.elapsed();
    Ok(cap)
}

/// Drain the bytes the probe still buffers once capture has stopped.
fn drain(probe: &mut Probe, cfg: &Config, cap: &mut Capture) -> Result<()> {
    let request = swo::encode_data(cfg.data_count);
    let expected = swo::data_response_len(cfg.data_count);
    for _ in 0..DRAIN_POLLS {
        let resp = probe.transfer_expect(&request, expected, CONTROL_TIMEOUT)?;
        let data = match swo::decode_data(&resp) {
            Ok(data) => data,
            Err(e) => {
                probe.mark_desynced();
                return Err(e.into());
            }
        };
        cap.polls += 1;
        if data.bytes.is_empty() {
            break;
        }
        cap.bytes.extend_from_slice(&data.bytes);
    }
    Ok(())
}

/// A zeroed capture the window fills in.
fn empty_capture() -> Capture {
    Capture {
        actual_baud: 0,
        bytes: Vec::new(),
        elapsed: Duration::ZERO,
        polls: 0,
        full_polls: 0,
        inactive_polls: 0,
        swd_words: 0,
        swd_mismatches: 0,
        swd_errors: 0,
        final_status: swo::Status {
            active: false,
            bytes_available: 0,
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn config(load: Option<LoadPlan>) -> Config {
        Config {
            baud: 1_000_000,
            window: Duration::from_millis(10),
            depth: 2,
            data_count: 508,
            load,
        }
    }

    fn plan() -> LoadPlan {
        LoadPlan {
            addr: 0x0800_0000,
            words: 127,
            dap_index: 0,
        }
    }

    #[test]
    fn an_unloaded_batch_is_all_polls_one_per_pipeline_slot() {
        assert_eq!(batch_slots(3, false), vec![Slot::SwoData; 3]);
        assert_eq!(batch_slots(0, false), vec![Slot::SwoData]);
    }

    #[test]
    fn a_loaded_batch_puts_a_tar_block_pair_in_front_of_every_poll() {
        assert_eq!(
            batch_slots(2, true),
            vec![
                Slot::TarWrite,
                Slot::BlockRead,
                Slot::SwoData,
                Slot::TarWrite,
                Slot::BlockRead,
                Slot::SwoData,
            ]
        );
    }

    #[test]
    fn unloaded_requests_are_swo_data_sized_to_the_packet() {
        let cfg = config(None);
        let (requests, lens) = build_batch(&batch_slots(2, false), &cfg);
        assert_eq!(requests.len(), 2);
        assert_eq!(requests[0], swo::encode_data(508));
        assert_eq!(lens, vec![512, 512]);
    }

    #[test]
    fn every_slot_names_the_command_id_its_response_must_echo() {
        assert_eq!(Slot::TarWrite.command_id(), dap::DAP_TRANSFER);
        assert_eq!(Slot::BlockRead.command_id(), dap::DAP_TRANSFER_BLOCK);
        assert_eq!(Slot::SwoData.command_id(), swo::DAP_SWO_DATA);
        // The ids a batch encodes are exactly the ids its slots promise.
        let cfg = config(Some(plan()));
        let slots = batch_slots(2, true);
        let (requests, _) = build_batch(&slots, &cfg);
        for (slot, req) in slots.iter().zip(&requests) {
            assert_eq!(req[0], slot.command_id());
        }
    }

    #[test]
    fn an_aligned_batch_passes_the_slot_check() {
        let slots = batch_slots(1, true);
        let responses = vec![
            vec![dap::DAP_TRANSFER, 1, 0x01],
            vec![dap::DAP_TRANSFER_BLOCK, 127, 0, 0x01],
            vec![swo::DAP_SWO_DATA, 0x01, 0, 0],
        ];
        assert!(check_slots(&slots, &responses).is_ok());
    }

    #[test]
    fn the_invalid_command_reply_in_a_poll_slot_is_reported_with_its_position() {
        let slots = batch_slots(1, true);
        let responses = vec![
            vec![dap::DAP_TRANSFER, 1, 0x01],
            vec![dap::DAP_TRANSFER_BLOCK, 127, 0, 0x01],
            vec![0xFF],
        ];
        let err = check_slots(&slots, &responses).unwrap_err().to_string();
        assert!(err.contains("slot 2"), "{err}");
        assert!(err.contains("SwoData"), "{err}");
        assert!(err.contains("0x1C"), "{err}");
        assert!(err.contains("0xFF"), "{err}");
        assert!(err.contains("invalid-command reply"), "{err}");
    }

    #[test]
    fn a_batch_shifted_by_one_names_the_slot_that_slipped() {
        let slots = batch_slots(1, true);
        // The poll slot received the block read's answer: the classic off-by-one.
        let responses = vec![
            vec![dap::DAP_TRANSFER, 1, 0x01],
            vec![dap::DAP_TRANSFER_BLOCK, 127, 0, 0x01],
            vec![dap::DAP_TRANSFER_BLOCK, 127, 0, 0x01],
        ];
        let err = check_slots(&slots, &responses).unwrap_err().to_string();
        assert!(err.contains("slot 2"), "{err}");
        assert!(err.contains("0x06"), "{err}");
        // A short batch is a mismatch of its own, not a silent zip truncation.
        assert!(check_slots(&slots, &responses[..2]).is_err());
        assert!(check_slots(&slots, &[vec![], vec![], vec![]]).is_err());
    }

    #[test]
    fn loaded_requests_re_arm_tar_before_each_block_read() {
        let cfg = config(Some(plan()));
        let (requests, lens) = build_batch(&batch_slots(1, true), &cfg);
        assert_eq!(requests.len(), 3);
        assert_eq!(requests[0][0], dap::DAP_TRANSFER);
        assert_eq!(requests[1][0], dap::DAP_TRANSFER_BLOCK);
        assert_eq!(requests[2][0], swo::DAP_SWO_DATA);
        // TAR write, block read of 127 words, and one full-packet SWO drain.
        assert_eq!(lens, vec![3, 4 + 127 * 4, 512]);
    }
}
