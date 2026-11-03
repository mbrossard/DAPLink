//! The msc case: push `--dut-uf2` onto the probe's volume over raw SCSI/BOT and verify it.

use std::ops::Range;
use std::time::{Duration, Instant};

use nusb::transfer::{Buffer, Bulk, Direction, In, Out, TransferError};
use nusb::{Endpoint, MaybeFuture};
use probe_rs::probe::DebugProbeSelector;

use crate::msc_wire::{
    self, BootLayout, SectorIo, CSW_LEN, CSW_PASSED, DETAILS_NAME, IMAGE_NAME, OP_READ_10,
    OP_WRITE_10, SECTOR_SIZE,
};
use crate::uf2::{
    block_addr, corrupt_block, first_block, image_span, middle_block, verify, UF2_BLOCK,
};
use crate::{Ctx, Failure};
use probe_rs::config::FlashProperties;
use probe_rs::MemoryInterface;

const TIMEOUT: Duration = Duration::from_secs(5);
const OP_INQUIRY: u8 = 0x12;
const OP_TEST_UNIT_READY: u8 = 0x00;
const OP_READ_CAPACITY_10: u8 = 0x25;
/// How long a fresh mount is given to answer TEST UNIT READY before the case gives up.
const READY_TIMEOUT: Duration = Duration::from_secs(5);
/// Past the device's 20 s idle timeout, its 2.5 s detach and a D+ bounce, so a stuck drop still reports.
const REMOUNT_TIMEOUT: Duration = Duration::from_secs(30);
const POLL_INTERVAL: Duration = Duration::from_millis(100);
/// How long a rejected drop may keep the DAP wire before the hold is the finding.
const HOLD_GRACE: Duration = Duration::from_secs(3);
/// Bound on waiting for the wire back, well past the firmware's own idle timeouts.
const WIRE_RELEASE_TIMEOUT: Duration = Duration::from_secs(45);
/// Bound on a CSW after a cleared stall; short, since a remounting volume never answers.
const CSW_AFTER_STALL_TIMEOUT: Duration = Duration::from_millis(1000);

/// How a tolerated write ended: which LBA, and a CSW failure or the device going away.
#[derive(Clone, Copy)]
struct Refusal {
    lba: u32,
    detached: bool,
}

impl Refusal {
    /// How the write ended, for a verdict or the case's own report.
    fn describe(&self) -> String {
        let how = if self.detached {
            "ended with the device detaching"
        } else {
            "was refused"
        };
        format!("the write at lba {} {how}", self.lba)
    }
}

/// The probe's claimed MSC/BOT interface and its bulk endpoint pair.
struct MscBulk {
    _interface: nusb::Interface,
    out: Endpoint<Bulk, Out>,
    inp: Endpoint<Bulk, In>,
    tag: u32,
    /// Corrupt-drop tolerance: the first LBA from which a WRITE(10) refusal is expected.
    refuse_from: Option<u32>,
    refused: Option<Refusal>,
    /// Whether the last command failed because the device went away mid-transfer.
    detached: bool,
}

/// What a finished write phase leaves to read the device's report on it.
struct Programmed {
    details_lba: u32,
    /// `DETAILS.TXT`'s remount count before the drop.
    remounts: u32,
    refused: Option<Refusal>,
}

impl MscBulk {
    fn next_tag(&mut self) -> u32 {
        self.tag = self.tag.wrapping_add(1);
        self.tag
    }

    /// One CBW/data/CSW exchange; a stall in the data stage or on the CSW is cleared, then the CSW read.
    fn command(
        &mut self,
        cb: &[u8],
        dir_in: bool,
        data_len: u32,
        out_payload: Option<&[u8]>,
    ) -> Result<(Vec<u8>, u8), String> {
        let tag = self.next_tag();
        self.detached = false;
        let cbw = msc_wire::build_cbw(tag, data_len, dir_in, cb);
        if let Err(e) = self
            .out
            .transfer_blocking(Buffer::from(cbw.to_vec()), TIMEOUT)
            .into_result()
        {
            return Err(self.transport_failed("CBW write", e));
        }
        let mut in_data = Vec::new();
        if data_len > 0 {
            if dir_in {
                // An IN request must be a whole number of packets; a short packet ends it.
                let want = (data_len as usize).next_multiple_of(self.inp.max_packet_size());
                match self
                    .inp
                    .transfer_blocking(Buffer::new(want), TIMEOUT)
                    .into_result()
                {
                    Ok(buf) => in_data = buf.into_vec(),
                    Err(TransferError::Stall) => self
                        .inp
                        .clear_halt()
                        .wait()
                        .map_err(|e| format!("clearing the bulk-in halt failed: {e}"))?,
                    Err(e) => return Err(self.transport_failed("data-in phase", e)),
                }
            } else {
                let payload = out_payload.ok_or("write command missing its OUT payload")?;
                match self
                    .out
                    .transfer_blocking(Buffer::from(payload.to_vec()), TIMEOUT)
                    .into_result()
                {
                    Ok(_) => {}
                    Err(TransferError::Stall) => self
                        .out
                        .clear_halt()
                        .wait()
                        .map_err(|e| format!("clearing the bulk-out halt failed: {e}"))?,
                    Err(e) => return Err(self.transport_failed("data-out phase", e)),
                }
            }
        }
        let csw = self.read_csw().map_err(|e| format!("CDB {cb:02x?}: {e}"))?;
        let (status, _residue) = msc_wire::parse_csw(&csw, tag)?;
        Ok((in_data, status))
    }

    /// Read the CSW, clearing the bulk-in halt once; a second stall is a protocol violation.
    fn read_csw(&mut self) -> Result<Vec<u8>, String> {
        let want = CSW_LEN.next_multiple_of(self.inp.max_packet_size());
        let mut timeout = TIMEOUT;
        for attempt in 0..2 {
            match self
                .inp
                .transfer_blocking(Buffer::new(want), timeout)
                .into_result()
            {
                Ok(buf) => return Ok(buf.into_vec()),
                Err(TransferError::Stall) if attempt == 0 => {
                    self.inp
                        .clear_halt()
                        .wait()
                        .map_err(|e| format!("clearing the bulk-in halt failed: {e}"))?;
                    timeout = CSW_AFTER_STALL_TIMEOUT;
                }
                Err(e) => return Err(self.transport_failed("CSW read", e)),
            }
        }
        Err(
            "CSW read failed: the device stalled the bulk-in pipe again after the halt was \
             cleared"
                .into(),
        )
    }

    /// Describe a failed transfer, latching whether the device went away rather than answering.
    fn transport_failed(&mut self, phase: &str, e: TransferError) -> String {
        self.detached = matches!(e, TransferError::Cancelled | TransferError::Disconnected);
        format!("{phase} failed: {e}")
    }

    fn inquiry(&mut self) -> Result<(), String> {
        let cb = [OP_INQUIRY, 0, 0, 0, 36, 0];
        let (data, status) = self.command(&cb, true, 36, None)?;
        if status != CSW_PASSED {
            return Err(format!("INQUIRY CSW status {status}"));
        }
        if data.len() < 36 {
            return Err(format!("INQUIRY reply too short: {} bytes", data.len()));
        }
        Ok(())
    }

    fn test_unit_ready(&mut self) -> Result<u8, String> {
        let cb = [OP_TEST_UNIT_READY, 0, 0, 0, 0, 0];
        let (_data, status) = self.command(&cb, false, 0, None)?;
        Ok(status)
    }

    fn read_capacity(&mut self) -> Result<(u32, u32), String> {
        let cb = [OP_READ_CAPACITY_10, 0, 0, 0, 0, 0, 0, 0, 0, 0];
        let (data, status) = self.command(&cb, true, 8, None)?;
        if status != CSW_PASSED {
            return Err(format!("READ CAPACITY(10) CSW status {status}"));
        }
        if data.len() < 8 {
            return Err("READ CAPACITY(10) reply too short".into());
        }
        let last_lba = u32::from_be_bytes(data[0..4].try_into().unwrap());
        let block_size = u32::from_be_bytes(data[4..8].try_into().unwrap());
        Ok((last_lba, block_size))
    }

    fn read10(&mut self, lba: u32, count: u16) -> Result<Vec<u8>, String> {
        let cb = msc_wire::rw10_cb(OP_READ_10, lba, count);
        let data_len = count as u32 * SECTOR_SIZE as u32;
        let (data, status) = self.command(&cb, true, data_len, None)?;
        if status != CSW_PASSED {
            return Err(format!(
                "READ(10) lba={lba} count={count} CSW status {status}"
            ));
        }
        if data.len() != data_len as usize {
            return Err(format!(
                "READ(10) lba={lba} short reply: {} of {data_len} bytes",
                data.len()
            ));
        }
        Ok(data)
    }

    /// One WRITE(10), returning the CSW status rather than judging it.
    fn write10_status(&mut self, lba: u32, count: u16, data: &[u8]) -> Result<u8, String> {
        let cb = msc_wire::rw10_cb(OP_WRITE_10, lba, count);
        let data_len = count as u32 * SECTOR_SIZE as u32;
        if data.len() != data_len as usize {
            return Err(format!(
                "WRITE(10) payload is {} bytes, want {data_len}",
                data.len()
            ));
        }
        let (_data, status) = self.command(&cb, false, data_len, Some(data))?;
        Ok(status)
    }

    /// Accept a WRITE(10) refusal from `lba` onwards, for the one drop that expects one.
    fn allow_write_refusal_from(&mut self, lba: Option<u32>) {
        self.refuse_from = lba;
        self.refused = None;
    }

    /// How the tolerated write ended, clearing the tolerance so no later phase inherits it.
    fn take_refusal(&mut self) -> Option<Refusal> {
        self.refuse_from = None;
        self.refused.take()
    }
}

impl SectorIo for MscBulk {
    fn read_sector(&mut self, lba: u32) -> Result<[u8; SECTOR_SIZE], String> {
        let data = self.read10(lba, 1)?;
        let mut out = [0u8; SECTOR_SIZE];
        out.copy_from_slice(&data);
        Ok(out)
    }

    fn write_sector(&mut self, lba: u32, data: &[u8; SECTOR_SIZE]) -> Result<(), String> {
        let tolerated = self.refuse_from.is_some_and(|from| lba >= from);
        let status = match self.write10_status(lba, 1, data) {
            Ok(status) => status,
            // A remounting volume can end the offending transfer with the device gone instead of a CSW.
            Err(e) if tolerated && self.detached => {
                self.refused = Some(Refusal {
                    lba,
                    detached: true,
                });
                return Err(format!("WRITE(10) lba={lba}: {e}"));
            }
            Err(e) => return Err(e),
        };
        if status == CSW_PASSED {
            return Ok(());
        }
        // A refusal at or after the tolerated LBA ends the write phase; see `take_refusal`.
        if tolerated {
            self.refused = Some(Refusal {
                lba,
                detached: false,
            });
        }
        Err(format!("WRITE(10) lba={lba} count=1 CSW status {status}"))
    }
}

/// Open the probe's MSC/BOT interface; `Ok(None)` when no interface is mass storage.
fn open_msc(selector: &DebugProbeSelector) -> Result<Option<MscBulk>, String> {
    let device = crate::dap::open_device(selector)?;
    let config = device.active_configuration().map_err(|e| e.to_string())?;

    let mut found = None;
    for group in config.interfaces() {
        let alt = group.first_alt_setting();
        if !msc_wire::is_msc_bot_class(alt.class(), alt.subclass(), alt.protocol()) {
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
    let Some((ifnum, out_addr, in_addr)) = found else {
        return Ok(None);
    };
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
    Ok(Some(MscBulk {
        _interface: interface,
        out,
        inp,
        tag: 0,
        refuse_from: None,
        refused: None,
        detached: false,
    }))
}

/// Poll TEST UNIT READY until the volume answers ready, or `deadline` passes.
fn wait_ready(bulk: &mut MscBulk, deadline: Instant) -> Result<(), String> {
    loop {
        if let Ok(status) = bulk.test_unit_ready() {
            if status == CSW_PASSED {
                return Ok(());
            }
        }
        if Instant::now() >= deadline {
            return Err("device did not report ready in time".into());
        }
        std::thread::sleep(POLL_INTERVAL);
    }
}

/// Wait out either remount shape: a UNIT ATTENTION latch, or a D+ bounce needing a reopen.
fn wait_ready_after_remount(
    selector: &DebugProbeSelector,
    bulk: &mut MscBulk,
    deadline: Instant,
) -> Result<(), String> {
    loop {
        match bulk.test_unit_ready() {
            Ok(status) if status == CSW_PASSED => return Ok(()),
            Ok(_) => {}
            Err(_) => {
                if let Ok(Some(fresh)) = open_msc(selector) {
                    *bulk = fresh;
                }
            }
        }
        if Instant::now() >= deadline {
            return Err("device did not become ready after the drop's remount".into());
        }
        std::thread::sleep(POLL_INTERVAL);
    }
}

/// The device's own verdict on a drop: a clean `Flash: ... -> Done` and nothing else.
fn is_done(status: &str) -> bool {
    status.starts_with("Flash:") && status.contains("-> Done")
}

/// Poll `read` until `DETAILS.TXT` counts a remount past `before`, and return that sector's `Status:`.
fn status_after_remount(
    mut read: impl FnMut() -> Result<[u8; SECTOR_SIZE], String>,
    before: u32,
    deadline: Instant,
) -> Result<String, Failure> {
    loop {
        let last = match read() {
            Ok(body) => match msc_wire::remount_count(&body) {
                // Above, not unequal: a rebooted probe restarts the count and seeds the old status.
                Some(now) if now > before => {
                    return msc_wire::status_line(&body).ok_or_else(|| {
                        Failure::Failed("DETAILS.TXT carries no Status: line".into())
                    });
                }
                now => {
                    let status = msc_wire::status_line(&body).unwrap_or_default();
                    format!("remount count {now:?}, status `{status}`")
                }
            },
            Err(e) => e,
        };
        if Instant::now() >= deadline {
            return Err(Failure::Fatal(format!(
                "DETAILS.TXT never counted a remount past {before} after the drop; last read: {last}"
            )));
        }
        std::thread::sleep(POLL_INTERVAL);
    }
}

/// Read `count` sectors from `lba` one at a time through `SectorIo`, concatenated.
fn read_multi<W: SectorIo>(w: &mut W, lba: u32, count: u32) -> Result<Vec<u8>, String> {
    let mut out = Vec::with_capacity(count as usize * SECTOR_SIZE);
    for i in 0..count {
        out.extend_from_slice(&w.read_sector(lba + i)?);
    }
    Ok(out)
}

/// Program `image` as a new file.
fn program(
    bulk: &mut MscBulk,
    image: &[u8],
    corrupt_index: Option<usize>,
) -> Result<Programmed, Failure> {
    bulk.inquiry().map_err(Failure::Fatal)?;
    crate::wait::timed("msc.ready", READY_TIMEOUT, || {
        wait_ready(bulk, Instant::now() + READY_TIMEOUT)
    })
    .map_err(Failure::Fatal)?;
    let (last_lba, block_size) = bulk.read_capacity().map_err(Failure::Fatal)?;
    if block_size as usize != SECTOR_SIZE || last_lba == 0 {
        return Err(Failure::Fatal(format!(
            "implausible capacity: last_lba={last_lba} block_size={block_size}"
        )));
    }

    let boot = bulk.read_sector(0).map_err(Failure::Fatal)?;
    let layout = BootLayout::parse(&boot).map_err(Failure::Fatal)?;

    let mut root = read_multi(bulk, layout.root_dir_lba(), layout.root_dir_sectors())
        .map_err(Failure::Fatal)?;
    let details = msc_wire::find_entry(&root, &DETAILS_NAME)
        .ok_or_else(|| Failure::Fatal("DETAILS.TXT dirent missing from root directory".into()))?;
    let details_lba = layout.cluster_to_lba(details.first_cluster);
    let remounts = msc_wire::remount_count(&bulk.read_sector(details_lba).map_err(Failure::Fatal)?)
        .ok_or_else(|| Failure::Fatal("DETAILS.TXT carries no Remount count: line".into()))?;

    let first_cluster = msc_wire::next_free_cluster(&root, &layout);
    let sectors_needed = (image.len() as u32).div_ceil(SECTOR_SIZE as u32).max(1);
    let clusters_needed = sectors_needed.div_ceil(layout.sectors_per_cluster());

    let slot = msc_wire::find_free_slot(&root)
        .ok_or_else(|| Failure::Fatal("root directory has no free slot".into()))?;
    let dirent = msc_wire::build_dirent(&IMAGE_NAME, 0x20, first_cluster, image.len() as u32);
    root[slot..slot + 32].copy_from_slice(&dirent);
    let sector_in_root = slot / SECTOR_SIZE;
    let dirent_lba = layout.root_dir_lba() + sector_in_root as u32;
    let mut dirent_sector = [0u8; SECTOR_SIZE];
    dirent_sector.copy_from_slice(
        &root[sector_in_root * SECTOR_SIZE..sector_in_root * SECTOR_SIZE + SECTOR_SIZE],
    );

    // A uf2 block is one sector, so the damaged block's index is its offset into the data run.
    let refuse_from = corrupt_index.map(|i| layout.cluster_to_lba(first_cluster) + i as u32);
    bulk.allow_write_refusal_from(refuse_from);

    let written = msc_wire::program_image(
        bulk,
        &layout,
        first_cluster,
        clusters_needed,
        image,
        dirent_lba,
        &dirent_sector,
    );
    let refused = bulk.take_refusal();
    match written {
        Ok(()) => {}
        Err(_) if refused.is_some() => {}
        Err(e) => return Err(Failure::Fatal(e)),
    }

    Ok(Programmed {
        details_lba,
        remounts,
        refused,
    })
}

/// Wait for this drop's remount, then return what `DETAILS.TXT` reports, unjudged.
fn settle_status(
    selector: &DebugProbeSelector,
    bulk: &mut MscBulk,
    dropped: &Programmed,
) -> Result<String, Failure> {
    let deadline = Instant::now() + REMOUNT_TIMEOUT;
    crate::wait::timed("msc.remount", REMOUNT_TIMEOUT, || {
        let read = || {
            let body = bulk.read_sector(dropped.details_lba);
            // A detached medium still answers TEST UNIT READY; only a device gone needs reopening.
            if body.is_err() && bulk.test_unit_ready().is_err() {
                if let Ok(Some(fresh)) = open_msc(selector) {
                    *bulk = fresh;
                }
            }
            body
        };
        let status = status_after_remount(read, dropped.remounts, deadline)?;
        // Takes the UNIT ATTENTION, so the volume is left ready.
        wait_ready_after_remount(selector, bulk, deadline).map_err(Failure::Fatal)?;
        Ok(status)
    })
}

/// Program `image` and read the device's report on it.
fn drive(
    selector: &DebugProbeSelector,
    bulk: &mut MscBulk,
    image: &[u8],
) -> Result<String, Failure> {
    let dropped = program(bulk, image, None)?;
    settle_status(selector, bulk, &dropped)
}

/// Push `--dut-uf2` onto the volume and verify the reflash; skips without an MSC interface.
pub(crate) fn run(cx: &mut Ctx, full_run: bool) -> Result<(), Failure> {
    let uf2_path = cx
        .dut_uf2
        .clone()
        .ok_or_else(|| Failure::Fatal("needs --dut-uf2".into()))?;
    let image =
        std::fs::read(&uf2_path).map_err(|e| Failure::Fatal(format!("read {uf2_path}: {e}")))?;
    let (addr, payload) = first_block(&image)
        .ok_or_else(|| Failure::Fatal(format!("{uf2_path} has no valid uf2 block")))?;

    // Raw bulk claims the USB interface, so the probe-rs session goes first.
    cx.session = None;
    let mut bulk = match open_msc(&cx.selector).map_err(Failure::Fatal)? {
        Some(b) => b,
        None => {
            crate::connect::run(cx, full_run)?;
            return Err(Failure::Skip("msc: firmware".into()));
        }
    };

    let flow = drive(&cx.selector, &mut bulk, &image);
    drop(bulk);

    // Later cases depend on the fresh SWD session, not on this case's success.
    crate::connect::run(cx, full_run)?;
    let status = flow?;
    if !is_done(&status) {
        return Err(Failure::Failed(format!("DETAILS.TXT status: {status}")));
    }
    verify(cx, addr, payload)
}

/// Drop a uf2 with one damaged block: it must be reported, change nothing, and not block a good drop.
pub(crate) fn run_corrupt(cx: &mut Ctx, full_run: bool) -> Result<(), Failure> {
    let uf2_path = cx
        .dut_uf2
        .clone()
        .ok_or_else(|| Failure::Fatal("needs --dut-uf2".into()))?;
    let image =
        std::fs::read(&uf2_path).map_err(|e| Failure::Fatal(format!("read {uf2_path}: {e}")))?;
    let (addr, payload) = first_block(&image)
        .ok_or_else(|| Failure::Fatal(format!("{uf2_path} has no valid uf2 block")))?;
    let payload = payload.to_vec();
    // The whole written range: a rejected drop programs the blocks before the bad one.
    let (span_addr, span_len) = image_span(&image)
        .ok_or_else(|| Failure::Fatal(format!("{uf2_path} spans no readable address range")))?;
    let Some(index) = corrupt_index(&image, &flash_properties(cx)) else {
        return Err(Failure::Skip(
            "msc-corrupt: the uf2 is one block, with none to corrupt but the verified one".into(),
        ));
    };
    let damaged = corrupt_block(&image, index)
        .ok_or_else(|| Failure::Fatal(format!("{uf2_path} is not a whole number of uf2 blocks")))?;

    let before = snapshot(cx, span_addr, span_len)?;
    let tail = erased_tail(cx, &damaged, index, span_addr, span_len);

    // Raw bulk claims the USB interface, so the probe-rs session goes first.
    cx.session = None;
    let mut bulk = match open_msc(&cx.selector).map_err(Failure::Fatal)? {
        Some(b) => b,
        None => {
            crate::connect::run(cx, full_run)?;
            return Err(Failure::Skip("msc-corrupt: firmware".into()));
        }
    };
    let programmed = program(&mut bulk, &damaged, Some(index));

    // Captured, not returned: the good image must go back on whatever happened here.
    let verdict = corrupt_verdict(
        cx, full_run, bulk, programmed, index, span_addr, tail, &before,
    );

    // The good image goes on unconditionally.
    report_both(reflash_good(cx, full_run, &image, addr, &payload), verdict)
}

/// The good drop's outcome and the rejected drop's verdict, neither hiding the other.
fn report_both(good: Result<(), Failure>, rejected: Result<(), Failure>) -> Result<(), Failure> {
    let (good, rejected) = match (good, rejected) {
        (Ok(()), Ok(())) => return Ok(()),
        (Err(only), Ok(())) | (Ok(()), Err(only)) => return Err(only),
        (Err(good), Err(rejected)) => (good, rejected),
    };
    let split = |f| match f {
        Failure::Skip(s) => (0, s),
        Failure::Failed(s) => (1, s),
        Failure::Fatal(s) => (2, s),
    };
    let (good_rank, good) = split(good);
    let (rejected_rank, rejected) = split(rejected);
    let both = format!("{good}; and the rejected drop: {rejected}");
    Err(match good_rank.max(rejected_rank) {
        0 => Failure::Skip(both),
        1 => Failure::Failed(both),
        _ => Failure::Fatal(both),
    })
}

/// Judge the rejected drop: wire hold, device report, and an unchanged image.
#[allow(clippy::too_many_arguments)]
fn corrupt_verdict(
    cx: &mut Ctx,
    full_run: bool,
    mut bulk: MscBulk,
    programmed: Result<Programmed, Failure>,
    index: usize,
    span_addr: u32,
    tail: Range<usize>,
    before: &[u8],
) -> Result<(), Failure> {
    // The wire, not the volume, says the rejected transfer has ended.
    let held = programmed
        .as_ref()
        .map_err(|_| ())
        .and_then(|_| wait_for_wire(&cx.selector).map_err(|_| ()));
    let dropped = programmed?;
    let refused = dropped.refused;
    let held = held.map_err(|()| Failure::Fatal("the probe stopped answering entirely".into()))?;
    crate::wait::note("msc.hold", HOLD_GRACE, held);

    let status = settle_status(&cx.selector, &mut bulk, &dropped);
    drop(bulk);
    crate::connect::run(cx, full_run)?;
    let status = status?;
    let after = forgive_erased_tail(before, snapshot(cx, span_addr, before.len())?, tail);

    match refused {
        Some(r) => println!(
            "msc-corrupt: block {index}, {}, wire back in {:.1}s",
            r.describe(),
            held.as_secs_f32()
        ),
        None => println!(
            "msc-corrupt: block {index} written without refusal, wire back in {:.1}s",
            held.as_secs_f32()
        ),
    }

    judge(index, refused, held, &status, before, &after, span_addr).map_err(Failure::Failed)?;
    cx.last_note = Some(format!("wire back in {:.1}s", held.as_secs_f32()));
    Ok(())
}

/// Span offsets from the refused block to the end of its erase sector; empty if the target names none.
fn erased_tail(
    cx: &Ctx,
    uf2: &[u8],
    index: usize,
    span_addr: u32,
    span_len: usize,
) -> Range<usize> {
    let flash = flash_properties(cx);
    let tail = || {
        let addr = block_addr(uf2, index)?;
        let end = sector_end(&flash, u64::from(addr))?;
        let start = addr.checked_sub(span_addr)? as usize;
        let end = usize::try_from(end - u64::from(span_addr)).ok()?;
        Some(start.min(span_len)..end.min(span_len))
    };
    tail().unwrap_or(0..0)
}

/// The target's flash sector maps, from the session probe-rs attached.
fn flash_properties(cx: &Ctx) -> Vec<&FlashProperties> {
    cx.session
        .iter()
        .flat_map(|s| &s.target().flash_algorithms)
        .map(|a| &a.flash_properties)
        .collect()
}

/// The block `msc-corrupt` damages: past the first sector's pages, so its place never moves with image size.
fn corrupt_index(uf2: &[u8], flash: &[&FlashProperties]) -> Option<usize> {
    let middle = middle_block(uf2)?;
    let aim = || {
        let base = u64::from(block_addr(uf2, 0)?);
        let p = flash.iter().find(|p| p.address_range.contains(&base))?;
        let sector = sector_end(flash, base)? - base;
        let page = u64::from(p.page_size.max(2));
        // Half a sector, at least 8 KiB so a sector-sized page has flushed; never on a page start.
        let mut aim = base + (sector / 2).max(0x2000);
        if aim % page == 0 {
            aim += page / 2;
        }
        let blocks = uf2.len() / UF2_BLOCK;
        (1..blocks - 1).find(|&i| block_addr(uf2, i).is_some_and(|a| u64::from(a) >= aim))
    };
    Some(aim().unwrap_or(middle))
}

/// The end of the erase sector holding `addr`.
fn sector_end(flash: &[&FlashProperties], addr: u64) -> Option<u64> {
    let p = flash.iter().find(|p| p.address_range.contains(&addr))?;
    let rel = addr - p.address_range.start;
    let s = p
        .sectors
        .iter()
        .filter(|s| s.address <= rel && s.size > 0)
        .max_by_key(|s| s.address)?;
    Some(p.address_range.start + s.address + ((rel - s.address) / s.size + 1) * s.size)
}

/// Count erased bytes in `tail` as unchanged: the sector's old contents there cannot be restored.
fn forgive_erased_tail(before: &[u8], mut after: Vec<u8>, tail: Range<usize>) -> Vec<u8> {
    for i in tail {
        if after.get(i) == Some(&0xFF) {
            after[i] = before[i];
        }
    }
    after
}

/// What the rejected drop must have done: released the wire, reported failure, changed nothing.
fn judge(
    index: usize,
    refused: Option<Refusal>,
    held: Duration,
    status: &str,
    before: &[u8],
    after: &[u8],
    span_addr: u32,
) -> Result<(), String> {
    let write = match refused {
        Some(r) => r.describe(),
        None => "no write was refused".to_string(),
    };
    if held > HOLD_GRACE {
        return Err(format!(
            "the probe refused DAP_SWD_Sequence for {:.1}s after a drop whose block {index} carries \
             no uf2 magic (grace {:.1}s); a rejected block must release the wire, not hold it \
             for the transfer's idle timeout. {write}. DETAILS.TXT status: `{status}`",
            held.as_secs_f32(),
            HOLD_GRACE.as_secs_f32()
        ));
    }
    if is_done(status) {
        return Err(format!(
            "DETAILS.TXT reports `{status}` for an image whose block {index} carries no uf2 \
             magic: a corrupt drop must be reported as a failure. {write}"
        ));
    }
    if let Some(off) = before.iter().zip(after).position(|(a, b)| a != b) {
        let changed = before.iter().zip(after).filter(|(a, b)| a != b).count();
        return Err(format!(
            "the target image changed under a rejected drop: {changed} of {} bytes differ, the \
             first at {:#010x}. A drop the device rejects must leave the target as it found it, \
             but for the erased tail of the bad block's sector. {write}. DETAILS.TXT status: \
             `{status}`",
            before.len(),
            span_addr as u64 + off as u64
        ));
    }
    Ok(())
}

/// Put the good image back and leave the DUT running it.
fn reflash_good(
    cx: &mut Ctx,
    full_run: bool,
    image: &[u8],
    addr: u32,
    payload: &[u8],
) -> Result<(), Failure> {
    cx.session = None;
    let mut bulk = open_msc(&cx.selector)
        .map_err(Failure::Fatal)?
        .ok_or_else(|| Failure::Fatal("the mass-storage interface went away".into()))?;
    // Wait out the rejected drop's remount first, so the next data-in is not cancelled.
    crate::wait::timed("msc.remount", REMOUNT_TIMEOUT, || {
        wait_ready_after_remount(&cx.selector, &mut bulk, Instant::now() + REMOUNT_TIMEOUT)
    })
    .map_err(Failure::Fatal)?;
    let flow = drive(&cx.selector, &mut bulk, image);
    drop(bulk);
    crate::connect::run(cx, full_run)?;
    let status = flow?;
    if !is_done(&status) {
        return Err(Failure::Failed(format!(
            "the good image after the rejected one reports `{status}`: the rejected drop left \
             the volume unable to take a working image"
        )));
    }
    verify(cx, addr, payload)?;
    reset_dut(cx)
}

/// Reset the DUT and wait for its boot event.
fn reset_dut(cx: &mut Ctx) -> Result<(), Failure> {
    let dut_elf = cx.dut_elf.clone();
    let session = cx
        .session
        .as_mut()
        .ok_or_else(|| Failure::Fatal("msc-corrupt lost the session before the reset".into()))?;
    let mut core = session
        .core(0)
        .map_err(|e| Failure::Fatal(crate::util::chain(e)))?;
    crate::reset(&mut core).map_err(|e| Failure::Failed(format!("reset after reflash: {e}")))?;
    let mut rtt = crate::rtt::attach(&mut core, dut_elf.as_deref()).map_err(Failure::Failed)?;
    crate::rtt::expect(&mut rtt, &mut core, daplink_test_protocol::EVT_BOOT)
        .map_err(|e| Failure::Failed(format!("no boot after the reflash: {e}")))?;
    Ok(())
}

/// Poll an idle-cycle `DAP_SWD_Sequence` until the probe stops refusing it; a refusal is the wire still held.
fn wait_for_wire(selector: &DebugProbeSelector) -> Result<Duration, String> {
    let start = Instant::now();
    let deadline = start + WIRE_RELEASE_TIMEOUT;
    loop {
        match crate::dap::wire_held(selector) {
            Ok(false) => return Ok(start.elapsed()),
            Ok(true) => {}
            // The remount detaches the device, so a transport error here is expected too.
            Err(_) => {}
        }
        if Instant::now() >= deadline {
            return Err(format!(
                "the probe still refused DAP_SWD_Sequence {}s after the drop",
                WIRE_RELEASE_TIMEOUT.as_secs()
            ));
        }
        std::thread::sleep(POLL_INTERVAL);
    }
}

/// Read `len` bytes of the target's image at `addr` through the live session.
fn snapshot(cx: &mut Ctx, addr: u32, len: usize) -> Result<Vec<u8>, Failure> {
    let session = cx
        .session
        .as_mut()
        .ok_or_else(|| Failure::Fatal("msc-corrupt needs a successful connect".into()))?;
    let mut core = session
        .core(0)
        .map_err(|e| Failure::Fatal(crate::util::chain(e)))?;
    let mut buf = vec![0u8; len];
    core.read_8(u64::from(addr), &mut buf).map_err(|e| {
        Failure::Fatal(format!(
            "readback at {addr:#010x}: {}",
            crate::util::chain(e)
        ))
    })?;
    Ok(buf)
}

#[cfg(test)]
mod tests {
    use super::is_done;
    use std::time::{Duration, Instant};

    /// `DETAILS.TXT` as the volume renders it.
    fn details(remounts: u32, status: &str) -> Result<[u8; 512], String> {
        let text = format!("Remount count: {remounts:08}\r\nStatus: {status:<64}\r\nFault: \r\n");
        let mut body = [b' '; 512];
        body[..text.len()].copy_from_slice(text.as_bytes());
        Ok(body)
    }

    /// Drive the wait over scripted reads, the last repeating, with `before` counted ahead of the drop.
    fn wait_over(
        script: Vec<Result<[u8; 512], String>>,
        before: u32,
        budget: Duration,
    ) -> Result<String, super::Failure> {
        let mut reads = script.into_iter();
        let mut last = Err("nothing scripted".to_string());
        super::status_after_remount(
            || {
                last = reads.next().unwrap_or_else(|| last.clone());
                last.clone()
            },
            before,
            Instant::now() + budget,
        )
    }

    fn message(f: super::Failure) -> String {
        match f {
            super::Failure::Failed(s) => format!("failed: {s}"),
            super::Failure::Skip(s) => format!("skip: {s}"),
            super::Failure::Fatal(s) => format!("fatal: {s}"),
        }
    }

    const STALE: &str = "Flash: 8704 bytes -> DecodeError [STM32F401CE]";
    const FRESH: &str = "Flash: 8704 bytes -> Done [STM32F401CE]";
    const BUDGET: Duration = Duration::from_secs(5);

    fn no_medium() -> Result<[u8; 512], String> {
        Err("READ(10) lba=135 count=1 CSW status 1".to_string())
    }

    /// A clean drop keeps the medium ready through its settle, showing the last report the whole time.
    #[test]
    fn a_settling_volume_is_not_read_as_this_drops_report() {
        let script = vec![
            details(2, STALE),
            details(2, STALE),
            no_medium(),
            no_medium(),
            details(3, FRESH),
        ];
        match wait_over(script, 2, BUDGET) {
            Ok(status) => assert_eq!(status, FRESH),
            Err(f) => panic!("{}", message(f)),
        }
    }

    /// The first drop since boot: the settling volume's `Status:` is still blank.
    #[test]
    fn a_blank_status_before_the_first_remount_is_not_a_report() {
        let script = vec![details(0, ""), no_medium(), details(1, FRESH)];
        match wait_over(script, 0, BUDGET) {
            Ok(status) => assert_eq!(status, FRESH),
            Err(f) => panic!("{}", message(f)),
        }
    }

    /// A failed drop, or a polled teardown past the settle, detaches before the first read.
    #[test]
    fn a_volume_that_detached_at_once_is_still_waited_for() {
        let script = vec![no_medium(), no_medium(), details(3, STALE)];
        match wait_over(script, 2, BUDGET) {
            Ok(status) => assert_eq!(status, STALE),
            Err(f) => panic!("{}", message(f)),
        }
    }

    /// A remount that finished before the first read has nothing left to wait for.
    #[test]
    fn a_remount_already_counted_returns_on_the_first_read() {
        let started = Instant::now();
        match wait_over(vec![details(3, FRESH)], 2, BUDGET) {
            Ok(status) => assert_eq!(status, FRESH),
            Err(f) => panic!("{}", message(f)),
        }
        assert!(started.elapsed() < super::POLL_INTERVAL);
    }

    /// The queued transport can serve a read from a medium that just left: zeros, not a report.
    #[test]
    fn a_blanked_sector_is_not_a_report() {
        let script = vec![Ok([0u8; 512]), details(3, FRESH)];
        match wait_over(script, 2, BUDGET) {
            Ok(status) => assert_eq!(status, FRESH),
            Err(f) => panic!("{}", message(f)),
        }
    }

    /// A rebooted probe restarts its count and seeds the old status, so a lower count is not a remount.
    #[test]
    fn a_remount_that_never_comes_is_a_bounded_error_naming_the_count() {
        let budget = Duration::from_millis(250);
        let started = Instant::now();
        let f = wait_over(vec![details(0, STALE)], 2, budget).expect_err("no remount was counted");
        assert!(started.elapsed() < budget * 4);
        let text = message(f);
        assert!(text.starts_with("fatal:"), "{text}");
        assert!(text.contains("past 2"), "{text}");
        assert!(text.contains(STALE), "{text}");
    }

    /// A counted remount whose body has no `Status:` is the device's own defect.
    #[test]
    fn a_counted_remount_without_a_status_line_is_a_failure() {
        let mut body = [b' '; 512];
        let text = b"Remount count: 00000003\r\n";
        body[..text.len()].copy_from_slice(text);
        let f = wait_over(vec![Ok(body)], 2, BUDGET).expect_err("nothing to report");
        assert!(message(f).starts_with("failed:"));
    }

    /// Neither drop's finding hides the other, and the graver kind decides what the run does.
    #[test]
    fn both_drops_are_reported() {
        use super::{report_both, Failure};
        assert!(report_both(Ok(()), Ok(())).is_ok());
        let only_good = report_both(Err(Failure::Failed("good".into())), Ok(()));
        assert_eq!(message(only_good.unwrap_err()), "failed: good");
        let only_rejected = report_both(Ok(()), Err(Failure::Fatal("rejected".into())));
        assert_eq!(message(only_rejected.unwrap_err()), "fatal: rejected");

        let both = report_both(
            Err(Failure::Failed("good".into())),
            Err(Failure::Failed("rejected".into())),
        );
        let text = message(both.unwrap_err());
        assert!(text.starts_with("failed:"), "{text}");
        assert!(text.contains("good") && text.contains("rejected"), "{text}");

        let graver = report_both(
            Err(Failure::Failed("good".into())),
            Err(Failure::Fatal("rejected".into())),
        );
        let text = message(graver.unwrap_err());
        assert!(text.starts_with("fatal:"), "{text}");
        assert!(text.contains("good") && text.contains("rejected"), "{text}");
    }

    /// A write the device refused with a CSW failure, the ordinary shape.
    fn refused_at(lba: u32) -> super::Refusal {
        super::Refusal {
            lba,
            detached: false,
        }
    }

    /// Only the device's own clean report passes.
    #[test]
    fn the_wire_wait_outlasts_the_grace_it_judges() {
        assert!(super::WIRE_RELEASE_TIMEOUT > super::HOLD_GRACE * 4);
        assert!(super::WIRE_RELEASE_TIMEOUT.as_secs() >= 30);
    }

    /// A rejected drop is judged over the whole range; the first block alone never differs.
    #[test]
    fn a_half_programmed_image_is_a_change_wherever_it_lands() {
        let before = vec![0xFFu8; 8];
        let mut after = before.clone();
        after[5] = 0x00;
        let err = super::judge(
            63,
            Some(refused_at(206)),
            Duration::from_millis(1),
            "Flash: image.uf2 -> Error",
            &before,
            &after,
            0x0800_0000,
        )
        .expect_err("a changed byte is a failure");
        assert!(err.contains("1 of 8 bytes differ"), "{err}");
        assert!(err.contains("0x08000005"), "{err}");
    }

    /// The findings are reported in order, each naming what the device said.
    #[test]
    fn every_verdict_carries_the_device_status() {
        let same = vec![0u8; 4];
        let status = "Flash: image.uf2 -> Error: bad magic";
        // A hold past the grace outranks everything else.
        let held = super::judge(
            63,
            Some(refused_at(206)),
            Duration::from_secs(20),
            status,
            &same,
            &same,
            0,
        );
        assert!(held.as_ref().unwrap_err().contains("20.0s"), "{held:?}");
        assert!(held.unwrap_err().contains(status));
        // A clean report for a corrupt image is the next finding.
        let done = super::judge(
            63,
            Some(refused_at(206)),
            Duration::from_millis(1),
            "Flash: image.uf2 -> Done",
            &same,
            &same,
            0,
        );
        assert!(done.unwrap_err().contains("-> Done"));
        // A refusal names the LBA that refused; an unrefused drop says so instead.
        let named = super::judge(
            63,
            Some(refused_at(206)),
            Duration::from_secs(20),
            status,
            &same,
            &same,
            0,
        );
        assert!(named
            .unwrap_err()
            .contains("the write at lba 206 was refused"));
        // A drop that ends with the device going away says so instead.
        let gone = super::judge(
            63,
            Some(super::Refusal {
                lba: 206,
                detached: true,
            }),
            Duration::from_secs(20),
            status,
            &same,
            &same,
            0,
        );
        assert!(gone
            .unwrap_err()
            .contains("ended with the device detaching"));
        let quiet = super::judge(63, None, Duration::from_secs(20), status, &same, &same, 0);
        assert!(quiet.unwrap_err().contains("no write was refused"));
        assert!(super::judge(
            63,
            Some(refused_at(206)),
            Duration::from_millis(1),
            status,
            &same,
            &same,
            0
        )
        .is_ok());
    }

    /// The F401's sector map: four 16 KiB sectors, one 64 KiB, then 128 KiB.
    fn f401() -> probe_rs::config::FlashProperties {
        let sector = |address, size| probe_rs::config::SectorDescription { address, size };
        probe_rs::config::FlashProperties {
            address_range: 0x0800_0000..0x0808_0000,
            page_size: 0x400,
            erased_byte_value: 0xFF,
            program_page_timeout: 0,
            erase_sector_timeout: 0,
            sectors: vec![
                sector(0, 0x4000),
                sector(0x1_0000, 0x1_0000),
                sector(0x2_0000, 0x2_0000),
            ],
        }
    }

    /// A uf2 of `blocks` 256-byte-payload blocks from `base`.
    fn fake_uf2(blocks: u32, base: u32) -> Vec<u8> {
        let mut out = vec![0u8; blocks as usize * 512];
        for (i, b) in out.chunks_mut(512).enumerate() {
            b[12..16].copy_from_slice(&(base + i as u32 * 256).to_le_bytes());
        }
        out
    }

    fn rp2040() -> probe_rs::config::FlashProperties {
        probe_rs::config::FlashProperties {
            address_range: 0x1000_0000..0x1800_0000,
            page_size: 0x1000,
            erased_byte_value: 0xFF,
            program_page_timeout: 0,
            erase_sector_timeout: 0,
            sectors: vec![probe_rs::config::SectorDescription {
                address: 0,
                size: 0x1000,
            }],
        }
    }

    /// The damaged block sits mid-sector and mid-page on every image size; the middle block otherwise.
    #[test]
    fn corrupt_index_does_not_drift_with_image_size() {
        let f = f401();
        for blocks in [125u32, 129, 132, 139, 800, 2000] {
            let img = fake_uf2(blocks, 0x0800_0000);
            let i = super::corrupt_index(&img, &[&f]).unwrap();
            let a = crate::uf2::block_addr(&img, i).unwrap();
            assert_eq!(a, 0x0800_2200, "{blocks} blocks");
            assert_ne!(a % 0x4000, 0);
            assert_ne!(a % 0x400, 0);
        }
        let r = rp2040();
        for blocks in [112u32, 113, 800] {
            let img = fake_uf2(blocks, 0x1000_0000);
            let i = super::corrupt_index(&img, &[&r]).unwrap();
            assert_eq!(crate::uf2::block_addr(&img, i), Some(0x1000_2800));
        }
        assert_eq!(
            super::corrupt_index(&fake_uf2(8, 0x0800_0000), &[&f]),
            Some(4)
        );
        assert_eq!(
            super::corrupt_index(&fake_uf2(132, 0x0800_0000), &[]),
            Some(66)
        );
        assert_eq!(super::corrupt_index(&fake_uf2(1, 0x0800_0000), &[&f]), None);
    }

    #[test]
    fn sector_end_follows_a_non_uniform_map() {
        let f = f401();
        let end = |addr| super::sector_end(&[&f], addr);
        assert_eq!(end(0x0800_3e00), Some(0x0800_4000));
        assert_eq!(end(0x0800_4000), Some(0x0800_8000));
        assert_eq!(end(0x0801_2345), Some(0x0802_0000));
        assert_eq!(end(0x0803_0000), Some(0x0804_0000));
        assert_eq!(end(0x0900_0000), None);
    }

    /// Only erased bytes in the refused block's sector tail are forgiven; anything else still differs.
    #[test]
    fn only_the_erased_sector_tail_is_forgiven() {
        let before = vec![0x11u8; 8];
        let mut after = before.clone();
        after[5] = 0xFF;
        after[6] = 0xFF;
        assert_eq!(
            super::forgive_erased_tail(&before, after.clone(), 4..8),
            before
        );
        after[2] = 0xFF;
        after[7] = 0x00;
        let left = super::forgive_erased_tail(&before, after, 4..8);
        assert_eq!(left, [0x11, 0x11, 0xFF, 0x11, 0x11, 0x11, 0x11, 0x00]);
    }

    #[test]
    fn only_a_clean_flash_report_is_done() {
        assert!(is_done("Flash: image.uf2 -> Done"));
        assert!(!is_done("Flash: image.uf2 -> Error: invalid uf2 magic"));
        assert!(!is_done("Idle"));
        assert!(!is_done(""));
    }
}
