//! The msc case: push `--dut-uf2` onto the probe's volume over raw SCSI/BOT and verify it.

use std::time::{Duration, Instant};

use nusb::transfer::{Buffer, Bulk, Direction, In, Out};
use nusb::{Endpoint, MaybeFuture};
use probe_rs::probe::DebugProbeSelector;

use crate::msc_wire::{
    self, BootLayout, SectorIo, CSW_LEN, CSW_PASSED, DETAILS_NAME, IMAGE_NAME, OP_READ_10,
    OP_WRITE_10, SECTOR_SIZE,
};
use crate::uf2::{first_block, verify};
use crate::{Ctx, Failure};

const TIMEOUT: Duration = Duration::from_secs(5);
const OP_INQUIRY: u8 = 0x12;
const OP_TEST_UNIT_READY: u8 = 0x00;
const OP_READ_CAPACITY_10: u8 = 0x25;
/// How long a fresh mount is given to answer TEST UNIT READY before the case gives up.
const READY_TIMEOUT: Duration = Duration::from_secs(5);
/// How long the post-write remount (UNIT ATTENTION latch or a full D+ bounce) is given.
const REMOUNT_TIMEOUT: Duration = Duration::from_secs(15);
const POLL_INTERVAL: Duration = Duration::from_millis(100);

/// The probe's claimed MSC/BOT interface and its bulk endpoint pair.
struct MscBulk {
    _interface: nusb::Interface,
    out: Endpoint<Bulk, Out>,
    inp: Endpoint<Bulk, In>,
    tag: u32,
}

impl MscBulk {
    fn next_tag(&mut self) -> u32 {
        self.tag = self.tag.wrapping_add(1);
        self.tag
    }

    /// One CBW/data-phase/CSW exchange, returning the bytes read back and the CSW status.
    fn command(
        &mut self,
        cb: &[u8],
        dir_in: bool,
        data_len: u32,
        out_payload: Option<&[u8]>,
    ) -> Result<(Vec<u8>, u8), String> {
        let tag = self.next_tag();
        let cbw = msc_wire::build_cbw(tag, data_len, dir_in, cb);
        self.out
            .transfer_blocking(Buffer::from(cbw.to_vec()), TIMEOUT)
            .into_result()
            .map_err(|e| format!("CBW write failed: {e}"))?;
        let mut in_data = Vec::new();
        if data_len > 0 {
            if dir_in {
                // An IN request must be a whole number of packets; a short packet ends it.
                let want = (data_len as usize).next_multiple_of(self.inp.max_packet_size());
                in_data = self
                    .inp
                    .transfer_blocking(Buffer::new(want), TIMEOUT)
                    .into_result()
                    .map_err(|e| format!("data-in phase failed: {e}"))?
                    .into_vec();
            } else {
                let payload = out_payload.ok_or("write command missing its OUT payload")?;
                self.out
                    .transfer_blocking(Buffer::from(payload.to_vec()), TIMEOUT)
                    .into_result()
                    .map_err(|e| format!("data-out phase failed: {e}"))?;
            }
        }
        let want = CSW_LEN.next_multiple_of(self.inp.max_packet_size());
        let csw = self
            .inp
            .transfer_blocking(Buffer::new(want), TIMEOUT)
            .into_result()
            .map_err(|e| format!("CSW read failed: {e}"))?
            .into_vec();
        let (status, _residue) = msc_wire::parse_csw(&csw, tag)?;
        Ok((in_data, status))
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

    fn write10(&mut self, lba: u32, count: u16, data: &[u8]) -> Result<(), String> {
        let cb = msc_wire::rw10_cb(OP_WRITE_10, lba, count);
        let data_len = count as u32 * SECTOR_SIZE as u32;
        if data.len() != data_len as usize {
            return Err(format!(
                "WRITE(10) payload is {} bytes, want {data_len}",
                data.len()
            ));
        }
        let (_data, status) = self.command(&cb, false, data_len, Some(data))?;
        if status != CSW_PASSED {
            return Err(format!(
                "WRITE(10) lba={lba} count={count} CSW status {status}"
            ));
        }
        Ok(())
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
        self.write10(lba, 1, data)
    }
}

/// Open the probe's MSC/BOT interface: `Ok(None)` when no interface carries the mass-storage
/// class triple, which is the capability probe, and `Err` only for an actual USB failure.
fn open_msc(selector: &DebugProbeSelector) -> Result<Option<MscBulk>, String> {
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

/// Tolerate both post-transfer remount shapes: a UNIT ATTENTION latch answered over the same
/// handle, or a D+ bounce that re-enumerates and makes the open handle error until reopened.
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

/// Re-read `DETAILS.TXT` after the remount and require its `Status:` line to name a clean
/// `Flash: ... -> Done`, which is the device's own report on the drop.
fn check_status(
    selector: &DebugProbeSelector,
    bulk: &mut MscBulk,
    layout: &BootLayout,
    details_cluster: u32,
    deadline: Instant,
) -> Result<(), Failure> {
    let lba = layout.cluster_to_lba(details_cluster);
    let body = loop {
        match bulk.read_sector(lba) {
            Ok(b) => break b,
            Err(e) => {
                if Instant::now() >= deadline {
                    return Err(Failure::Fatal(format!("DETAILS.TXT re-read failed: {e}")));
                }
                if let Ok(Some(fresh)) = open_msc(selector) {
                    *bulk = fresh;
                }
                std::thread::sleep(POLL_INTERVAL);
            }
        }
    };
    let status = msc_wire::status_line(&body)
        .ok_or_else(|| Failure::Failed("DETAILS.TXT carries no Status: line".into()))?;
    if status.starts_with("Flash:") && status.contains("-> Done") {
        Ok(())
    } else {
        Err(Failure::Failed(format!("DETAILS.TXT status: {status}")))
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

/// INQUIRY, TEST UNIT READY, READ CAPACITY, the boot sector and root directory; program `image`
/// as a new file; wait out the remount; check what `DETAILS.TXT` reports.
fn drive(selector: &DebugProbeSelector, bulk: &mut MscBulk, image: &[u8]) -> Result<(), Failure> {
    bulk.inquiry().map_err(Failure::Fatal)?;
    wait_ready(bulk, Instant::now() + READY_TIMEOUT).map_err(Failure::Fatal)?;
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

    msc_wire::program_image(
        bulk,
        &layout,
        first_cluster,
        clusters_needed,
        image,
        dirent_lba,
        &dirent_sector,
    )
    .map_err(Failure::Fatal)?;

    let remount_deadline = Instant::now() + REMOUNT_TIMEOUT;
    wait_ready_after_remount(selector, bulk, remount_deadline).map_err(Failure::Fatal)?;

    check_status(
        selector,
        bulk,
        &layout,
        details.first_cluster,
        remount_deadline,
    )
}

/// Claim the MSC interface, push `--dut-uf2` onto the volume as a new file, and verify the
/// reflash at the target's flash base; skips when the firmware ships no such interface.
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
    flow?;
    verify(cx, addr, payload)
}
