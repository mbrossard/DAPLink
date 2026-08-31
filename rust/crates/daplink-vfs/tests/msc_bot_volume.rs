//! Drag-n-drop driven through the Bulk-Only transport into the real volume.
//!
//! The transport's own tests answer against a mock block device, and the volume's tests call
//! `write_sector` directly; nothing joins the two. These drive `run_command` over [`DirectIo`]
//! and a live [`Vfs`], so a Command Block Wrapper reaches the flash backend the way a host's
//! does: CBW, 64-byte OUT packets, data-phase bracket, CSW.

use core::future::Future;
use core::pin::pin;
use core::task::{Context, Poll, Waker};

use daplink_core::msc_proto::{
    parse_cbw, CBW_LEN, CBW_SIGNATURE, CSW_FAILED, CSW_LEN, CSW_PASSED, CSW_SIGNATURE, SENSE_NONE,
};
use daplink_core::{FlashBackend, FlashPhase, MSC_SECTOR_SIZE};
use daplink_targets::{targets::rp2040::RP2040, Target};
use daplink_vfs::{
    fat, run_command, BulkIn, BulkOut, DetailsFile, DeviceInfo, DirectIo, FlashSink, MediaState,
    NoObserver, TransferOutcome, VfsConfig, VfsFile,
};

type TestVfs<'a> = daplink_vfs::Vfs<'a, FlashSink<RecordingBackend>, NoObserver>;

/// A flash backend recording every programmed fragment, over the RP2040 record's flash window.
struct RecordingBackend {
    ranges: [(u32, u32); 1],
    writes: Vec<(u32, Vec<u8>)>,
    finished: Option<bool>,
}

impl RecordingBackend {
    fn new() -> Self {
        let flash = RP2040
            .flash_regions()
            .iter()
            .find(|r| r.is_default)
            .expect("the RP2040 record has a default flash region");
        Self {
            ranges: [(flash.start, flash.end)],
            writes: Vec::new(),
            finished: None,
        }
    }

    fn programmed_from(&self, base: u32) -> Vec<u8> {
        let mut out = Vec::new();
        for (addr, data) in &self.writes {
            if *addr >= base {
                out.extend_from_slice(data);
            }
        }
        out
    }
}

impl FlashBackend for RecordingBackend {
    fn begin(&mut self) -> Result<(), FlashPhase> {
        Ok(())
    }
    fn program(&mut self, addr: u32, data: &[u8]) -> Result<(), FlashPhase> {
        self.writes.push((addr, data.to_vec()));
        Ok(())
    }
    fn finish(&mut self, success: bool) -> Result<(), FlashPhase> {
        self.finished = Some(success);
        Ok(())
    }
    fn finish_with_reset(&mut self, success: bool, _reset: bool) -> Result<(), FlashPhase> {
        self.finish(success)
    }
    fn flash_ranges(&self) -> &[(u32, u32)] {
        &self.ranges
    }
    fn selected_target_name(&self) -> Option<&str> {
        Some("RP2040")
    }
}

fn block_on<F: Future>(f: F) -> F::Output {
    let mut cx = Context::from_waker(Waker::noop());
    let mut f = pin!(f);
    loop {
        if let Poll::Ready(v) = f.as_mut().poll(&mut cx) {
            return v;
        }
    }
}

/// Collects the device-to-host stream.
struct HostIn(Vec<u8>);

impl BulkIn for HostIn {
    async fn write(&mut self, data: &[u8]) -> Result<(), ()> {
        self.0.extend_from_slice(data);
        Ok(())
    }
}

/// Hands the host-to-device payload over in 64-byte packets, as a full-speed bulk pipe would.
struct HostOut {
    data: Vec<u8>,
    at: usize,
}

impl BulkOut for HostOut {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, ()> {
        if self.at >= self.data.len() {
            return Err(());
        }
        let n = core::cmp::min(core::cmp::min(64, buf.len()), self.data.len() - self.at);
        buf[..n].copy_from_slice(&self.data[self.at..self.at + n]);
        self.at += n;
        Ok(n)
    }
}

fn cbw(tag: u32, data_len: u32, dir_in: bool, cb: &[u8]) -> [u8; CBW_LEN] {
    let mut pkt = [0u8; CBW_LEN];
    pkt[0..4].copy_from_slice(&CBW_SIGNATURE.to_le_bytes());
    pkt[4..8].copy_from_slice(&tag.to_le_bytes());
    pkt[8..12].copy_from_slice(&data_len.to_le_bytes());
    pkt[12] = if dir_in { 0x80 } else { 0x00 };
    pkt[14] = cb.len() as u8;
    pkt[15..15 + cb.len()].copy_from_slice(cb);
    pkt
}

fn rw10(op: u8, lba: u32, count: u16) -> [u8; 10] {
    let l = lba.to_be_bytes();
    let c = count.to_be_bytes();
    [op, 0x00, l[0], l[1], l[2], l[3], 0x00, c[0], c[1], 0x00]
}

/// One host command against the live volume: returns the data-phase bytes and the CSW triple,
/// plus how many host-to-device bytes the transport actually consumed.
fn run_cmd(
    vfs: &mut TestVfs<'_>,
    cbw: &[u8; CBW_LEN],
    payload: &[u8],
) -> (Vec<u8>, (u32, u32, u8), usize) {
    let parsed = parse_cbw(cbw).expect("a well-formed CBW");
    let mut ep_in = HostIn(Vec::new());
    let mut ep_out = HostOut {
        data: payload.to_vec(),
        at: 0,
    };
    let mut sector = [0u8; MSC_SECTOR_SIZE];
    let mut sense = SENSE_NONE;
    let mut ua = false;
    {
        let mut io = DirectIo::new(vfs);
        block_on(run_command::<_, _, _, 64>(
            &mut ep_in,
            &mut ep_out,
            &mut io,
            &mut sector,
            &mut sense,
            &mut ua,
            &parsed,
        ));
    }
    let n = ep_in.0.len();
    let csw = &ep_in.0[n - CSW_LEN..];
    assert_eq!(
        u32::from_le_bytes([csw[0], csw[1], csw[2], csw[3]]),
        CSW_SIGNATURE
    );
    let tag = u32::from_le_bytes([csw[4], csw[5], csw[6], csw[7]]);
    let residue = u32::from_le_bytes([csw[8], csw[9], csw[10], csw[11]]);
    (
        ep_in.0[..n - CSW_LEN].to_vec(),
        (tag, residue, csw[12]),
        ep_out.at,
    )
}

fn dir_entry(name: &[u8; 11], attr: u8, first_cluster: u16, size: u32) -> [u8; 32] {
    let mut e = [0u8; 32];
    e[0..11].copy_from_slice(name);
    e[11] = attr;
    e[26..28].copy_from_slice(&first_cluster.to_le_bytes());
    e[28..32].copy_from_slice(&size.to_le_bytes());
    e
}

const XIP_BASE: u32 = 0x1000_0000;
const USER_CLUSTER: u16 = 10;

fn user_lba(sector: u32) -> u32 {
    fat::cluster_to_lba(USER_CLUSTER as u32) + sector
}

fn uf2_block(block_no: u32, num_blocks: u32, payload: &[u8]) -> [u8; 512] {
    let mut b = [0u8; 512];
    b[0..4].copy_from_slice(&0x0A32_4655u32.to_le_bytes());
    b[4..8].copy_from_slice(&0x9E5D_5157u32.to_le_bytes());
    b[12..16].copy_from_slice(&(XIP_BASE + block_no * 256).to_le_bytes());
    b[16..20].copy_from_slice(&(payload.len() as u32).to_le_bytes());
    b[20..24].copy_from_slice(&block_no.to_le_bytes());
    b[24..28].copy_from_slice(&num_blocks.to_le_bytes());
    b[32..32 + payload.len()].copy_from_slice(payload);
    b[508..512].copy_from_slice(&0x0AB1_6F30u32.to_le_bytes());
    b
}

/// The root-directory sector carrying one entry for the dropped image.
fn root_with_entry(size: u32) -> [u8; 512] {
    let mut root = [0u8; 512];
    root[0..32].copy_from_slice(&dir_entry(b"APP     UF2", 0x20, USER_CLUSTER, size));
    root
}

macro_rules! with_vfs {
    ($vfs:ident, $body:block) => {{
        let mut details = DetailsFile::new(DeviceInfo {
            version: "0.1.0",
            serial: "SER123",
            reset_count: 0,
        });
        let mut files: [&mut dyn VfsFile; 1] = [&mut details];
        let mut $vfs: TestVfs<'_> = daplink_vfs::Vfs::new(
            VfsConfig::default(),
            &mut files[..],
            FlashSink::new(RecordingBackend::new()),
            NoObserver,
        );
        $body
    }};
}

#[test]
fn a_drop_delivered_as_write10_commands_programs_the_image() {
    with_vfs!(vfs, {
        let p0 = [0xA1u8; 256];
        let p1 = [0xB2u8; 256];
        let mut image = Vec::new();
        image.extend_from_slice(&uf2_block(0, 2, &p0));
        image.extend_from_slice(&uf2_block(1, 2, &p1));

        let dirent = run_cmd(
            &mut vfs,
            &cbw(1, 512, false, &rw10(0x2A, 131, 1)),
            &root_with_entry(1024),
        );
        assert_eq!(dirent.1, (1, 0, CSW_PASSED), "the dirent command");
        assert_eq!(dirent.2, 512, "the whole dirent sector is consumed");

        let data = run_cmd(
            &mut vfs,
            &cbw(2, 1024, false, &rw10(0x2A, user_lba(0), 2)),
            &image,
        );
        assert_eq!(data.1, (2, 0, CSW_PASSED), "the two-sector image command");
        assert_eq!(data.2, 1024, "both sectors are consumed");

        // The medium may not change under an open command: only the tick after it detaches.
        assert_eq!(vfs.media(), MediaState::Ready);
        assert_eq!(vfs.tick(1), MediaState::Detached);

        let mut expected = Vec::new();
        expected.extend_from_slice(&p0);
        expected.extend_from_slice(&p1);
        assert_eq!(vfs.sink().outcome(), Some(TransferOutcome::Done));
        assert_eq!(vfs.sink().backend().programmed_from(XIP_BASE), expected);
        assert_eq!(vfs.sink().backend().finished, Some(true));
    });
}

#[test]
fn read10_returns_what_the_volume_itself_renders() {
    with_vfs!(vfs, {
        let (data, csw, consumed) = run_cmd(&mut vfs, &cbw(3, 1024, true, &rw10(0x28, 0, 2)), &[]);
        assert_eq!(csw, (3, 0, CSW_PASSED));
        assert_eq!(consumed, 0, "a read consumes no host packets");
        assert_eq!(data.len(), 1024);

        let mut want = Vec::new();
        for lba in 0..2 {
            let mut buf = [0u8; 512];
            vfs.read_sector(lba, &mut buf);
            want.extend_from_slice(&buf);
        }
        assert_eq!(data, want, "the transport streams the volume verbatim");
        assert_eq!((data[510], data[511]), (0x55, 0xAA), "boot signature");
    });
}

#[test]
fn a_write10_length_disagreement_never_reaches_the_flash_backend() {
    with_vfs!(vfs, {
        let p0 = [0xA1u8; 256];
        let dirent = run_cmd(
            &mut vfs,
            &cbw(4, 512, false, &rw10(0x2A, 131, 1)),
            &root_with_entry(512),
        );
        assert_eq!(dirent.1, (4, 0, CSW_PASSED));

        // Two sectors declared as one: accepting the phase would program a half image.
        let image = uf2_block(0, 1, &p0);
        let (data, csw, consumed) = run_cmd(
            &mut vfs,
            &cbw(5, 512, false, &rw10(0x2A, user_lba(0), 2)),
            &image,
        );
        assert!(data.is_empty(), "a rejected command has no data phase");
        assert_eq!(csw, (5, 512, CSW_FAILED));
        assert_eq!(consumed, 0, "no host packet is consumed");

        assert!(!vfs.is_transfer_active());
        assert!(vfs.sink().backend().writes.is_empty());
        assert_eq!(vfs.sink().backend().finished, None);
    });
}
