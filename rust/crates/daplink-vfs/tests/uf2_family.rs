//! Real UF2 file shapes on real chip records, on both the drive and the vendor path.

mod common;

use common::{dir_entry, uf2_numbered};
use daplink_core::{FlashBackend, FlashPhase, VendorHandler};
use daplink_targets::targets::{rp2040::RP2040, rp2350::RP2350, stm32f401ce::STM32F401CE};
use daplink_targets::{FixedTarget, ImageHint, Target, TargetSelector};
use daplink_vfs::transfer::TransferOutcome;
use daplink_vfs::{fat, AllDecoders, FlashSink, NoObserver, VendorFlash, VfsConfig, VfsFile};

const F_RP2040: u32 = 0xE48B_FF56;
const F_ABSOLUTE: u32 = 0xE48B_FF57;
const F_RP2350: u32 = 0xE48B_FF59;
const F_STM32F4: u32 = 0x5775_5A57;
const XIP: u32 = 0x1000_0000;
const FLAG_FAMILY: u32 = 0x2000;
const FLAG_EXT: u32 = 0x8000;
type Block = [u8; 512];
type Table = &'static [&'static dyn Target];

/// `HwBackend`'s selection logic over a real table, recording instead of programming.
struct Row {
    table: Table,
    selected: Option<&'static dyn Target>,
    ranges: Vec<(u32, u32)>,
    begins: u32,
    programmed: Vec<u32>,
    finishes: Vec<bool>,
}

impl FlashBackend for Row {
    fn begin(&mut self) -> Result<(), FlashPhase> {
        self.begin_with_hint(None)
    }
    fn begin_with_hint(&mut self, family: Option<u32>) -> Result<(), FlashPhase> {
        self.selected = None;
        let hint = family.map_or(ImageHint::NONE, ImageHint::uf2);
        let t = FixedTarget
            .select(hint, self.table)
            .map_err(|_| FlashPhase::Family)?;
        self.selected = Some(t);
        self.ranges = t.flash_regions().iter().map(|r| (r.start, r.end)).collect();
        self.begins += 1;
        Ok(())
    }
    fn program(&mut self, addr: u32, _data: &[u8]) -> Result<(), FlashPhase> {
        self.programmed.push(addr);
        Ok(())
    }
    fn finish(&mut self, success: bool) -> Result<(), FlashPhase> {
        self.finishes.push(success);
        Ok(())
    }
    fn finish_with_reset(&mut self, success: bool, _reset: bool) -> Result<(), FlashPhase> {
        self.finish(success)
    }
    fn flash_ranges(&self) -> &[(u32, u32)] {
        &self.ranges
    }
    fn expected_uf2_family(&self) -> Option<u32> {
        match self.selected {
            Some(t) => t.uf2_family(),
            None if self.table.len() == 1 => self.table[0].uf2_family(),
            None => None,
        }
    }
    fn requires_uf2_family(&self) -> bool {
        self.table.len() > 1
    }
    fn accepts_uf2_family(&self, family: u32) -> bool {
        self.selected
            .is_none_or(|t| t.uf2_family().is_none() || t.accepts_uf2_family(family))
    }
    fn can_select_uf2_family(&self, family: u32) -> bool {
        self.table.is_empty() || self.table.iter().any(|t| t.accepts_uf2_family(family))
    }
}

fn row(table: Table) -> Row {
    Row {
        table,
        selected: None,
        ranges: Vec::new(),
        begins: 0,
        programmed: Vec::new(),
        finishes: Vec::new(),
    }
}

const ONLY_2040: Table = &[&RP2040];
const ONLY_2350: Table = &[&RP2350];
const FLEET: Table = &[&RP2040, &STM32F401CE, &RP2350];
const NO_RP2350: Table = &[&RP2040, &STM32F401CE];

/// Block `no` of `of`, at its position in flash.
fn page(no: u32, of: u32, family: Option<u32>) -> Block {
    let mut b = uf2_numbered(no, of, XIP + no * 256, &[no as u8; 256]);
    if let Some(f) = family {
        b[8..12].copy_from_slice(&FLAG_FAMILY.to_le_bytes());
        b[28..32].copy_from_slice(&f.to_le_bytes());
    }
    b
}

fn run(family: u32, n: u32) -> Vec<Block> {
    (0..n).map(|i| page(i, n, Some(family))).collect()
}

/// The RP2350-E10 block: picotool 2.1.0 and later tag it, earlier ones only fill it with `0xEF`.
fn e10(tagged: bool) -> Block {
    let mut b = uf2_numbered(0, 2, 0x10FF_FF00, &[0xEF; 256]);
    let flags = FLAG_FAMILY | if tagged { FLAG_EXT } else { 0 };
    b[8..12].copy_from_slice(&flags.to_le_bytes());
    b[28..32].copy_from_slice(&F_ABSOLUTE.to_le_bytes());
    if tagged {
        b[288..292].copy_from_slice(&0x9957_E304u32.to_le_bytes());
    }
    b
}

fn addrs(n: u32) -> Vec<u32> {
    (0..n).map(|i| XIP + i * 256).collect()
}

/// How one drop ended, as the user and the target see it.
#[derive(Debug, PartialEq)]
struct End {
    outcome: Option<TransferOutcome>,
    programmed: Vec<u32>,
    reset: bool,
}

fn end_of(s: &FlashSink<Row>) -> End {
    End {
        outcome: s.outcome(),
        programmed: s.backend().programmed.clone(),
        reset: s.backend().finishes.contains(&true),
    }
}

fn msc(table: Table, blocks: &[Block]) -> End {
    let mut files: [&mut dyn VfsFile; 0] = [];
    let mut vfs: daplink_vfs::Vfs<'_, FlashSink<Row>, NoObserver> = daplink_vfs::Vfs::new(
        VfsConfig::default(),
        &mut files,
        FlashSink::new(row(table)),
        NoObserver,
    );
    let mut root = [0u8; 512];
    root[0..32].copy_from_slice(&dir_entry(
        b"APP     UF2",
        0x20,
        2,
        blocks.len() as u32 * 512,
    ));
    vfs.write_sector(131, &root);
    for (i, b) in blocks.iter().enumerate() {
        vfs.write_sector(fat::DATA_START_LBA + i as u32, b);
    }
    end_of(vfs.sink())
}

fn vendor(table: Table, blocks: &[Block]) -> (End, u8) {
    let mut h: VendorFlash<AllDecoders, FlashSink<Row>> =
        VendorFlash::new(FlashSink::new(row(table)));
    let mut resp = [0u8; 8];
    h.handle(&[0x8A, 2], &mut resp);
    let image: Vec<u8> = blocks.iter().flatten().copied().collect();
    for piece in image.chunks(62) {
        let mut r = vec![0x8C, piece.len() as u8];
        r.extend_from_slice(piece);
        h.handle(&r, &mut resp);
        if resp[1] != 0 {
            break;
        }
    }
    h.handle(&[0x8B], &mut resp);
    (end_of(h.sink()), resp[1])
}

/// Drop `blocks` on both paths; both must end alike.
fn drop_it(table: Table, blocks: &[Block], outcome: TransferOutcome, programmed: Vec<u32>) {
    let want = End {
        outcome: Some(outcome),
        programmed,
        reset: outcome == TransferOutcome::Done,
    };
    assert_eq!(msc(table, blocks), want, "drive path");
    let (got, status) = vendor(table, blocks);
    assert_eq!(got, want, "vendor path");
    assert_eq!(
        status == 0,
        outcome == TransferOutcome::Done,
        "vendor close status {status}"
    );
}

fn cat(parts: &[&[Block]]) -> Vec<Block> {
    parts.concat()
}

const DONE: TransferOutcome = TransferOutcome::Done;
const SHORT: TransferOutcome = TransferOutcome::DecodeError;
const FAMILY: TransferOutcome = TransferOutcome::FlashError(FlashPhase::Family);
const RANGE: TransferOutcome = TransferOutcome::FlashError(FlashPhase::Range);

#[test]
fn e10_vector_matches_the_real_files_header() {
    let b = e10(true);
    let word = |o: usize| u32::from_le_bytes(b[o..o + 4].try_into().unwrap());
    let header: Vec<u32> = (0..8).map(|i| word(i * 4)).collect();
    assert_eq!(
        header,
        [
            0x0A32_4655,
            0x9E5D_5157,
            0xA000,
            0x10FF_FF00,
            256,
            0,
            2,
            0xE48B_FF57
        ]
    );
}

#[test]
fn e10_prefix_tagged_is_disregarded_whole() {
    let image = cat(&[&[e10(true)], &run(F_RP2350, 3)]);
    for table in [ONLY_2350, FLEET] {
        drop_it(table, &image, DONE, addrs(3));
    }
}

#[test]
fn e10_prefix_all_ef_is_disregarded_whole() {
    let image = cat(&[&[e10(false)], &run(F_RP2350, 3)]);
    for table in [ONLY_2350, FLEET] {
        drop_it(table, &image, DONE, addrs(3));
    }
}

#[test]
fn e10_prefix_then_a_cut_image_is_short() {
    let image = cat(&[&[e10(true)], &run(F_RP2350, 4)[..2]]);
    drop_it(ONLY_2350, &image, SHORT, addrs(2));
}

/// A block that merely resembles the E10 one is the image's own, so it is not disregarded.
#[test]
fn an_absolute_image_of_two_pages_is_not_the_e10_block() {
    let mut first = run(F_ABSOLUTE, 2);
    first[0][32..288].fill(0xEF);
    first[0][40] = 0;
    drop_it(ONLY_2350, &first, DONE, addrs(2));
}

#[test]
fn universal_in_cat_order_flashes_each_chip_its_half() {
    let image = cat(&[&run(F_RP2040, 4), &run(F_ABSOLUTE, 3)]);
    drop_it(ONLY_2040, &image, DONE, addrs(4));
    drop_it(ONLY_2350, &image, DONE, addrs(3));
}

#[test]
fn universal_cut_in_the_targets_half_is_short() {
    let cut_first = cat(&[&run(F_RP2040, 4)[..2], &run(F_ABSOLUTE, 3)]);
    drop_it(ONLY_2040, &cut_first, SHORT, addrs(2));
    let cut_second = cat(&[&run(F_RP2040, 4), &run(F_ABSOLUTE, 3)[..2]]);
    drop_it(ONLY_2350, &cut_second, SHORT, addrs(2));
}

/// The published `flash_nuke.uf2` order: one family's run is split by the other's.
#[test]
fn universal_interleaved_flashes_each_chip_its_half() {
    let a = run(F_RP2040, 4);
    let b = run(F_ABSOLUTE, 4);
    let image = cat(&[&a[..3], &b, &a[3..]]);
    drop_it(ONLY_2040, &image, DONE, addrs(4));
    drop_it(ONLY_2350, &image, DONE, addrs(4));
    let two_late = cat(&[&a[..2], &b, &a[2..]]);
    drop_it(ONLY_2040, &two_late, DONE, addrs(4));
}

/// First family selects: on a fleet table the RP2040 half wins, and the other half is ignored.
#[test]
fn universal_on_a_multi_target_table_follows_its_first_family() {
    let image = cat(&[&run(F_RP2040, 4), &run(F_ABSOLUTE, 3)]);
    drop_it(FLEET, &image, DONE, addrs(4));
    drop_it(NO_RP2350, &image, DONE, addrs(4));
}

#[test]
fn uf2conv_without_a_family_needs_a_single_target_table() {
    let image: Vec<Block> = (0..3).map(|i| page(i, 3, None)).collect();
    drop_it(ONLY_2040, &image, DONE, addrs(3));
    drop_it(FLEET, &image, FAMILY, vec![]);
}

#[test]
fn uf2conv_default_base_is_out_of_range() {
    let image: Vec<Block> = (0..2)
        .map(|i| uf2_numbered(i, 2, 0x2000 + i * 256, &[0; 256]))
        .collect();
    drop_it(ONLY_2040, &image, RANGE, vec![]);
}

#[test]
fn no_block_of_a_composed_family_fails_family_with_no_session() {
    for (table, image) in [
        (ONLY_2350, run(F_RP2040, 3)),
        (ONLY_2040, run(F_RP2350, 3)),
        (ONLY_2040, cat(&[&[e10(true)], &run(F_RP2350, 3)])),
        (NO_RP2350, cat(&[&[e10(true)], &run(F_RP2350, 3)])),
    ] {
        drop_it(table, &image, FAMILY, vec![]);
    }
}

/// The record's RISC-V and non-secure families stay refused.
#[test]
fn rp2350_other_families_stay_refused() {
    for f in [0xE48B_FF5A, 0xE48B_FF5B] {
        drop_it(ONLY_2350, &run(f, 2), FAMILY, vec![]);
    }
}

/// A selection left over from the last drop must not filter the next one.
#[test]
fn the_next_drop_selects_afresh() {
    let mut h: VendorFlash<AllDecoders, FlashSink<Row>> =
        VendorFlash::new(FlashSink::new(row(NO_RP2350)));
    let mut resp = [0u8; 8];
    let mut stream = |image: &[Block]| {
        h.handle(&[0x8A, 2], &mut resp);
        let bytes: Vec<u8> = image.iter().flatten().copied().collect();
        for piece in bytes.chunks(62) {
            let mut r = vec![0x8C, piece.len() as u8];
            r.extend_from_slice(piece);
            h.handle(&r, &mut resp);
        }
        h.handle(&[0x8B], &mut resp);
        resp[1]
    };
    assert_eq!(stream(&run(F_RP2040, 2)), 0);
    let stm: Vec<Block> = (0..2)
        .map(|i| {
            let mut b = uf2_numbered(i, 2, 0x0800_0000 + i * 256, &[1; 256]);
            b[8..12].copy_from_slice(&FLAG_FAMILY.to_le_bytes());
            b[28..32].copy_from_slice(&F_STM32F4.to_le_bytes());
            b
        })
        .collect();
    assert_eq!(stream(&stm), 0);
    assert_eq!(h.sink().outcome(), Some(DONE));
}

#[test]
fn a_numbering_hole_or_duplicate_then_a_cut_is_short() {
    let p = |n, of| page(n, of, Some(F_RP2040));
    drop_it(
        ONLY_2040,
        &[p(0, 4), p(1, 4), p(3, 4)],
        SHORT,
        vec![XIP, XIP + 256, XIP + 768],
    );
    drop_it(
        ONLY_2040,
        &[p(0, 5), p(1, 5), p(1, 5), p(2, 5)],
        SHORT,
        vec![XIP, XIP + 256, XIP + 256, XIP + 512],
    );
}

#[test]
fn nothing_programmable_is_no_data() {
    let mut meta = page(0, 1, Some(F_RP2040));
    meta[8..12].copy_from_slice(&(FLAG_FAMILY | 1).to_le_bytes());
    drop_it(ONLY_2040, &[meta], TransferOutcome::NoData, vec![]);
    let (end, status) = vendor(ONLY_2040, &[]);
    assert_eq!(end.outcome, Some(TransferOutcome::NoData));
    assert_ne!(status, 0);
}

/// No count on the wire judges these two, so they stay `Done` on the vendor path.
#[test]
fn an_uncounted_cut_and_a_cut_second_run_still_close_ok() {
    let uncounted: Vec<Block> = (0..2).map(|i| page(i, 0, Some(F_RP2040))).collect();
    let second_run = [
        page(0, 2, Some(F_RP2040)),
        page(1, 2, Some(F_RP2040)),
        page(0, 3, Some(F_RP2040)),
    ];
    for image in [&uncounted[..], &second_run[..]] {
        let (end, status) = vendor(ONLY_2040, image);
        assert_eq!((end.outcome, status), (Some(DONE), 0));
    }
}

#[test]
fn a_decoder_rejection_releases_the_session_with_its_reply() {
    let mut h: VendorFlash<AllDecoders, FlashSink<Row>> =
        VendorFlash::new(FlashSink::new(row(ONLY_2040)));
    let mut resp = [0u8; 8];
    h.handle(&[0x8A, 2], &mut resp);
    let mut image: Vec<u8> = run(F_RP2040, 2).iter().flatten().copied().collect();
    image[512 + 508..].fill(0);
    for piece in image.chunks(62) {
        let mut r = vec![0x8C, piece.len() as u8];
        r.extend_from_slice(piece);
        h.handle(&r, &mut resp);
        if resp[1] != 0 {
            break;
        }
    }
    assert_eq!(
        h.sink().backend().finishes,
        [false],
        "session held after the reply"
    );
}

#[test]
fn a_session_reset_retires_an_abandoned_stream() {
    let mut h: VendorFlash<AllDecoders, FlashSink<Row>> =
        VendorFlash::new(FlashSink::new(row(ONLY_2040)));
    let mut resp = [0u8; 8];
    h.handle(&[0x8A, 2], &mut resp);
    let image = run(F_RP2040, 2)[0];
    let mut r = vec![0x8C, 62];
    r.extend_from_slice(&image[..62]);
    h.handle(&r, &mut resp);
    h.handle(&r, &mut resp);
    h.session_reset();
    h.handle(&[0x8A, 2], &mut resp);
    assert_eq!(resp[1], 0, "a new host opens");
}
