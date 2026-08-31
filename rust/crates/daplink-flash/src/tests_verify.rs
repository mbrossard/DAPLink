//! Read-back verify tests: word placement, mismatch detection, and the memory-mapped-window retry.

extern crate std;

use core::cell::Cell;
use dap_rs::swd::{APnDP, Config, DPRegister, Error as SwdErr, Result as SwdResult, Swd};
use embedded_hal::delay::DelayNs;
use std::boxed::Box;

use crate::error::Error;
use crate::flash_algo::{FlashAlgorithm, FlashRegion, RamRegion, SectorInfo, SyscallParams};
use crate::reset::ResetControl;
use crate::target::{DefaultCallbacks, Target};
use crate::target_flash::TargetFlash;

/// Geometry of the target under test: 1 KiB pages, no `Verify` entry (read-back fallback).
const FLASH_BASE: u32 = 0x0800_0000;
const FLASH_END: u32 = 0x0800_4000;
const PAGE: usize = 0x400;
const ALGO_START: u32 = 0x2000_0000;
const INIT_OFF: u32 = 0x20;
const UNINIT_OFF: u32 = 0x40;

/// Core-debug registers the mock answers specially, and the DCRSR selector for a PC write.
const DCRSR: u32 = 0xE000_EDF4;
const DCRDR: u32 = 0xE000_EDF8;
const PC_WRITE_SELECT: u32 = 15 | (1 << 16);

/// The word this mock's flash holds at `addr`, distinct per address.
fn flash_word(addr: u32) -> u32 {
    addr ^ 0xA5A5_A5A5
}

/// A page of the bytes [`flash_word`] describes, starting at `addr`.
fn page_bytes(addr: u32) -> [u8; PAGE] {
    let mut page = [0u8; PAGE];
    for (i, chunk) in page.chunks_mut(4).enumerate() {
        chunk.copy_from_slice(&flash_word(addr + (i as u32) * 4).to_le_bytes());
    }
    page
}

/// Shared configuration and observations for one session.
#[derive(Default)]
struct Bus {
    /// Every SWD transaction the driver was asked to run.
    ops: Cell<u32>,
    /// Algorithm `Init` / `UnInit` entries actually executed.
    inits: Cell<u32>,
    uninits: Cell<u32>,
    /// Whether the algorithm is currently initialized (`Init` ran, `UnInit` has not).
    algo_live: Cell<bool>,
    /// Config: flash reads back as garbage while the algorithm is initialized.
    window_hidden: Cell<bool>,
    /// Config: one flash address that reads back as the complement of what was written.
    poison: Cell<Option<u32>>,
}

fn leak_bus() -> &'static Bus {
    Box::leak(Box::new(Bus::default()))
}

/// SWD driver mock over a flat memory model with ADIv5 posted-read and TAR auto-increment
/// semantics.
struct MockSwd {
    cfg: Config,
    bus: &'static Bus,
    /// Current TAR, which auto-increments on every DRW access exactly as on silicon.
    tar: u32,
    /// The posted read awaiting delivery by the next AP or RDBUFF read.
    posted: u32,
    /// Value staged into DCRDR, committed as the routine entry by a PC write.
    pending_dcrdr: u32,
}

impl MockSwd {
    fn new(bus: &'static Bus) -> Self {
        Self {
            cfg: Config::default(),
            bus,
            tar: 0,
            posted: 0,
            pending_dcrdr: 0,
        }
    }

    /// The word stored at `addr` in the model.
    fn mem(&self, addr: u32) -> u32 {
        if addr == DCRDR {
            return 0;
        }
        if !(FLASH_BASE..FLASH_END).contains(&addr) {
            return 0xFFFF_FFFF;
        }
        if self.bus.window_hidden.get() && self.bus.algo_live.get() {
            return 0xDEAD_BEEF;
        }
        if self.bus.poison.get() == Some(addr) {
            return !flash_word(addr);
        }
        flash_word(addr)
    }
}

impl Swd<MockSwd> for MockSwd {
    fn available(_deps: &MockSwd) -> bool {
        true
    }
    fn config(&mut self) -> &mut Config {
        &mut self.cfg
    }
    fn read_inner(&mut self, apndp: APnDP, a: DPRegister) -> SwdResult<u32> {
        self.bus.ops.set(self.bus.ops.get() + 1);
        match (apndp, a) {
            // Posted AP read: returns the previous read's data; the access itself advances TAR.
            (APnDP::AP, _) => {
                let value = self.mem(self.tar);
                self.tar = self.tar.wrapping_add(4);
                Ok(core::mem::replace(&mut self.posted, value))
            }
            (APnDP::DP, DPRegister::RDBUFF) => Ok(self.posted),
            (APnDP::DP, _) => Ok(0xFFFF_FFFF),
        }
    }
    fn write_inner(&mut self, apndp: APnDP, a: DPRegister, data: u32) -> SwdResult<()> {
        self.bus.ops.set(self.bus.ops.get() + 1);
        if apndp != APnDP::AP {
            return Ok(());
        }
        if a == DPRegister::CTRLSTAT {
            self.tar = data;
            return Ok(());
        }
        if a != DPRegister::RDBUFF {
            return Ok(());
        }
        let addr = self.tar;
        self.tar = self.tar.wrapping_add(4);
        if addr == DCRDR {
            self.pending_dcrdr = data;
        } else if addr == DCRSR && data == PC_WRITE_SELECT {
            if self.pending_dcrdr == ALGO_START + INIT_OFF {
                self.bus.inits.set(self.bus.inits.get() + 1);
                self.bus.algo_live.set(true);
            } else if self.pending_dcrdr == ALGO_START + UNINIT_OFF {
                self.bus.uninits.set(self.bus.uninits.get() + 1);
                self.bus.algo_live.set(false);
            }
        }
        Ok(())
    }
    fn set_clock(&mut self, _max_frequency: u32) -> bool {
        true
    }
    fn write_sequence(&mut self, _num_bits: usize, _data: &[u8]) -> SwdResult<()> {
        Ok(())
    }
    fn read_sequence(&mut self, _num_bits: usize, _data: &mut [u8]) -> SwdResult<()> {
        Ok(())
    }
}

impl ResetControl for MockSwd {
    type Error = SwdErr;
    fn set_reset(&mut self, _asserted: bool) -> Result<(), SwdErr> {
        Ok(())
    }
}

struct NoDelay;
impl DelayNs for NoDelay {
    fn delay_ns(&mut self, _ns: u32) {}
}

static BLOB: [u8; 364] = [0; 364];
static ALGO: FlashAlgorithm<'static> = FlashAlgorithm {
    init: INIT_OFF,
    uninit: UNINIT_OFF,
    erase_chip: 0x60,
    erase_sector: 0x80,
    program_page: 0xA0,
    verify: None,
    syscall: SyscallParams {
        breakpoint: ALGO_START,
        static_base: 0x2000_0100,
        stack_pointer: 0x2000_1000,
    },
    program_buffer: 0x2000_0200,
    program_buffer_size: PAGE as u32,
    algo_start: ALGO_START,
    algo_blob: &BLOB,
    verify_returns_address: false,
    single_init_type: false,
    skip_chip_erase: false,
};
static FLASH_REGIONS: [FlashRegion<'static>; 1] = [FlashRegion {
    start: FLASH_BASE,
    end: FLASH_END,
    is_default: true,
    is_secure: false,
    algo: &ALGO,
}];
static RAM_REGIONS: [RamRegion; 1] = [RamRegion {
    start: 0x2000_0000,
    end: 0x2000_8000,
}];
static SECTORS: [SectorInfo; 1] = [SectorInfo {
    start: FLASH_BASE,
    size: PAGE as u32,
    count: 16,
}];

struct MockTarget;
impl Target for MockTarget {
    fn flash_regions(&self) -> &[FlashRegion<'_>] {
        &FLASH_REGIONS
    }
    fn ram_regions(&self) -> &[RamRegion] {
        &RAM_REGIONS
    }
    fn sectors(&self) -> &[SectorInfo] {
        &SECTORS
    }
}

type Flash<'a> = TargetFlash<'a, MockSwd, NoDelay, MockTarget, DefaultCallbacks, MockSwd>;

fn flash<'a>(bus: &'static Bus, target: &'a MockTarget) -> Flash<'a> {
    TargetFlash::new(MockSwd::new(bus), NoDelay, target, DefaultCallbacks)
}

/// Program `pages` pages, verifying each when asked, and return the transaction count.
fn run_pages(bus: &'static Bus, pages: u32, verify: bool) -> u32 {
    let target = MockTarget;
    let mut tf = flash(bus, &target);
    tf.init().expect("init");
    let before = bus.ops.get();
    for p in 0..pages {
        let addr = FLASH_BASE + p * PAGE as u32;
        let page = page_bytes(addr);
        tf.program_page(addr, &page).expect("program");
        if verify {
            tf.verify(addr, &page).expect("verify");
        }
    }
    bus.ops.get() - before
}

/// The whole page reads back through the block path, word for word.
#[test]
fn readback_verify_matches_every_word_of_a_page() {
    let bus = leak_bus();
    let target = MockTarget;
    let mut tf = flash(bus, &target);
    tf.init().expect("init");
    let page = page_bytes(FLASH_BASE);
    tf.program_page(FLASH_BASE, &page).expect("program");
    tf.verify(FLASH_BASE, &page)
        .expect("a faithful page verifies");
}

/// A run longer than the 1 KiB TAR auto-increment window re-arms TAR at every crossing.
#[test]
fn readback_verify_spans_the_tar_auto_increment_window() {
    let bus = leak_bus();
    let target = MockTarget;
    let mut tf = flash(bus, &target);
    tf.init().expect("init");
    let mut run = [0u8; 4 * PAGE];
    for (i, chunk) in run.chunks_mut(4).enumerate() {
        chunk.copy_from_slice(&flash_word(FLASH_BASE + (i as u32) * 4).to_le_bytes());
    }
    let before = bus.ops.get();
    tf.verify(FLASH_BASE, &run).expect("4 KiB verifies");
    assert_eq!(
        bus.ops.get() - before,
        4 * 258,
        "one TAR write and one pipeline prime per 1 KiB window"
    );

    let bad = FLASH_BASE + 3 * PAGE as u32 + 0x40;
    bus.poison.set(Some(bad));
    assert_eq!(
        tf.verify(FLASH_BASE, &run),
        Err(Error::VerifyFailed { addr: bad })
    );
}

/// A single wrong word mid-page still fails, and names its own address.
#[test]
fn readback_verify_catches_a_mismatch_mid_page() {
    for word in [0u32, 1, 137, 255] {
        let bus = leak_bus();
        let bad = FLASH_BASE + word * 4;
        bus.poison.set(Some(bad));
        let target = MockTarget;
        let mut tf = flash(bus, &target);
        tf.init().expect("init");
        let page = page_bytes(FLASH_BASE);
        tf.program_page(FLASH_BASE, &page).expect("program");
        assert_eq!(
            tf.verify(FLASH_BASE, &page),
            Err(Error::VerifyFailed { addr: bad }),
            "the failing word's own address is reported"
        );
    }
}

/// A target whose flash is unreadable while its algorithm is initialized still verifies: the
/// mismatch is retried after the algorithm is torn down.
#[test]
fn readback_verify_retries_behind_a_hidden_window() {
    let bus = leak_bus();
    bus.window_hidden.set(true);
    let target = MockTarget;
    let mut tf = flash(bus, &target);
    tf.init().expect("init");
    let page = page_bytes(FLASH_BASE);
    tf.program_page(FLASH_BASE, &page).expect("program");
    tf.verify(FLASH_BASE, &page)
        .expect("the read-back succeeds once the window is restored");
    assert_eq!(bus.uninits.get(), 1, "the algorithm was torn down once");
}

/// What one verified page costs, counted in the steady state (a later page, so neither figure
/// carries the session's one-time algorithm load).
#[test]
fn readback_verify_costs_one_transaction_a_word() {
    let marginal =
        |verify: bool| run_pages(leak_bus(), 3, verify) - run_pages(leak_bus(), 2, verify);
    assert_eq!(
        marginal(true) - marginal(false),
        258,
        "a 1 KiB page verifies in one TAR write, one pipeline prime, 255 posted reads and \
         one RDBUFF fetch"
    );

    let bus = leak_bus();
    run_pages(bus, 3, true);
    assert_eq!(bus.inits.get(), 1, "one Init for the whole session");
    assert_eq!(bus.uninits.get(), 0, "no teardown on a passing verify");
}
