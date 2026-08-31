//! The hardware flash backend: lends SWD, delay and staging buffer to a session, reclaims on finish.

use dap_rs::swd::Swd;
use embedded_hal::delay::DelayNs;

use daplink_core::flash_sink::{FlashBackend, FlashPhase, TargetWindows};
use daplink_core::probe::{FlashArbiter, ProbeInfo, ResetControl};
use daplink_flash::target::{Target, TargetCallbacks};
use daplink_flash::{EraseMode, Error, FlashManager, TargetFlash};
use daplink_targets::{ImageHint, SelectError, TargetSelector};

/// Maximum number of programmable flash ranges cached per session.
const MAX_RANGES: usize = 8;

/// Maximum number of composed flash/RAM window pairings cached; raise it for a sixth table row,
/// or a target with a second flash or RAM region.
const MAX_COMPOSED_WINDOWS: usize = 5;

/// Maximum number of RAM ranges cached for the selected target.
const MAX_RAM_RANGES: usize = 2;

/// Report a window that did not fit its fixed-capacity table; overflow only makes the
/// range checks refuse more, never accept more.
fn note_dropped_window(table: &'static str) {
    let _ = table;
    debug_assert!(false, "composed target windows exceeded a fixed capacity");
}

/// The page-buffering manager wrapping the on-target flash driver for one session.
type SessionManager<'a, S, D, T, C, DEPS> = FlashManager<'a, TargetFlash<'a, S, D, T, C, DEPS>>;

/// The session's flash driver, driving whichever `dyn Target` the selector resolved.
type SessionFlash<'a, S, D, C, DEPS> = TargetFlash<'a, S, D, dyn Target + 'a, C, DEPS>;

/// Concrete [`FlashBackend`] backed by the on-target flash-algorithm runtime. Generic over the SWD
/// driver `S`, delay `D`, target `T`, callbacks `C`, reset `R`, selector `Sel`, arbiter `A`, `DEPS`.
pub struct HwBackend<'a, S, D, T, C, R, Sel, A, DEPS>
where
    S: Swd<DEPS> + ResetControl,
    D: DelayNs,
    T: Target,
    C: TargetCallbacks<S, D> + Clone,
    R: ResetControl + ProbeInfo,
    Sel: TargetSelector,
    A: FlashArbiter,
{
    swd: Option<S>,
    delay: Option<D>,
    buffer: Option<&'a mut [u8]>,
    callbacks: C,
    reset: R,
    selector: Sel,
    target: &'a T,
    table: &'a [&'a dyn Target],
    /// The target the selector resolved for the current/last session; `None` before the first
    /// `begin`.
    selected: Option<&'a dyn Target>,
    auto_reset: bool,
    manager: Option<SessionManager<'a, S, D, dyn Target + 'a, C, DEPS>>,
    ranges: heapless::Vec<(u32, u32), MAX_RANGES>,
    /// The selected target's RAM regions, cached alongside `ranges` at `begin`.
    ram_ranges: heapless::Vec<(u32, u32), MAX_RAM_RANGES>,
    /// Every composed target's flash windows paired with that same target's RAM windows; paired,
    /// never pooled, so no image is accepted on one part's RAM and another's flash.
    composed_windows: heapless::Vec<TargetWindows, MAX_COMPOSED_WINDOWS>,
    arbiter: A,
    session: Option<A::Session>,
    /// Erase granularity applied to each session's manager; per-sector unless a host selects
    /// chip erase through the vendor surface.
    sector_erase: bool,
}

impl<'a, S, D, T, C, R, Sel, A, DEPS> HwBackend<'a, S, D, T, C, R, Sel, A, DEPS>
where
    S: Swd<DEPS> + ResetControl,
    D: DelayNs,
    T: Target,
    C: TargetCallbacks<S, D> + Clone,
    R: ResetControl + ProbeInfo,
    Sel: TargetSelector,
    A: FlashArbiter,
{
    /// Build a backend from its owned parts. `swd`/`delay`/`buffer` are lent to the flash runtime
    /// per session and reclaimed afterwards; `arbiter` gates exclusive bus access.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        swd: S,
        delay: D,
        callbacks: C,
        reset: R,
        selector: Sel,
        target: &'a T,
        table: &'a [&'a dyn Target],
        buffer: &'a mut [u8],
        arbiter: A,
        auto_reset: bool,
    ) -> Self {
        // The composed windows are a fact about the firmware, not about a session.
        let mut composed_windows = heapless::Vec::new();
        for entry in table {
            for flash in entry.flash_regions() {
                for ram in entry.ram_regions() {
                    let window = TargetWindows {
                        flash: (flash.start, flash.end),
                        ram: (ram.start, ram.end),
                    };
                    if composed_windows.push(window).is_err() {
                        note_dropped_window("MAX_COMPOSED_WINDOWS");
                    }
                }
            }
        }
        Self {
            swd: Some(swd),
            delay: Some(delay),
            buffer: Some(buffer),
            callbacks,
            reset,
            selector,
            target,
            table,
            selected: None,
            auto_reset,
            manager: None,
            ranges: heapless::Vec::new(),
            ram_ranges: heapless::Vec::new(),
            composed_windows,
            arbiter,
            session: None,
            sector_erase: true,
        }
    }

    /// Read the attached debug port's DPIDR through the probe handle; `None` if nothing answers.
    /// The presence gate, outside any session: a caller checks the wire before opening one.
    pub fn read_dpidr(&mut self) -> Option<u32> {
        self.reset.read_dpidr()
    }

    /// Reclaim the SWD driver, delay and buffer from a finished or aborted manager.
    fn recover_from_manager(&mut self, parts: (SessionFlash<'a, S, D, C, DEPS>, &'a mut [u8])) {
        let (flash, buffer) = parts;
        let (swd, delay) = flash.into_parts();
        self.swd = Some(swd);
        self.delay = Some(delay);
        self.buffer = Some(buffer);
    }

    /// Restore whatever parts are passed, drop any in-flight session, and report an init failure.
    /// It never touches the target: no caller reaches it with a debug session still established.
    fn fail_init(
        &mut self,
        swd: Option<S>,
        delay: Option<D>,
        buffer: Option<&'a mut [u8]>,
    ) -> FlashPhase {
        self.swd = swd;
        self.delay = delay;
        self.buffer = buffer;
        self.session = None;
        FlashPhase::Init
    }

    /// Map a flash-runtime error onto the phase currency of the [`FlashBackend`] port.
    fn program_phase(err: Error) -> FlashPhase {
        match err {
            Error::EraseFailed | Error::EraseNotSupported => FlashPhase::Erase,
            // An address the target maps no flash region to; sectors resolve only at a flush.
            Error::AddressOutOfRange { .. } | Error::NoFlashRegion { .. } => FlashPhase::Range,
            Error::VerifyFailed { .. } => FlashPhase::Verify,
            _ => FlashPhase::Program,
        }
    }

    /// Claim exclusive bus access, resolve a target, and cache its flash/RAM ranges. On failure the
    /// just-claimed session is released first, so a rejection never leaves the DAP engine gated.
    fn claim_and_select(&mut self, family: Option<u32>) -> Result<&'a dyn Target, FlashPhase> {
        let Some(session) = self.arbiter.begin_session() else {
            return Err(FlashPhase::NoTarget);
        };
        self.session = Some(session);

        let hint = match family {
            Some(f) => ImageHint::uf2(f),
            None => ImageHint::NONE,
        };
        let selected = match self.selector.select(hint, self.table) {
            Ok(target) => target,
            Err(err) => {
                self.session = None;
                return Err(match err {
                    SelectError::NoTarget => FlashPhase::NoTarget,
                    SelectError::ImageMismatch => FlashPhase::Family,
                });
            }
        };
        self.selected = Some(selected);

        self.ranges.clear();
        for region in selected.flash_regions() {
            if self.ranges.push((region.start, region.end)).is_err() {
                note_dropped_window("MAX_RANGES");
            }
        }
        self.ram_ranges.clear();
        for region in selected.ram_regions() {
            if self.ram_ranges.push((region.start, region.end)).is_err() {
                note_dropped_window("MAX_RAM_RANGES");
            }
        }

        Ok(selected)
    }
}

impl<S, D, T, C, R, Sel, A, DEPS> FlashBackend for HwBackend<'_, S, D, T, C, R, Sel, A, DEPS>
where
    S: Swd<DEPS> + ResetControl,
    D: DelayNs,
    T: Target,
    C: TargetCallbacks<S, D> + Clone,
    R: ResetControl + ProbeInfo,
    Sel: TargetSelector,
    A: FlashArbiter,
{
    fn begin(&mut self) -> Result<(), FlashPhase> {
        self.begin_with_hint(None)
    }

    fn begin_with_hint(&mut self, family: Option<u32>) -> Result<(), FlashPhase> {
        let selected = self.claim_and_select(family)?;

        // Take the lent parts; if any is already out, restore and fail.
        let swd = self.swd.take();
        let delay = self.delay.take();
        let buffer = self.buffer.take();
        let (swd, delay, buffer) = match (swd, delay, buffer) {
            (Some(s), Some(d), Some(b)) => (s, d, b),
            (s, d, b) => return Err(self.fail_init(s, d, b)),
        };

        // Same predicate `FlashManager::new` applies, while all three parts can still be restored.
        if !daplink_flash::page_buffer_ok(buffer.len()) {
            return Err(self.fail_init(Some(swd), Some(delay), Some(buffer)));
        }

        let mut flash: SessionFlash<'_, S, D, C, DEPS> =
            TargetFlash::new(swd, delay, selected, self.callbacks.clone());
        // The boot-time policy governs teardowns with no flag; `finish_with_reset` overrides it.
        flash.set_post_flash_reset(self.auto_reset);
        if let Err(err) = flash.init() {
            // `init` has already driven the target back to running on its way out.
            let (swd, delay) = flash.into_parts();
            let _ = self.fail_init(Some(swd), Some(delay), Some(buffer));
            return Err(match err {
                Error::IdentityMismatch { .. } => FlashPhase::Identity,
                _ => FlashPhase::Init,
            });
        }

        // Unreachable: `new` rejects only what `page_buffer_ok` above already rejected.
        let mut manager = match FlashManager::new(flash, buffer) {
            Ok(manager) => manager,
            Err(_) => {
                self.session = None;
                return Err(FlashPhase::Init);
            }
        };
        manager.set_erase_mode(if self.sector_erase {
            EraseMode::Sector
        } else {
            EraseMode::Chip
        });
        // Verify inline: the staging buffer still holds the expected bytes only at this moment.
        manager.set_verify(true);
        self.manager = Some(manager);
        Ok(())
    }

    fn set_sector_erase(&mut self, sector: bool) -> Result<(), FlashPhase> {
        self.sector_erase = sector;
        Ok(())
    }

    fn erase_all(&mut self) -> Result<(), FlashPhase> {
        // A mass-erase carries no image, so the selector runs exactly as for a hintless `begin`.
        let selected = self.claim_and_select(None)?;

        // Unlike `begin_with_hint`, a bare mass-erase never touches the staging buffer.
        let swd = self.swd.take();
        let delay = self.delay.take();
        let (swd, delay) = match (swd, delay) {
            (Some(s), Some(d)) => (s, d),
            (s, d) => {
                self.swd = s;
                self.delay = d;
                self.session = None;
                return Err(FlashPhase::Init);
            }
        };

        let mut flash: SessionFlash<'_, S, D, C, DEPS> =
            TargetFlash::new(swd, delay, selected, self.callbacks.clone());
        flash.set_post_flash_reset(self.auto_reset);
        if let Err(err) = flash.init() {
            let (swd, delay) = flash.into_parts();
            self.swd = Some(swd);
            self.delay = Some(delay);
            self.session = None;
            return Err(match err {
                Error::IdentityMismatch { .. } => FlashPhase::Identity,
                _ => FlashPhase::Init,
            });
        }

        // Tear down unconditionally, so no erase error leaves the target halted mid-algorithm.
        let erase_result = flash.erase_chip();
        let uninit_result = flash.uninit();
        let (swd, delay) = flash.into_parts();
        self.swd = Some(swd);
        self.delay = Some(delay);
        self.session = None;

        if erase_result.is_err() {
            // Every `erase_chip` failure reports the `Erase` phase, like the default it overrides.
            return Err(FlashPhase::Erase);
        }
        if uninit_result.is_err() {
            return Err(FlashPhase::Uninit);
        }
        Ok(())
    }

    fn program(&mut self, addr: u32, data: &[u8]) -> Result<(), FlashPhase> {
        self.manager
            .as_mut()
            .ok_or(FlashPhase::Program)?
            .write(addr, data)
            .map_err(Self::program_phase)
    }

    fn finish(&mut self, success: bool) -> Result<(), FlashPhase> {
        // No live policy supplied: fall back to the boot-time composition.
        self.finish_with_reset(success, self.auto_reset)
    }

    fn finish_with_reset(&mut self, success: bool, reset: bool) -> Result<(), FlashPhase> {
        let manager = match self.manager.take() {
            Some(manager) => manager,
            None => {
                // Nothing was programmed; make sure no session lingers regardless.
                self.session = None;
                return Ok(());
            }
        };

        if !success {
            // The abort path takes no reset argument, but still releases the target it halted.
            let parts = manager.abort();
            self.recover_from_manager(parts);
        } else {
            // Recover the lent parts on both arms, so a programming error leaves the backend usable.
            let (result, parts) = manager.finish(reset);
            self.recover_from_manager(parts);
            if let Err(err) = result {
                self.session = None;
                return Err(Self::program_phase(err));
            }
        }

        // The teardown applies the reset best-effort when set, and leaves the core halted when not.
        self.session = None;
        Ok(())
    }

    fn flash_ranges(&self) -> &[(u32, u32)] {
        &self.ranges
    }

    fn ram_ranges(&self) -> &[(u32, u32)] {
        // The selected target's RAM, empty until a session resolves one.
        &self.ram_ranges
    }

    fn candidate_windows(&self) -> &[TargetWindows] {
        // One pairing per composed target, the only set available before a session picks a chip.
        &self.composed_windows
    }

    fn expected_uf2_family(&self) -> Option<u32> {
        // Follow the selected target; before the first `begin`, the composed one.
        match self.selected {
            Some(target) => target.uf2_family(),
            None => self.target.uf2_family(),
        }
    }

    fn selected_target_name(&self) -> Option<&str> {
        // Empty names surface as `None`, so a status line omits them.
        self.selected
            .map(|t| t.name())
            .filter(|name| !name.is_empty())
    }
}

#[cfg(test)]
mod hw_regression {
    extern crate std;

    use super::*;
    use core::cell::Cell;
    use dap_rs::swd::{APnDP, DPRegister};
    use daplink_flash::flash_algo::{
        FlashAlgorithm, FlashRegion, RamRegion, SectorInfo, SyscallParams,
    };
    use daplink_flash::target::DefaultCallbacks;
    use daplink_targets::FixedTarget;
    use std::boxed::Box;

    // DCRDR carries the algorithm status word (0 = success); every other read returns all-ones.
    const DCRDR: u32 = 0xE000_EDF8;
    // DCRSR selects which core register a DCRDR access targets.
    const DCRSR: u32 = 0xE000_EDF4;
    // Core-register write selector for r15 (PC): register number plus bit 16.
    const PC_WRITE_SELECT: u32 = 15 | (1 << 16);
    // Entry addresses of the mock algorithm's routines (algo_start + offset).
    const PROGRAM_ENTRY: u32 = 0x2000_00A0;
    const VERIFY_ENTRY: u32 = 0x2000_00C0;
    const ERASE_CHIP_ENTRY: u32 = 0x2000_0060;
    // The DHCSR write that releases the core to run: the tell-tale of a reset-to-run.
    const DHCSR: u32 = 0xE000_EDF0;
    const DHCSR_RELEASE_RUN: u32 = 0xA05F_0000;

    /// SWD driver mock: every poll and syscall succeeds unless a one-shot failure is armed.
    struct MockSwd {
        cfg: dap_rs::swd::Config,
        /// Current TAR, auto-incrementing on every DRW access exactly as on silicon.
        last_tar: u32,
        /// The posted read awaiting delivery by the next AP or RDBUFF read.
        posted: u32,
        fail_init: bool,
        /// Last value staged into DCRDR.
        pending_dcrdr: u32,
        /// Entry address of the routine most recently loaded into the PC.
        last_entry: u32,
        fail_program: bool,
        fail_verify: bool,
        fail_erase: bool,
        /// Fail `set_reset` only once the reset-to-run phase has begun.
        fail_reset: bool,
        /// Set once DHCSR has been released to run, which only the post-flash reset does.
        run_released: bool,
        /// Optional counter of those releases, so a test can prove the target was let go.
        releases: Option<&'static Cell<u32>>,
    }

    impl MockSwd {
        fn new(fail_init: bool) -> Self {
            Self {
                cfg: dap_rs::swd::Config::default(),
                last_tar: 0,
                posted: 0,
                fail_init,
                pending_dcrdr: 0,
                last_entry: 0,
                fail_program: false,
                fail_verify: false,
                fail_erase: false,
                fail_reset: false,
                run_released: false,
                releases: None,
            }
        }

        /// Count every release-to-run into `cell`.
        fn counting_releases(mut self, cell: &'static Cell<u32>) -> Self {
            self.releases = Some(cell);
            self
        }

        /// Arm a one-shot ProgramPage failure.
        fn with_fail_program(mut self) -> Self {
            self.fail_program = true;
            self
        }

        /// Arm a one-shot Verify failure: the page programs cleanly and reads back wrong.
        fn with_fail_verify(mut self) -> Self {
            self.fail_verify = true;
            self
        }

        /// Arm a one-shot EraseChip failure.
        fn with_fail_erase(mut self) -> Self {
            self.fail_erase = true;
            self
        }

        /// Fail the post-flash reset-to-run nRESET assertion; the init reset-and-halt still works.
        fn with_fail_reset(mut self) -> Self {
            self.fail_reset = true;
            self
        }

        /// The word held at `addr`: DCRDR gives the algorithm's r0 status, everything else all-ones.
        fn mem(&mut self, addr: u32) -> u32 {
            if addr != DCRDR {
                return 0xFFFF_FFFF;
            }
            if self.fail_program && self.last_entry == PROGRAM_ENTRY {
                self.fail_program = false;
                1
            } else if self.fail_verify && self.last_entry == VERIFY_ENTRY {
                self.fail_verify = false;
                1
            } else if self.fail_erase && self.last_entry == ERASE_CHIP_ENTRY {
                self.fail_erase = false;
                1
            } else {
                0
            }
        }
    }

    // `Swd<DEPS>: From<DEPS>`; the reflexive blanket impl satisfies the bound for `DEPS = MockSwd`.
    impl Swd<MockSwd> for MockSwd {
        fn available(_deps: &MockSwd) -> bool {
            true
        }
        fn config(&mut self) -> &mut dap_rs::swd::Config {
            &mut self.cfg
        }
        fn read_inner(&mut self, apndp: APnDP, a: DPRegister) -> dap_rs::swd::Result<u32> {
            match (apndp, a) {
                // A posted AP read returns the previous read's data and starts this one.
                (APnDP::AP, _) => {
                    let addr = self.last_tar;
                    self.last_tar = self.last_tar.wrapping_add(4);
                    let value = self.mem(addr);
                    Ok(core::mem::replace(&mut self.posted, value))
                }
                (APnDP::DP, DPRegister::RDBUFF) => Ok(self.posted),
                (APnDP::DP, _) => Ok(0xFFFF_FFFF),
            }
        }
        fn write_inner(
            &mut self,
            apndp: APnDP,
            a: DPRegister,
            data: u32,
        ) -> dap_rs::swd::Result<()> {
            // TAR lives at AP offset 0x04, encoded as `DPRegister::CTRLSTAT`.
            if apndp == APnDP::AP && a == DPRegister::CTRLSTAT {
                self.last_tar = data;
            }
            // DRW writes (AP offset 0x0C) carry memory data and auto-increment TAR like a read.
            if apndp == APnDP::AP && a == DPRegister::RDBUFF {
                let addr = self.last_tar;
                self.last_tar = self.last_tar.wrapping_add(4);
                if addr == DCRDR {
                    self.pending_dcrdr = data;
                } else if addr == DCRSR && data == PC_WRITE_SELECT {
                    self.last_entry = self.pending_dcrdr;
                } else if addr == DHCSR && data == DHCSR_RELEASE_RUN {
                    self.run_released = true;
                    if let Some(cell) = self.releases {
                        cell.set(cell.get() + 1);
                    }
                }
            }
            Ok(())
        }
        fn set_clock(&mut self, _max_frequency: u32) -> bool {
            true
        }
        fn write_sequence(&mut self, num_bits: usize, _data: &[u8]) -> dap_rs::swd::Result<()> {
            // The JTAG-to-SWD switch word is the 16-bit sequence in `init_debug`.
            if self.fail_init && num_bits == 16 {
                self.fail_init = false;
                return Err(dap_rs::swd::Error::NoAck);
            }
            Ok(())
        }
        fn read_sequence(&mut self, _num_bits: usize, _data: &mut [u8]) -> dap_rs::swd::Result<()> {
            Ok(())
        }
    }

    impl ResetControl for MockSwd {
        type Error = ();
        fn set_reset(&mut self, _asserted: bool) -> Result<(), ()> {
            // Fail only the post-flash reset-to-run, so the init reset-and-halt still succeeds.
            if self.fail_reset && self.run_released {
                Err(())
            } else {
                Ok(())
            }
        }
    }

    /// Probe/reset mock used for target selection and the post-program reset pulse.
    struct MockProbe {
        fail_reset: bool,
    }

    impl ResetControl for MockProbe {
        type Error = ();
        fn set_reset(&mut self, _asserted: bool) -> Result<(), ()> {
            if self.fail_reset {
                Err(())
            } else {
                Ok(())
            }
        }
    }

    impl ProbeInfo for MockProbe {
        fn read_dpidr(&mut self) -> Option<u32> {
            Some(0x0BC1_1477)
        }
    }

    /// Minimal single-region target: 1 KiB of flash in one sector.
    static ALGO_BLOB: [u8; 4] = [0; 4];
    static ALGO: FlashAlgorithm<'static> = FlashAlgorithm {
        init: 0x20,
        uninit: 0x40,
        erase_chip: 0x60,
        erase_sector: 0x80,
        program_page: 0xA0,
        // Present, like the RP bootrom shims: the session's verify runs on the target.
        verify: Some(0xC0),
        syscall: SyscallParams {
            breakpoint: 0x2000_0000,
            static_base: 0x2000_0100,
            stack_pointer: 0x2000_1000,
        },
        program_buffer: 0x2000_0200,
        program_buffer_size: 1024,
        algo_start: 0x2000_0000,
        algo_blob: &ALGO_BLOB,
        verify_returns_address: false,
        single_init_type: false,
        skip_chip_erase: false,
    };
    static FLASH_REGIONS: [FlashRegion<'static>; 1] = [FlashRegion {
        start: 0x0800_0000,
        end: 0x0800_0400,
        is_default: true,
        is_secure: false,
        algo: &ALGO,
    }];
    static RAM_REGIONS: [RamRegion; 1] = [RamRegion {
        start: 0x2000_0000,
        end: 0x2000_2000,
    }];
    static SECTORS: [SectorInfo; 1] = [SectorInfo {
        start: 0x0800_0000,
        size: 0x400,
        count: 1,
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
        fn uf2_family(&self) -> Option<u32> {
            Some(0x1111_1111)
        }
    }

    // A second target at a different flash base and family, distinguishable from the composed one.
    static FLASH_REGIONS_B: [FlashRegion<'static>; 1] = [FlashRegion {
        start: 0x1000_0000,
        end: 0x1000_0400,
        is_default: true,
        is_secure: false,
        algo: &ALGO,
    }];
    static SECTORS_B: [SectorInfo; 1] = [SectorInfo {
        start: 0x1000_0000,
        size: 0x400,
        count: 1,
    }];
    // Deliberately not `MockTarget`'s RAM: sharing it would hide a pooled composition.
    static RAM_REGIONS_B: [RamRegion; 1] = [RamRegion {
        start: 0x2400_0000,
        end: 0x2408_0000,
    }];

    struct SecondTarget;
    impl Target for SecondTarget {
        fn flash_regions(&self) -> &[FlashRegion<'_>] {
            &FLASH_REGIONS_B
        }
        fn ram_regions(&self) -> &[RamRegion] {
            &RAM_REGIONS_B
        }
        fn sectors(&self) -> &[SectorInfo] {
            &SECTORS_B
        }
        fn uf2_family(&self) -> Option<u32> {
            Some(0x2222_2222)
        }
    }

    // A third target whose algorithm exports no `Verify`, so its session compares by read-back.
    static ALGO_NOVERIFY: FlashAlgorithm<'static> = FlashAlgorithm {
        init: 0x20,
        uninit: 0x40,
        erase_chip: 0x60,
        erase_sector: 0x80,
        program_page: 0xA0,
        verify: None,
        syscall: SyscallParams {
            breakpoint: 0x2000_0000,
            static_base: 0x2000_0100,
            stack_pointer: 0x2000_1000,
        },
        program_buffer: 0x2000_0200,
        program_buffer_size: 1024,
        algo_start: 0x2000_0000,
        algo_blob: &ALGO_BLOB,
        verify_returns_address: false,
        single_init_type: false,
        skip_chip_erase: false,
    };
    static FLASH_REGIONS_C: [FlashRegion<'static>; 1] = [FlashRegion {
        start: 0x0900_0000,
        end: 0x0900_0400,
        is_default: true,
        is_secure: false,
        algo: &ALGO_NOVERIFY,
    }];
    static SECTORS_C: [SectorInfo; 1] = [SectorInfo {
        start: 0x0900_0000,
        size: 0x400,
        count: 1,
    }];

    struct ReadbackTarget;
    impl Target for ReadbackTarget {
        fn flash_regions(&self) -> &[FlashRegion<'_>] {
            &FLASH_REGIONS_C
        }
        fn ram_regions(&self) -> &[RamRegion] {
            &RAM_REGIONS
        }
        fn sectors(&self) -> &[SectorInfo] {
            &SECTORS_C
        }
        fn uf2_family(&self) -> Option<u32> {
            Some(0x3333_3333)
        }
    }

    /// Selector that records the image hint it was handed and resolves to the first table row.
    #[derive(Clone, Copy)]
    struct HintSelector {
        seen: &'static Cell<Option<u32>>,
    }
    impl TargetSelector for HintSelector {
        fn select<'t>(
            &mut self,
            hint: ImageHint,
            table: &'t [&'t dyn Target],
        ) -> Result<&'t dyn Target, SelectError> {
            self.seen.set(hint.uf2_family);
            table.first().copied().ok_or(SelectError::NoTarget)
        }
    }

    /// The dropped image's family reaches the selector as the hint; a plain `begin` hands it `None`.
    #[test]
    fn begin_with_hint_plumbs_family_to_selector() {
        let live: &'static Cell<i32> = Box::leak(Box::new(Cell::new(0)));
        let seen: &'static Cell<Option<u32>> = Box::leak(Box::new(Cell::new(None)));
        let target = MockTarget;
        let table: [&dyn Target; 1] = [&target];
        let mut buf = [0u8; 256];
        let mut be: HwBackend<
            '_,
            MockSwd,
            MockDelay,
            MockTarget,
            DefaultCallbacks,
            MockProbe,
            HintSelector,
            CountingArbiter,
            MockSwd,
        > = HwBackend::new(
            MockSwd::new(false),
            MockDelay,
            DefaultCallbacks,
            MockProbe { fail_reset: false },
            HintSelector { seen },
            &target,
            &table,
            &mut buf,
            CountingArbiter { live },
            true,
        );

        be.begin_with_hint(Some(0x1111_1111)).expect("begin");
        assert_eq!(
            seen.get(),
            Some(0x1111_1111),
            "the noted UF2 family reaches the selector as the hint"
        );
        be.finish(true).expect("finish");

        be.begin().expect("begin");
        assert_eq!(seen.get(), None, "plain begin passes an empty hint");
        be.finish(true).expect("finish");
    }

    /// Selector that rejects every selection, standing in for a wrong-chip cross-check.
    #[derive(Clone, Copy, Default)]
    struct RejectSelector;
    impl TargetSelector for RejectSelector {
        fn select<'t>(
            &mut self,
            _hint: ImageHint,
            _table: &'t [&'t dyn Target],
        ) -> Result<&'t dyn Target, SelectError> {
            Err(SelectError::ImageMismatch)
        }
    }

    /// A rejected `begin` drops the just-claimed session, so DAP commands still run afterwards.
    #[test]
    fn wrong_chip_rejection_releases_session() {
        let live: &'static Cell<i32> = Box::leak(Box::new(Cell::new(0)));
        let target = MockTarget;
        let table: [&dyn Target; 1] = [&target];
        let mut buf = [0u8; 256];
        let mut be: HwBackend<
            '_,
            MockSwd,
            MockDelay,
            MockTarget,
            DefaultCallbacks,
            MockProbe,
            RejectSelector,
            CountingArbiter,
            MockSwd,
        > = HwBackend::new(
            MockSwd::new(false),
            MockDelay,
            DefaultCallbacks,
            MockProbe { fail_reset: false },
            RejectSelector,
            &target,
            &table,
            &mut buf,
            CountingArbiter { live },
            true,
        );

        assert_eq!(
            be.begin_with_hint(Some(0x9999_9999)),
            Err(FlashPhase::Family)
        );
        assert_eq!(live.get(), 0, "the rejected begin must release the session");

        // The abort path then calls finish(false) with no manager open: a clean no-op.
        assert_eq!(be.finish(false), Ok(()));
        assert_eq!(
            live.get(),
            0,
            "finish after a rejected begin holds no session"
        );
    }

    /// Selector that always returns the second table row.
    #[derive(Clone, Copy, Default)]
    struct SecondSelector;
    impl TargetSelector for SecondSelector {
        fn select<'t>(
            &mut self,
            _hint: ImageHint,
            table: &'t [&'t dyn Target],
        ) -> Result<&'t dyn Target, SelectError> {
            table.get(1).copied().ok_or(SelectError::NoTarget)
        }
    }

    /// Inert delay.
    struct MockDelay;
    impl DelayNs for MockDelay {
        fn delay_ns(&mut self, _ns: u32) {}
    }

    /// Arbiter whose sessions increment a counter while live, so a test can assert balance.
    struct CountingArbiter {
        live: &'static Cell<i32>,
    }
    struct CountingSession {
        live: &'static Cell<i32>,
    }
    impl FlashArbiter for CountingArbiter {
        type Session = CountingSession;
        fn begin_session(&self) -> Option<CountingSession> {
            self.live.set(self.live.get() + 1);
            Some(CountingSession { live: self.live })
        }
    }
    impl Drop for CountingSession {
        fn drop(&mut self) {
            self.live.set(self.live.get() - 1);
        }
    }

    type Backend<'a> = HwBackend<
        'a,
        MockSwd,
        MockDelay,
        MockTarget,
        DefaultCallbacks,
        MockProbe,
        FixedTarget,
        CountingArbiter,
        MockSwd,
    >;

    #[allow(clippy::too_many_arguments)]
    fn backend<'a>(
        fail_init: bool,
        fail_reset: bool,
        auto_reset: bool,
        target: &'a MockTarget,
        table: &'a [&'a dyn Target],
        buffer: &'a mut [u8],
        live: &'static Cell<i32>,
    ) -> Backend<'a> {
        HwBackend::new(
            MockSwd::new(fail_init),
            MockDelay,
            DefaultCallbacks,
            MockProbe { fail_reset },
            FixedTarget,
            target,
            table,
            buffer,
            CountingArbiter { live },
            auto_reset,
        )
    }

    /// Two full cycles: the second can only run because the first handed the parts back.
    #[test]
    fn repeatable_begin_program_finish() {
        let live: &'static Cell<i32> = Box::leak(Box::new(Cell::new(0)));
        let target = MockTarget;
        let table: [&dyn Target; 1] = [&target];
        let mut buf = [0u8; 256];
        let mut be = backend(false, false, true, &target, &table, &mut buf, live);

        for _ in 0..2 {
            be.begin().expect("begin");
            assert_eq!(live.get(), 1, "session live during the cycle");
            be.program(0x0800_0000, &[0xAA; 8]).expect("program");
            be.finish(true).expect("finish");
            assert_eq!(live.get(), 0, "session released after the cycle");
        }
    }

    /// The presence gate reaches the probe handle without opening a session.
    #[test]
    fn read_dpidr_reaches_the_probe_handle() {
        let live: &'static Cell<i32> = Box::leak(Box::new(Cell::new(0)));
        let target = MockTarget;
        let table: [&dyn Target; 1] = [&target];
        let mut buf = [0u8; 256];
        let mut be = backend(false, false, true, &target, &table, &mut buf, live);

        assert_eq!(be.read_dpidr(), Some(0x0BC1_1477));
        assert_eq!(live.get(), 0, "the presence gate opens no session");
    }

    /// A session that programs nothing still releases the target `begin` halted.
    #[test]
    fn session_that_programs_nothing_releases_the_target() {
        for success in [true, false] {
            let live: &'static Cell<i32> = Box::leak(Box::new(Cell::new(0)));
            let released: &'static Cell<u32> = Box::leak(Box::new(Cell::new(0)));
            let target = MockTarget;
            let table: [&dyn Target; 1] = [&target];
            let mut buf = [0u8; 256];
            let mut be: Backend<'_> = HwBackend::new(
                MockSwd::new(false).counting_releases(released),
                MockDelay,
                DefaultCallbacks,
                MockProbe { fail_reset: false },
                FixedTarget,
                &target,
                &table,
                &mut buf,
                CountingArbiter { live },
                true,
            );

            be.begin().expect("begin");
            assert_eq!(released.get(), 0, "the core stays halted for the session");
            be.finish(success).expect("finish");
            assert_eq!(
                released.get(),
                1,
                "a session that programmed nothing still releases the core"
            );
            assert_eq!(live.get(), 0, "and holds no arbiter session afterwards");
        }
    }

    /// The live reset policy overrides the composed default, leaving the core halted.
    #[test]
    fn live_reset_policy_overrides_the_composed_default() {
        for reset in [false, true] {
            let live: &'static Cell<i32> = Box::leak(Box::new(Cell::new(0)));
            let released: &'static Cell<u32> = Box::leak(Box::new(Cell::new(0)));
            let target = MockTarget;
            let table: [&dyn Target; 1] = [&target];
            let mut buf = [0u8; 256];
            let mut be: Backend<'_> = HwBackend::new(
                MockSwd::new(false).counting_releases(released),
                MockDelay,
                DefaultCallbacks,
                MockProbe { fail_reset: false },
                FixedTarget,
                &target,
                &table,
                &mut buf,
                CountingArbiter { live },
                true,
            );

            be.begin().expect("begin");
            be.program(0x0800_0000, &[0xCC; 8]).expect("program");
            be.finish_with_reset(true, reset).expect("finish");
            assert_eq!(
                released.get(),
                u32::from(reset),
                "the target is released only when the live policy asks for it"
            );
            assert_eq!(live.get(), 0, "and the session is released either way");
        }
    }

    /// A failed transfer still releases the target, whatever the reset argument says.
    #[test]
    fn failed_transfer_ignores_the_reset_argument_and_releases() {
        let live: &'static Cell<i32> = Box::leak(Box::new(Cell::new(0)));
        let released: &'static Cell<u32> = Box::leak(Box::new(Cell::new(0)));
        let target = MockTarget;
        let table: [&dyn Target; 1] = [&target];
        let mut buf = [0u8; 256];
        let mut be: Backend<'_> = HwBackend::new(
            MockSwd::new(false).counting_releases(released),
            MockDelay,
            DefaultCallbacks,
            MockProbe { fail_reset: false },
            FixedTarget,
            &target,
            &table,
            &mut buf,
            CountingArbiter { live },
            true,
        );

        be.begin().expect("begin");
        be.finish_with_reset(false, false).expect("abort");
        assert_eq!(released.get(), 1, "an aborted session still releases");
        assert_eq!(live.get(), 0);
    }

    /// A failed connect releases the session, and the next `begin` succeeds.
    #[test]
    fn failed_init_then_retry_succeeds() {
        let live: &'static Cell<i32> = Box::leak(Box::new(Cell::new(0)));
        let target = MockTarget;
        let table: [&dyn Target; 1] = [&target];
        let mut buf = [0u8; 256];
        let mut be = backend(true, false, false, &target, &table, &mut buf, live);

        assert_eq!(be.begin(), Err(FlashPhase::Init));
        assert_eq!(live.get(), 0, "failed begin releases the session");

        be.begin().expect("retry begin");
        assert_eq!(live.get(), 1);
        be.finish(true).expect("finish");
        assert_eq!(live.get(), 0);
    }

    /// An aborted session hands the parts back and can be reused.
    #[test]
    fn abort_then_reuse() {
        let live: &'static Cell<i32> = Box::leak(Box::new(Cell::new(0)));
        let target = MockTarget;
        let table: [&dyn Target; 1] = [&target];
        let mut buf = [0u8; 256];
        let mut be = backend(false, false, false, &target, &table, &mut buf, live);

        be.begin().expect("begin");
        be.program(0x0800_0000, &[0x11; 4]).expect("program");
        be.finish(false).expect("abort");
        assert_eq!(live.get(), 0, "abort releases the session");

        be.begin().expect("reuse begin");
        assert_eq!(live.get(), 1);
        be.finish(true).expect("finish");
        assert_eq!(live.get(), 0);
    }

    /// A failing post-program reset is non-fatal: `finish(true)` still reports success.
    #[test]
    fn auto_reset_failure_is_non_fatal() {
        let live: &'static Cell<i32> = Box::leak(Box::new(Cell::new(0)));
        let target = MockTarget;
        let table: [&dyn Target; 1] = [&target];
        let mut buf = [0u8; 256];
        let mut be: Backend<'_> = HwBackend::new(
            MockSwd::new(false).with_fail_reset(),
            MockDelay,
            DefaultCallbacks,
            MockProbe { fail_reset: false },
            FixedTarget,
            &target,
            &table,
            &mut buf,
            CountingArbiter { live },
            true,
        );

        be.begin().expect("begin");
        be.program(0x0800_0000, &[0x22; 4]).expect("program");
        assert_eq!(
            be.finish(true),
            Ok(()),
            "reset failure must not fail finish"
        );
        assert_eq!(live.get(), 0);
    }

    /// A programming error on the final flush must not brick the backend for later sessions.
    #[test]
    fn failed_final_flush_then_reuse_succeeds() {
        let live: &'static Cell<i32> = Box::leak(Box::new(Cell::new(0)));
        let target = MockTarget;
        let table: [&dyn Target; 1] = [&target];
        let mut buf = [0u8; 256];
        let mut be: Backend<'_> = HwBackend::new(
            MockSwd::new(false).with_fail_program(),
            MockDelay,
            DefaultCallbacks,
            MockProbe { fail_reset: false },
            FixedTarget,
            &target,
            &table,
            &mut buf,
            CountingArbiter { live },
            true,
        );

        be.begin().expect("begin");
        be.program(0x0800_0000, &[0xAA; 8]).expect("program");
        assert_eq!(
            be.finish(true),
            Err(FlashPhase::Program),
            "failed final flush reports a program-phase error"
        );
        assert_eq!(live.get(), 0, "failed finish still releases the session");

        be.begin().expect("reuse begin");
        assert_eq!(live.get(), 1);
        be.program(0x0800_0000, &[0xBB; 8]).expect("reuse program");
        be.finish(true).expect("reuse finish");
        assert_eq!(live.get(), 0);
    }

    /// A wrong-part image and a failed write have opposite remedies, so they get distinct phases.
    #[test]
    fn an_unmapped_address_and_a_failed_write_report_different_phases() {
        let live: &'static Cell<i32> = Box::leak(Box::new(Cell::new(0)));
        let target = MockTarget;
        let table: [&dyn Target; 1] = [&target];
        let mut buf = [0u8; 256];
        let mut be = backend(false, false, true, &target, &table, &mut buf, live);
        be.begin().expect("begin");
        // 0x0900_0000 is outside this target's only region.
        be.program(0x0900_0000, &[0xAA; 8]).expect("staged");
        let unmapped = be.finish(true);
        assert_eq!(live.get(), 0);

        let live2: &'static Cell<i32> = Box::leak(Box::new(Cell::new(0)));
        let target2 = MockTarget;
        let table2: [&dyn Target; 1] = [&target2];
        let mut buf2 = [0u8; 256];
        let mut be2: Backend<'_> = HwBackend::new(
            MockSwd::new(false).with_fail_program(),
            MockDelay,
            DefaultCallbacks,
            MockProbe { fail_reset: false },
            FixedTarget,
            &target2,
            &table2,
            &mut buf2,
            CountingArbiter { live: live2 },
            true,
        );
        be2.begin().expect("begin");
        be2.program(0x0800_0000, &[0xAA; 8]).expect("staged");
        let wire_failure = be2.finish(true);
        assert_eq!(
            wire_failure,
            Err(FlashPhase::Program),
            "a write that reached the wire and failed stays a Program failure"
        );
        assert!(unmapped.is_err(), "an unmapped address must still fail");
        assert_ne!(
            unmapped, wire_failure,
            "an unmapped address must not report as a failed write"
        );
    }

    /// A page that programs cleanly and reads back wrong fails the transfer, at both flush sites.
    #[test]
    fn verify_mismatch_fails_the_transfer() {
        for fill_a_page in [false, true] {
            let live: &'static Cell<i32> = Box::leak(Box::new(Cell::new(0)));
            let target = MockTarget;
            let table: [&dyn Target; 1] = [&target];
            let mut buf = [0u8; 256];
            let mut be: Backend<'_> = HwBackend::new(
                MockSwd::new(false).with_fail_verify(),
                MockDelay,
                DefaultCallbacks,
                MockProbe { fail_reset: false },
                FixedTarget,
                &target,
                &table,
                &mut buf,
                CountingArbiter { live },
                true,
            );

            be.begin().expect("begin");
            if fill_a_page {
                // 256 bytes fills the staging buffer, so the flush happens inside `program`.
                assert_eq!(
                    be.program(0x0800_0000, &[0xAA; 256]),
                    Err(FlashPhase::Verify),
                    "a page-filling write reports the mismatch from program"
                );
                assert_eq!(be.finish(false), Ok(()), "the sink then aborts");
            } else {
                be.program(0x0800_0000, &[0xAA; 8]).expect("program");
                assert_eq!(
                    be.finish(true),
                    Err(FlashPhase::Verify),
                    "the final flush reports the mismatch from finish"
                );
            }
            assert_eq!(live.get(), 0, "a failed verify still releases the session");

            be.begin().expect("reuse begin");
            be.program(0x0800_0000, &[0xAA; 8]).expect("reuse program");
            be.finish(true).expect("reuse finish");
            assert_eq!(live.get(), 0);
        }
    }

    /// A target whose algorithm exports no `Verify` is verified by read-back, and fails the same.
    #[test]
    fn readback_verify_covers_a_target_without_a_verify_entry() {
        let live: &'static Cell<i32> = Box::leak(Box::new(Cell::new(0)));
        let target = ReadbackTarget;
        let table: [&dyn Target; 1] = [&target];
        let mut buf = [0u8; 256];
        let mut be: HwBackend<
            '_,
            MockSwd,
            MockDelay,
            ReadbackTarget,
            DefaultCallbacks,
            MockProbe,
            FixedTarget,
            CountingArbiter,
            MockSwd,
        > = HwBackend::new(
            MockSwd::new(false),
            MockDelay,
            DefaultCallbacks,
            MockProbe { fail_reset: false },
            FixedTarget,
            &target,
            &table,
            &mut buf,
            CountingArbiter { live },
            true,
        );

        be.begin().expect("begin");
        be.program(0x0900_0000, &[0xFF; 8]).expect("program");
        be.finish(true)
            .expect("a page that reads back as written passes the read-back verify");

        be.begin().expect("begin");
        be.program(0x0900_0000, &[0xAA; 8]).expect("program");
        assert_eq!(
            be.finish(true),
            Err(FlashPhase::Verify),
            "a page that reads back wrong fails the transfer"
        );
        assert_eq!(live.get(), 0);
    }

    /// The window seams narrow from the composed union to the selected target, RAM included.
    #[test]
    fn window_seams_narrow_from_the_composed_union_to_the_selected_target() {
        let live: &'static Cell<i32> = Box::leak(Box::new(Cell::new(0)));
        let composed = MockTarget;
        let second = SecondTarget;
        let table: [&dyn Target; 2] = [&composed, &second];
        let mut buf = [0u8; 256];
        let mut be: HwBackend<
            '_,
            MockSwd,
            MockDelay,
            MockTarget,
            DefaultCallbacks,
            MockProbe,
            SecondSelector,
            CountingArbiter,
            MockSwd,
        > = HwBackend::new(
            MockSwd::new(false),
            MockDelay,
            DefaultCallbacks,
            MockProbe { fail_reset: false },
            SecondSelector,
            &composed,
            &table,
            &mut buf,
            CountingArbiter { live },
            true,
        );

        assert_eq!(
            be.candidate_windows(),
            &[
                TargetWindows {
                    flash: (0x0800_0000, 0x0800_0400),
                    ram: (0x2000_0000, 0x2000_2000),
                },
                TargetWindows {
                    flash: (0x1000_0000, 0x1000_0400),
                    ram: (0x2400_0000, 0x2408_0000),
                },
            ]
        );
        assert!(be.ram_ranges().is_empty(), "no target resolved yet");

        be.begin().expect("begin");
        assert_eq!(be.flash_ranges(), &[(0x1000_0000, 0x1000_0400)]);
        assert_eq!(be.ram_ranges(), &[(0x2400_0000, 0x2408_0000)]);
        assert_eq!(
            be.candidate_windows().len(),
            2,
            "the composed set is a fact about the firmware, not the session"
        );
        be.finish(true).expect("finish");
    }

    /// The session programs the target the selector returns, not the composed one.
    #[test]
    fn selector_second_row_is_programmed() {
        let live: &'static Cell<i32> = Box::leak(Box::new(Cell::new(0)));
        let composed = MockTarget;
        let second = SecondTarget;
        let table: [&dyn Target; 2] = [&composed, &second];
        let mut buf = [0u8; 256];

        let mut be: HwBackend<
            '_,
            MockSwd,
            MockDelay,
            MockTarget,
            DefaultCallbacks,
            MockProbe,
            SecondSelector,
            CountingArbiter,
            MockSwd,
        > = HwBackend::new(
            MockSwd::new(false),
            MockDelay,
            DefaultCallbacks,
            MockProbe { fail_reset: false },
            SecondSelector,
            &composed,
            &table,
            &mut buf,
            CountingArbiter { live },
            true,
        );
        assert_eq!(
            be.expected_uf2_family(),
            Some(0x1111_1111),
            "before begin: falls back to the composed target's family"
        );

        be.begin().expect("begin");
        assert_eq!(
            be.flash_ranges(),
            &[(0x1000_0000, 0x1000_0400)],
            "ranges follow the selected (second) target"
        );
        assert_eq!(
            be.expected_uf2_family(),
            Some(0x2222_2222),
            "expected family follows the selected (second) target"
        );

        be.program(0x1000_0000, &[0xAA; 8])
            .expect("program at the selected target's base");
        be.finish(true).expect("finish");
        assert_eq!(live.get(), 0);
    }

    /// A mass-erase drives `EraseChip` and releases both target and session, opening no program run.
    #[test]
    fn erase_all_erases_the_chip_and_releases_the_session() {
        let live: &'static Cell<i32> = Box::leak(Box::new(Cell::new(0)));
        let released: &'static Cell<u32> = Box::leak(Box::new(Cell::new(0)));
        let target = MockTarget;
        let table: [&dyn Target; 1] = [&target];
        let mut buf = [0u8; 256];
        let mut be: Backend<'_> = HwBackend::new(
            MockSwd::new(false).counting_releases(released),
            MockDelay,
            DefaultCallbacks,
            MockProbe { fail_reset: false },
            FixedTarget,
            &target,
            &table,
            &mut buf,
            CountingArbiter { live },
            true,
        );

        assert_eq!(be.erase_all(), Ok(()));
        assert_eq!(
            released.get(),
            1,
            "the mass-erase releases the target to run"
        );
        assert_eq!(live.get(), 0, "and holds no arbiter session afterwards");

        be.begin().expect("begin after erase_all");
        be.program(0x0800_0000, &[0xAA; 4]).expect("program");
        be.finish(true).expect("finish");
        assert_eq!(live.get(), 0);
    }

    /// A chip-erase that faults is reported in the `Erase` phase and still releases the session.
    #[test]
    fn erase_all_reports_erase_phase_on_algorithm_failure() {
        let live: &'static Cell<i32> = Box::leak(Box::new(Cell::new(0)));
        let target = MockTarget;
        let table: [&dyn Target; 1] = [&target];
        let mut buf = [0u8; 256];
        let mut be: Backend<'_> = HwBackend::new(
            MockSwd::new(false).with_fail_erase(),
            MockDelay,
            DefaultCallbacks,
            MockProbe { fail_reset: false },
            FixedTarget,
            &target,
            &table,
            &mut buf,
            CountingArbiter { live },
            true,
        );

        assert_eq!(be.erase_all(), Err(FlashPhase::Erase));
        assert_eq!(
            live.get(),
            0,
            "a failed mass-erase still releases the session"
        );

        be.begin().expect("begin after failed erase_all");
        be.finish(true).expect("finish");
        assert_eq!(live.get(), 0);
    }

    /// `erase_all` releases the session on a selector rejection exactly as `begin` does.
    #[test]
    fn erase_all_wrong_chip_rejection_releases_session() {
        let live: &'static Cell<i32> = Box::leak(Box::new(Cell::new(0)));
        let target = MockTarget;
        let table: [&dyn Target; 1] = [&target];
        let mut buf = [0u8; 256];
        let mut be: HwBackend<
            '_,
            MockSwd,
            MockDelay,
            MockTarget,
            DefaultCallbacks,
            MockProbe,
            RejectSelector,
            CountingArbiter,
            MockSwd,
        > = HwBackend::new(
            MockSwd::new(false),
            MockDelay,
            DefaultCallbacks,
            MockProbe { fail_reset: false },
            RejectSelector,
            &target,
            &table,
            &mut buf,
            CountingArbiter { live },
            true,
        );

        assert_eq!(be.erase_all(), Err(FlashPhase::Family));
        assert_eq!(
            live.get(),
            0,
            "the rejected erase_all must release the session"
        );
    }
}
