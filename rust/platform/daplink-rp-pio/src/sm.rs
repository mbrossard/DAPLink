//! The [`PioShifter`] mechanism over raw PIO registers, with no runtime or HAL in the path.

use daplink_rs::dap_rs::swj::Pins;
use daplink_rs::{spin_cycles, ScanChainStorage};
use rp_pac as pac;

use crate::clock::{
    clock_divider, jtag_clock_divider, swd_cycles_per_bit, PioClockDivider, JTAG_MIN_DIVIDER,
    MIN_DIVIDER,
};
use crate::generic_roles::PioSwdDeps;
use crate::jtag::PioJtagShifter;
use crate::pins::{JtagPinMap, PinMap, PioJtagPins, PioSwdPins};
use crate::program::{select_jtag_program, select_program, JtagProgramEntries};
use crate::regs::{self, FUNCSEL_PIO0, FUNCSEL_PIO1, FUNCSEL_SIO, GPIO_COUNT};
use crate::shifter::{start_idle, PioShifter, SwjPinsSio};
use crate::swd_transaction_hybrid::PioSwdTransactionDeps;
use crate::transaction_sm::{RpPioSwdTransaction, RpPioSwdTransactionFence};
#[cfg(any(feature = "rp2040", feature = "rp235x"))]
use crate::{PioTransferDeps, RpPioTransferEngine, RpPioTransferFence};

const DEFAULT_SWCLK_HZ: u32 = 1_000_000;

/// SWD and JTAG use separate state machines in the chosen block.
const SWD_SM: usize = 0;
const JTAG_SM: usize = 1;

/// Which PIO block drives the wire.
#[derive(Copy, Clone, PartialEq, Eq)]
pub enum PioBlock {
    /// PIO0.
    Pio0,
    /// PIO1.
    Pio1,
}

/// The raw-register state machine: the mechanism half of the PIO SWD backend.
pub struct PioSm {
    pio: pac::pio::Pio,
    funcsel: u8,
    swclk: u8,
    swdio_out: u8,
    swdio_in: u8,
    oen: Option<u8>,
    nreset: Option<u8>,
    jtag: Option<PioJtagPins>,
    sys_hz: u32,
}

/// Bring up the PIO SWD backend and hand back the DAP dependencies over it.
///
/// # Safety
/// Own the whole selected PIO block, including instruction memory and global configuration, and every pad in `pins`.
pub(crate) unsafe fn pio_swd_deps<const N: usize>(
    block: PioBlock,
    pins: PioSwdPins,
    sys_hz: u32,
    scan: &'static ScanChainStorage<N>,
) -> PioSwdDeps<PioSm> {
    let (sm, entries) = PioSm::new(block, pins, sys_hz);
    PioSwdDeps::new(sm, entries, sys_hz, scan)
}

/// Bring up generic PIO0 SWJ beside the PIO1 SWD accelerator; attach the fence with `DapProbe::with_command_fence`.
///
/// # Safety
/// Own the whole PIO0 and PIO1 blocks and every pad in `pins`, with no other raw PIO1 engine alive.
pub unsafe fn pio_swd_transaction_deps<const N: usize>(
    pins: PioSwdPins,
    sys_hz: u32,
    scan: &'static ScanChainStorage<N>,
) -> (
    PioSwdTransactionDeps<PioSm, RpPioSwdTransaction>,
    RpPioSwdTransactionFence,
) {
    let generic = pio_swd_deps(PioBlock::Pio0, pins, sys_hz, scan);
    let (accelerator, fence) = RpPioSwdTransaction::new(pins, sys_hz);
    (PioSwdTransactionDeps::new(generic, accelerator), fence)
}

/// Bring up the paired PIO SWD/JTAG backend.
///
/// # Safety
/// Own the whole selected PIO block and every pad in both pin maps.
pub(crate) unsafe fn pio_swd_jtag_deps<const N: usize>(
    block: PioBlock,
    swd: PioSwdPins,
    jtag: PioJtagPins,
    sys_hz: u32,
    scan: &'static ScanChainStorage<N>,
) -> PioSwdDeps<PioSm> {
    let (sm, swd_entries, jtag_entries) = PioSm::new_jtag(block, swd, jtag, sys_hz);
    PioSwdDeps::new_jtag(sm, swd_entries, jtag_entries, sys_hz, scan)
}

/// Bring up generic PIO0 SWJ beside the shared PIO1 SWD/JTAG engine; install the fence on DAP and flash backends.
///
/// # Safety
/// Own PIO0 and PIO1, PIO1's IRQ flags 0 and 1, every pad in both maps, and the PIO1 mode state.
#[cfg(any(feature = "rp2040", feature = "rp235x"))]
pub unsafe fn pio_transfer_deps<const N: usize>(
    swd: PioSwdPins,
    jtag: PioJtagPins,
    sys_hz: u32,
    scan: &'static ScanChainStorage<N>,
) -> (
    PioTransferDeps<PioSm, RpPioTransferEngine>,
    RpPioTransferFence,
) {
    assert!(jtag.valid_with_gpio_count(&swd, GPIO_COUNT));
    let generic = pio_swd_jtag_deps(PioBlock::Pio0, swd, jtag, sys_hz, scan);
    let (accelerator, fence) = RpPioTransferEngine::new(swd, jtag, sys_hz);
    (PioSwdTransactionDeps::new(generic, accelerator), fence)
}

impl PioSm {
    /// Load the program, configure the state machine, park it at the dispatcher and enable it.
    ///
    /// # Safety
    /// Own the whole PIO block and every pad in `pins`.
    pub unsafe fn new(
        block: PioBlock,
        pins: PioSwdPins,
        sys_hz: u32,
    ) -> (Self, crate::program::ProgramEntries) {
        let (sm, entries, _) = Self::new_inner(block, pins, None, sys_hz);
        (sm, entries)
    }

    /// Load paired SWD/JTAG programs and configure state machines 0 and 1.
    ///
    /// # Safety
    /// Own the whole PIO block and every named pad.
    pub unsafe fn new_jtag(
        block: PioBlock,
        swd: PioSwdPins,
        jtag: PioJtagPins,
        sys_hz: u32,
    ) -> (Self, crate::program::ProgramEntries, JtagProgramEntries) {
        let (sm, swd_entries, jtag_entries) = Self::new_inner(block, swd, Some(jtag), sys_hz);
        (
            sm,
            swd_entries,
            jtag_entries.expect("JTAG program selected"),
        )
    }

    unsafe fn new_inner(
        block: PioBlock,
        pins: PioSwdPins,
        jtag: Option<PioJtagPins>,
        sys_hz: u32,
    ) -> (
        Self,
        crate::program::ProgramEntries,
        Option<JtagProgramEntries>,
    ) {
        let (pio, funcsel) = match block {
            PioBlock::Pio0 => (pac::PIO0, FUNCSEL_PIO0),
            PioBlock::Pio1 => (pac::PIO1, FUNCSEL_PIO1),
        };
        regs::unreset(block);

        let mut sm = Self {
            pio,
            funcsel,
            swclk: pins.swclk,
            swdio_out: pins.swdio_out,
            swdio_in: pins.readback(),
            oen: pins.oen,
            nreset: pins.nreset,
            jtag,
            sys_hz,
        };

        let prog = select_program(sm.oen.is_some(), sys_hz).expect("unsupported PIO system clock");
        let map = PinMap::new(&pins, prog.side_set_bits);
        let jtag_map = sm.jtag.map(|jtag| JtagPinMap::new(&pins, &jtag));
        let jtag_prog = jtag_map
            .map(|_| select_jtag_program(sys_hz).expect("unsupported PIO JTAG system clock"));
        // The block is ours, so the program loads at offset 0 and the entries are its labels.
        let entries = prog.entries(0);
        sm.set_enabled(false);
        regs::load(pio, 0, prog.instructions);
        if let Some(jtag_prog) = jtag_prog {
            regs::load(pio, jtag_prog.load_offset as usize, jtag_prog.instructions);
        }
        regs::gpio_base_zero(pio);

        let regs = pio.sm(SWD_SM);
        regs.execctrl().modify(|w| {
            w.set_wrap_bottom(prog.wrap_target);
            w.set_wrap_top(prog.wrap);
            w.set_side_en(prog.side_set_opt);
            w.set_side_pindir(false);
        });
        regs.shiftctrl().modify(|w| {
            // LSB first both ways; auto refill is off because every data word has a command word.
            w.set_in_shiftdir(true);
            w.set_out_shiftdir(true);
            w.set_autopush(false);
            w.set_autopull(false);
            // A threshold of 32 encodes as 0.
            w.set_push_thresh(0);
            w.set_pull_thresh(0);
        });
        regs.pinctrl().write(|w| {
            // A split wiring differs only here: `IN` on the readback pad, `OUT`/`SET` on the driven one.
            w.set_out_base(map.out_base);
            w.set_out_count(1);
            w.set_set_base(map.set_base);
            w.set_set_count(1);
            w.set_in_base(map.in_base);
            w.set_sideset_base(map.sideset_base);
            w.set_sideset_count(map.sideset_count);
        });

        if let (Some(map), Some(jtag_prog)) = (jtag_map, jtag_prog) {
            // The late-sample cell reads TDO directly during the stable high phase.
            let bypass = pio.input_sync_bypass().read();
            pio.input_sync_bypass()
                .write_value(bypass | (1 << map.in_base));
            let regs = pio.sm(JTAG_SM);
            regs.execctrl().modify(|w| {
                w.set_wrap_bottom(jtag_prog.wrap_target);
                w.set_wrap_top(jtag_prog.wrap);
                w.set_side_en(true);
                w.set_side_pindir(false);
            });
            regs.shiftctrl().modify(|w| {
                w.set_in_shiftdir(true);
                w.set_out_shiftdir(true);
                w.set_autopush(false);
                w.set_autopull(false);
                w.set_push_thresh(0);
                w.set_pull_thresh(0);
            });
            regs.pinctrl().write(|w| {
                w.set_out_base(map.out_base);
                w.set_out_count(1);
                w.set_set_base(map.out_base);
                w.set_set_count(1);
                w.set_in_base(map.in_base);
                w.set_sideset_base(map.sideset_base);
                w.set_sideset_count(map.sideset_count);
            });
        }

        let cycles = swd_cycles_per_bit(sys_hz).expect("unsupported PIO system clock");
        let div = clock_divider(sys_hz, DEFAULT_SWCLK_HZ, cycles, MIN_DIVIDER)
            .expect("default SWD clock is representable");
        sm.set_clock_divider(div);
        if jtag_map.is_some() {
            let jtag_div = jtag_clock_divider(sys_hz, DEFAULT_SWCLK_HZ)
                .unwrap_or(PioClockDivider::integer(JTAG_MIN_DIVIDER));
            sm.set_jtag_clock_divider(jtag_div);
        }

        // Claim the pads for PIO, then nRESET for SIO; nRESET is never PIO-owned.
        claim_pad(sm.swclk, funcsel);
        claim_pad(sm.swdio_out, funcsel);
        if sm.swdio_in != sm.swdio_out {
            claim_pad(sm.swdio_in, funcsel);
        }
        if let Some(oe) = sm.oen {
            claim_pad(oe, funcsel);
        }
        if let Some(nrst) = sm.nreset {
            claim_pad(nrst, FUNCSEL_SIO);
            // Open drain: the output register stays low, so asserting is enabling the driver.
            sio_drive_level(nrst, false);
            sio_set_oe(nrst, false);
        }
        if let Some(jtag) = sm.jtag {
            claim_pad(jtag.tdi, funcsel);
            claim_pad(jtag.tdo, funcsel);
        }

        start_idle(&mut sm, entries.get_next_cmd);
        (sm, entries, jtag_prog.map(|program| program.entries))
    }

    /// One state machine's bit in the block-wide control masks.
    const fn mask(index: usize) -> u8 {
        1 << index
    }
}

impl PioShifter for PioSm {
    fn put(&mut self, word: u32) {
        while self.pio.fstat().read().txfull() & Self::mask(SWD_SM) != 0 {}
        self.pio.txf(SWD_SM).write_value(word);
    }

    fn get(&mut self) -> u32 {
        while self.pio.fstat().read().rxempty() & Self::mask(SWD_SM) != 0 {}
        self.pio.rxf(SWD_SM).read()
    }

    fn set_clock_divider(&mut self, divider: u16) {
        regs::set_divider(self.pio, SWD_SM, PioClockDivider::integer(divider));
    }

    fn set_enabled(&mut self, enabled: bool) {
        regs::set_enabled(self.pio, SWD_SM, enabled);
    }

    fn clear_fifos(&mut self) {
        regs::clear_fifos(self.pio, SWD_SM);
    }

    fn restart(&mut self) {
        regs::restart(self.pio, SWD_SM);
    }

    fn exec_jmp(&mut self, addr: u8) {
        // An unconditional `jmp` is the address alone; opcode and condition are both zero.
        regs::exec(self.pio, SWD_SM, addr as u16);
    }

    fn force_oen_released(&mut self) {
        if let Some(oe) = self.oen {
            regs::exec_set_pin(self.pio, SWD_SM, oe, true);
        }
    }

    fn force_swclk_low(&mut self) {
        regs::exec_set_pin(self.pio, SWD_SM, self.swclk, false);
    }

    fn force_pindirs_idle(&mut self) {
        regs::exec_set_pindir(self.pio, SWD_SM, self.swclk, true);
        regs::exec_set_pindir(self.pio, SWD_SM, self.swdio_out, false);
        if let Some(oe) = self.oen {
            regs::exec_set_pindir(self.pio, SWD_SM, oe, true);
        }
    }

    fn pads_to_sio(&mut self) {
        regs::set_funcsel(self.swclk, FUNCSEL_SIO);
        regs::set_funcsel(self.swdio_out, FUNCSEL_SIO);
    }

    fn pads_to_pio(&mut self) {
        regs::set_funcsel(self.swclk, self.funcsel);
        regs::set_funcsel(self.swdio_out, self.funcsel);
    }
}

impl PioJtagShifter for PioSm {
    fn jtag_put(&mut self, word: u32) {
        while self.pio.fstat().read().txfull() & Self::mask(JTAG_SM) != 0 {}
        self.pio.txf(JTAG_SM).write_value(word);
    }

    fn jtag_get(&mut self) -> u32 {
        while self.pio.fstat().read().rxempty() & Self::mask(JTAG_SM) != 0 {}
        self.pio.rxf(JTAG_SM).read()
    }

    fn set_jtag_clock_divider(&mut self, divider: PioClockDivider) {
        regs::set_divider(self.pio, JTAG_SM, divider);
    }

    fn enter_jtag(&mut self, dispatcher: u8) {
        let Some(jtag) = self.jtag else {
            return;
        };
        self.set_enabled(false);
        regs::set_enabled(self.pio, JTAG_SM, false);
        regs::clear_fifos(self.pio, JTAG_SM);
        regs::restart(self.pio, JTAG_SM);

        regs::exec_set_pin(self.pio, JTAG_SM, jtag.tck, false);
        regs::exec_set_pin(self.pio, JTAG_SM, jtag.tms, true);
        regs::exec_set_pin(self.pio, JTAG_SM, jtag.tdi, true);
        regs::exec_set_pindir(self.pio, JTAG_SM, jtag.tdo, false);
        regs::exec_set_pindir(self.pio, JTAG_SM, jtag.tdi, true);
        regs::exec_set_pindir(self.pio, JTAG_SM, jtag.tms, true);
        regs::exec_set_pindir(self.pio, JTAG_SM, jtag.tck, true);
        regs::exec(self.pio, JTAG_SM, dispatcher as u16);

        regs::set_enabled(self.pio, JTAG_SM, true);
    }

    fn exit_jtag(&mut self) {
        let Some(jtag) = self.jtag else {
            return;
        };
        regs::set_enabled(self.pio, JTAG_SM, false);
        regs::exec_set_pindir(self.pio, JTAG_SM, jtag.tck, false);
        regs::exec_set_pindir(self.pio, JTAG_SM, jtag.tms, false);
        regs::exec_set_pindir(self.pio, JTAG_SM, jtag.tdi, false);
        regs::exec_set_pindir(self.pio, JTAG_SM, jtag.tdo, false);
    }

    fn park_jtag(&mut self) {
        let Some(jtag) = self.jtag else {
            return;
        };
        regs::set_enabled(self.pio, JTAG_SM, false);

        // Both blocks drive the same idle levels while FUNCSEL changes, so TCK never floats.
        regs::exec_set_pin(self.pio, JTAG_SM, jtag.tck, false);
        regs::exec_set_pin(self.pio, JTAG_SM, jtag.tms, true);
        regs::exec_set_pin(self.pio, JTAG_SM, jtag.tdi, true);
        regs::exec_set_pindir(self.pio, JTAG_SM, jtag.tdo, false);
        regs::exec_set_pindir(self.pio, JTAG_SM, jtag.tdi, true);
        regs::exec_set_pindir(self.pio, JTAG_SM, jtag.tms, true);
        regs::exec_set_pindir(self.pio, JTAG_SM, jtag.tck, true);
    }
}

impl SwjPinsSio for PioSm {
    fn swj_pins(&mut self, output: Pins, mask: Pins, wait_us: u32) -> Pins {
        // SWCLK is a driven output: high only when requested, low at idle.
        sio_set_oe(self.swclk, true);
        sio_drive_level(
            self.swclk,
            mask.contains(Pins::SWCLK) && output.contains(Pins::SWCLK),
        );

        // SWDIO drives only when requested; otherwise the pull-up holds the bus.
        if mask.contains(Pins::SWDIO) {
            sio_set_oe(self.swdio_out, true);
            sio_drive_level(self.swdio_out, output.contains(Pins::SWDIO));
        } else {
            sio_set_oe(self.swdio_out, false);
        }

        if let Some(nrst) = self.nreset {
            if mask.contains(Pins::NRESET) {
                // Open drain: assert drives low, release goes high-Z for the target pull-up.
                sio_set_oe(nrst, !output.contains(Pins::NRESET));
            }
        }

        if wait_us != 0 {
            spin_cycles(wait_us.saturating_mul(self.sys_hz / 1_000_000));
        }

        let mut read = Pins::empty();
        read.set(Pins::SWCLK, sio_read(self.swclk));
        read.set(Pins::SWDIO, sio_read(self.swdio_in));
        read.set(Pins::NRESET, self.nreset.is_none_or(sio_read));
        // No nTRST line on this backend; report it released.
        read.set(Pins::NTRST, true);
        read
    }

    fn release_nreset(&mut self) {
        if let Some(nrst) = self.nreset {
            // Only the driver turns off; the latch stays low, so this can never assert.
            sio_set_oe(nrst, false);
        }
    }
}

/// Enable the pad's input buffer and route it to `funcsel`; a later handover moves the function select alone.
fn claim_pad(gpio: u8, funcsel: u8) {
    pac::PADS_BANK0.gpio(gpio as usize).modify(|w| {
        w.set_ie(true);
        w.set_od(false);
        // RP2350 pads come out of reset isolated.
        #[cfg(feature = "rp235x")]
        w.set_iso(false);
    });
    regs::set_funcsel(gpio, funcsel);
}

fn sio_set_oe(gpio: u8, output: bool) {
    let oe = pac::SIO.gpio_oe(0);
    if output {
        oe.value_set().write_value(1 << gpio);
    } else {
        oe.value_clr().write_value(1 << gpio);
    }
}

fn sio_drive_level(gpio: u8, high: bool) {
    let out = pac::SIO.gpio_out(0);
    if high {
        out.value_set().write_value(1 << gpio);
    } else {
        out.value_clr().write_value(1 << gpio);
    }
}

fn sio_read(gpio: u8) -> bool {
    (pac::SIO.gpio_in(0).read() >> gpio) & 1 != 0
}
