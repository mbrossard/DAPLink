//! High-speed CMSIS-DAP v2 probe for the NXP MCU-Link (LPC55S69) on RTIC.

#![no_std]
#![no_main]

// A flash build latches the fault into `RESET_STAMP` and resets; every other build halts.
#[cfg(all(feature = "flash", not(feature = "defmt")))]
daplink_vfs::fault_reset_handlers!();
#[cfg(all(not(feature = "flash"), not(feature = "defmt")))]
use panic_halt as _;
// `defmt` halts in every build, so the RTT-attached probe sees the panic message.
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

// One wire engine per image: `bitbang` wins over `accel`, and `flash` wins over both.
#[cfg(not(any(feature = "accel", feature = "bitbang")))]
compile_error!("enable `accel` (the default wire engine) or `bitbang`");

/// Millisecond wall clock; SysTick is the sole writer and readers take wrap-safe deltas.
static ELAPSED_MS: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

/// Reset-surviving probe state (boot counter, last transfer status) in `.uninit` RAM; the boot
/// ROM uses SRAM too, so a clobbered magic reads as a power-on and the counter restarts.
#[cfg(feature = "flash")]
#[link_section = ".uninit.RESET_STAMP"]
static mut RESET_STAMP: core::mem::MaybeUninit<daplink_vfs::ResetStamp> =
    core::mem::MaybeUninit::uninit();

/// 1 ms tick: advance the wall clock and pend the USB task; every drain stays in the USB pass.
#[cortex_m_rt::exception]
fn SysTick() {
    ELAPSED_MS.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
    cortex_m::peripheral::NVIC::pend(lpc55_hal::raw::Interrupt::USB1);
}

#[rtic::app(device = lpc55_hal::raw)]
mod app {
    use core::sync::atomic::{AtomicU32, Ordering};
    use cortex_m::peripheral::NVIC;
    use daplink_lpc55_hal::{
        observe_rx_errors, probe_parts, reboot_armed, swo_dma_service_irq, vcom_baud_state,
        vcom_rx_discards, vcom_rx_dma_errors, vcom_service_irq, vcom_tx_sent, vcom_tx_service_irq,
        Bus, DynCycleDelay, Fc3Swo, Lpc55Boot, Vcom, CPU_HZ,
    };

    #[cfg(any(feature = "bitbang", feature = "flash"))]
    use daplink_lpc55_hal::IoPin;
    #[cfg(all(feature = "accel", not(feature = "bitbang"), not(feature = "flash")))]
    use daplink_lpc55_hal::{masked_deps, nops, JtagPacing, MaskedPacing, MaskedSwdDap};
    use daplink_rs::daplink_core::{
        product_string, DapQueue, CDC_HS_PACKET_SIZE, DAP_HS_PACKET_SIZE,
    };
    use daplink_rs::{
        BootloaderEntry, RebootVendor, VcomBaudVendor, VcomCounters, VcomStatsVendor, VendorChain,
    };
    use daplink_usb_device::DapUsb;
    use lpc55_hal::raw::Interrupt;
    use static_cell::ConstStaticCell;

    #[cfg(not(feature = "flash"))]
    use daplink_rs::daplink_core::DAP_FW_VERSION;
    #[cfg(all(feature = "bitbang", not(feature = "flash")))]
    use daplink_rs::BitbangDap;

    use cortex_m::peripheral::syst::SystClkSource;
    #[cfg(feature = "flash")]
    use daplink_rs::flash_mux::{
        role_build_flash_backend, role_mux_dap_engine_with_vendor, RoleMuxDapEngine,
        RoleMuxFlashBackend,
    };
    #[cfg(feature = "flash")]
    use daplink_rs::{LateJtag, LateJtagDeps, LateSwdDrv, ScanChainStorage, SwdMux};
    #[cfg(feature = "flash-stm32f401")]
    use daplink_targets::targets::stm32f401ce::STM32F401CE;
    #[cfg(feature = "flash")]
    use daplink_vfs::{
        AllDecoders, FlashSink, FlashVolume, StaticCell, VendorFlash, VolumeStorage,
    };

    // The registry the flash spine selects from, derived from the rows below.
    #[cfg(feature = "flash")]
    daplink_targets::target_table! {
        umbrella = "flash",
        count = TARGET_COUNT,
        primary = PrimaryTarget,
        selector = Selector,
        scratch = SCRATCH,
        primary_ref = PRIMARY,
        table = flash_table,
        buffer = flash_scratch,
        rows = [
            ("flash-stm32f401", STM32F401CE, daplink_targets::targets::stm32f401ce::PROGRAM_BUFFER_SIZE),
        ],
    }

    /// Core cycles of fixed bit-cell work subtracted from every requested half-period delay.
    const SWD_FIXED_WORK_CYCLES: u32 = 0;

    /// Floor on the residual spin, in core cycles; zero lets a saturated request run flat out.
    const MIN_HALF_PERIOD_CYCLES: u32 = 0;

    /// Pipelined command-queue depth: eight 512-byte packets in flight.
    const DAP_QUEUE_DEPTH: usize = 8;

    /// Per-slot byte capacity of the command queue: one high-speed DAP v2 bulk packet.
    const DAP_QUEUE_SLOT: usize = DAP_HS_PACKET_SIZE as usize;

    /// This image's command queue: the depth the host is told about and the storage behind it.
    type Queue = DapQueue<DAP_QUEUE_DEPTH, DAP_QUEUE_SLOT>;

    /// The `DAP_Info(MaxPacketCount)` this image advertises.
    const DAP_PACKET_COUNT: u8 = 8;

    // Advertised count, ring depth and slot size are one fact; widening one alone fails the build.
    const _: () = {
        assert!(Queue::new().depth() == DAP_PACKET_COUNT as usize);
        assert!(Queue::new().packet_capacity() == DAP_HS_PACKET_SIZE as usize);
    };

    /// The flash build's mass-storage bulk packet size: the high-speed 512, matching DAP.
    const MSC_EP: usize = DAP_HS_PACKET_SIZE as usize;

    /// The VCOM CDC-ACM data packet size: the high-speed 512, matching DAP and MSC.
    const CDC_EP: usize = CDC_HS_PACKET_SIZE as usize;

    /// The VCOM stream-buffer capacity: four packets of staging for a host that returns late.
    const CDC_BUF: usize = 4 * CDC_EP;

    /// Renegotiate the chirp when the device settles at full speed, where its descriptors are
    /// illegal and the host will never address it.
    #[cfg(feature = "usb-hs-retry")]
    pub mod hs_retry {
        use core::sync::atomic::{AtomicU32, Ordering};

        /// Chirp renegotiations forced, and the last status word one was judged from.
        #[no_mangle]
        pub static PROBE_HS_RETRIES: AtomicU32 = AtomicU32::new(0);
        #[no_mangle]
        pub static PROBE_HS_STATUS: AtomicU32 = AtomicU32::new(0);

        /// Give the host this long to finish enumerating before judging the attempt failed.
        const SETTLE_MS: u32 = 500;
        /// How long to hold the pull-up down; hosts debounce a disconnect over 100-200 ms.
        const DOWN_MS: u32 = 300;
        /// A host that will never enumerate this device must not be retried forever.
        const MAX_RETRIES: u32 = 10;

        /// Write-one-to-clear status bits: SETUP and the connect/suspend/reset change flags;
        /// every write below masks them out rather than acknowledging an unseen event.
        const W1C: u32 = (1 << 8) | (1 << 24) | (1 << 25) | (1 << 26);
        /// The connect pull-up.
        const DCON: u32 = 1 << 16;

        static DOWN_SINCE: AtomicU32 = AtomicU32::new(0);
        static LAST_CHANGE: AtomicU32 = AtomicU32::new(0);

        /// One pass of the recovery, cheap enough to run from every USB interrupt.
        pub fn poll(now: u32) {
            let usb = unsafe { &*lpc55_hal::raw::USB1::ptr() };
            let stat = usb.devcmdstat.read().bits();
            PROBE_HS_STATUS.store(stat, Ordering::Relaxed);

            let down = DOWN_SINCE.load(Ordering::Relaxed);
            if down != 0 {
                if now.wrapping_sub(down) >= DOWN_MS {
                    usb.devcmdstat
                        .write(|w| unsafe { w.bits((stat & !W1C) | DCON) });
                    DOWN_SINCE.store(0, Ordering::Relaxed);
                    LAST_CHANGE.store(now, Ordering::Relaxed);
                }
                return;
            }

            // Only a bus-powered device that settled at full speed without an address is stuck.
            let vbus = stat & (1 << 28) != 0;
            let full_speed = (stat >> 22) & 0b11 == 0b01;
            let addressed = stat & 0x7f != 0;
            if !vbus || addressed || !full_speed {
                LAST_CHANGE.store(now, Ordering::Relaxed);
                return;
            }
            if now.wrapping_sub(LAST_CHANGE.load(Ordering::Relaxed)) < SETTLE_MS {
                return;
            }
            if PROBE_HS_RETRIES.load(Ordering::Relaxed) >= MAX_RETRIES {
                return;
            }
            usb.devcmdstat
                .write(|w| unsafe { w.bits(stat & !W1C & !DCON) });
            DOWN_SINCE.store(now.max(1), Ordering::Relaxed);
            PROBE_HS_RETRIES.fetch_add(1, Ordering::Relaxed);
        }
    }

    /// Bench-only: counts what re-enters the USB1 pass and what the pass did, in one array a
    /// debugger snapshots in a single read.
    #[cfg(feature = "pass-trace")]
    pub mod pass_trace {
        use core::sync::atomic::{AtomicU32, Ordering};

        pub const PASSES: usize = 0;
        /// Passes entered with the controller asserting no enabled interrupt -- a software pend.
        pub const IDLE_ENTRY: usize = 1;
        /// Worst cycles one pass took, and worst cycles inside its DAP loop.
        pub const PASS_MAX: usize = 2;
        /// FLEXCOMM0 entries: the other priority-2 handler that can keep this pass off the core.
        pub const MAX_GAP: usize = 3;
        pub const POLLED: usize = 4;
        /// Packets offered to the CDC IN endpoint, for a bench build measuring the return leg.
        pub const FLUSHES: usize = 5;
        pub const REQUEST: usize = 6;
        pub const EXECUTED: usize = 7;
        pub const RESPONSE: usize = 8;
        pub const INTSTAT_LAST: usize = 9;
        pub const PENDING_LAST: usize = 10;
        pub const DAP_MAX: usize = 11;
        #[no_mangle]
        pub static PASS_TRACE: [AtomicU32; 12] = [const { AtomicU32::new(0) }; 12];

        /// The last 32 passes that saw anything, as (millisecond, what), newest at `PASS_LOG_AT`.
        pub const LOG_ENTRIES: usize = 32;
        #[no_mangle]
        pub static PASS_LOG: [AtomicU32; 2 * LOG_ENTRIES] =
            [const { AtomicU32::new(0) }; 2 * LOG_ENTRIES];
        #[no_mangle]
        pub static PASS_LOG_AT: AtomicU32 = AtomicU32::new(0);
        /// Millisecond the ring was frozen at, or zero while it is still recording.
        #[no_mangle]
        pub static PASS_LOG_FROZE: AtomicU32 = AtomicU32::new(0);

        /// `what`: the pending set in bits 0..15, then polled, request, response, staging room.
        pub fn log(ms: u32, pending: u32, polled: bool, room: u32, blocks: u32) {
            static SEEN_REQ: AtomicU32 = AtomicU32::new(0);
            static SEEN_RESP: AtomicU32 = AtomicU32::new(0);
            let took = PASS_TRACE[REQUEST].load(Ordering::Relaxed);
            let wrote = PASS_TRACE[RESPONSE].load(Ordering::Relaxed);
            let took = SEEN_REQ.swap(took, Ordering::Relaxed) != took;
            let wrote = SEEN_RESP.swap(wrote, Ordering::Relaxed) != wrote;
            if PASS_LOG_FROZE.load(Ordering::Relaxed) != 0 {
                return;
            }
            // Freeze on the silence itself: a third of a second with no event is the stall.
            {
                static POLLED_AT: AtomicU32 = AtomicU32::new(0);
                if polled {
                    POLLED_AT.store(ms, Ordering::Relaxed);
                } else {
                    let since = POLLED_AT.load(Ordering::Relaxed);
                    // Armed only once the stream is well under way, so the setup lull cannot trip it.
                    let running = super::VCOM_UART_RX.load(Ordering::Relaxed) > 50_000;
                    if running && since != 0 && ms.wrapping_sub(since) > 300 {
                        PASS_LOG_FROZE.store(ms, Ordering::Relaxed);
                        return;
                    }
                }
            }

            static LAST_MS: AtomicU32 = AtomicU32::new(0);
            let quiet = ms.wrapping_sub(LAST_MS.load(Ordering::Relaxed)) < 32;
            if pending == 0 && !polled && !took && !wrote && quiet {
                return;
            }
            LAST_MS.store(ms, Ordering::Relaxed);
            let at = (PASS_LOG_AT.fetch_add(1, Ordering::Relaxed) as usize) % LOG_ENTRIES;
            let what = (pending & 0xfff)
                | (polled as u32) << 12
                | (took as u32) << 13
                | (wrote as u32) << 14
                | (room.min(0xfff)) << 15
                | (blocks & 0x1f) << 27;
            PASS_LOG[2 * at].store(ms, Ordering::Relaxed);
            PASS_LOG[2 * at + 1].store(what, Ordering::Relaxed);
        }

        pub fn bump(slot: usize) {
            PASS_TRACE[slot].fetch_add(1, Ordering::Relaxed);
        }

        /// Free-running cycle counter, armed on the first pass.
        pub fn now() -> u32 {
            static ARMED: AtomicU32 = AtomicU32::new(0);
            if ARMED.swap(1, Ordering::Relaxed) == 0 {
                // SAFETY: the debug block is otherwise untouched by this firmware.
                let mut core = unsafe { cortex_m::Peripherals::steal() };
                core.DCB.enable_trace();
                core.DWT.enable_cycle_counter();
            }
            cortex_m::peripheral::DWT::cycle_count()
        }

        pub fn worst(slot: usize, since: u32) {
            PASS_TRACE[slot].fetch_max(now().wrapping_sub(since), Ordering::Relaxed);
        }

        /// Sample the controller before the stack gets a chance to clear anything.
        pub fn entry() {
            // SAFETY: status registers of the peripheral this task owns; read-only here.
            let usb = unsafe { &*lpc55_hal::raw::USB1::ptr() };
            let intstat = usb.intstat.read().bits();
            let pending = intstat & usb.inten.read().bits();
            bump(PASSES);
            PASS_TRACE[INTSTAT_LAST].store(intstat, Ordering::Relaxed);
            PASS_TRACE[PENDING_LAST].store(pending, Ordering::Relaxed);
            {
                static LAST: AtomicU32 = AtomicU32::new(0);
                let at = now();
                let prev = LAST.swap(at, Ordering::Relaxed);
                if prev != 0 {
                    PASS_TRACE[MAX_GAP].fetch_max(at.wrapping_sub(prev), Ordering::Relaxed);
                }
            }
            LAST_PENDING.store(pending, Ordering::Relaxed);
            if pending == 0 {
                bump(IDLE_ENTRY);
            }
        }

        pub static LAST_PENDING: AtomicU32 = AtomicU32::new(0);

        pub fn polled(reported: bool) {
            if reported {
                bump(POLLED);
            }
        }

        pub fn flushed() {
            bump(FLUSHES);
        }

        pub fn request() {
            bump(REQUEST);
        }

        pub fn executed() {
            bump(EXECUTED);
        }

        pub fn response() {
            bump(RESPONSE);
        }

        /// One line per half second: interval totals, plus the last pass's intstat/pending sample.
        #[cfg(feature = "defmt")]
        pub fn report(now_ms: u32) {
            static NEXT_MS: AtomicU32 = AtomicU32::new(0);
            if now_ms < NEXT_MS.load(Ordering::Relaxed) {
                return;
            }
            NEXT_MS.store(now_ms.wrapping_add(500), Ordering::Relaxed);
            let take = |slot: usize| PASS_TRACE[slot].swap(0, Ordering::Relaxed);
            defmt::info!(
                "usb pass: {=u32} passes, {=u32} idle-entry, intstat {=u32:#010x}, pending {=u32:#010x} | polled {=u32}, flushed {=u32}, req {=u32}, exec {=u32}, resp {=u32}",
                take(PASSES),
                take(IDLE_ENTRY),
                PASS_TRACE[INTSTAT_LAST].load(Ordering::Relaxed),
                PASS_TRACE[PENDING_LAST].load(Ordering::Relaxed),
                take(POLLED),
                take(FLUSHES),
                take(REQUEST),
                take(EXECUTED),
                take(RESPONSE),
            );
        }
    }

    /// Bench-only: where the shared USB1 pass's time goes, read out of RAM over SWD.
    #[cfg(feature = "probe-timing")]
    pub mod timing {
        use core::sync::atomic::{AtomicU32, Ordering};

        /// USB1 and FLEXCOMM0 entries, and worst microseconds per pass (VCOM leg, whole pass).
        #[no_mangle]
        pub static PROBE_USB1_PASSES: AtomicU32 = AtomicU32::new(0);
        #[no_mangle]
        pub static PROBE_FC0_ENTRIES: AtomicU32 = AtomicU32::new(0);
        #[no_mangle]
        pub static PROBE_USB1_MAX_US: AtomicU32 = AtomicU32::new(0);
        #[no_mangle]
        pub static PROBE_VCOM_MAX_US: AtomicU32 = AtomicU32::new(0);
        #[no_mangle]
        pub static PROBE_USB1_TOTAL_US: AtomicU32 = AtomicU32::new(0);

        /// CDC packets each way; the forward pair also counts the transmit path's DMA transfers.
        #[no_mangle]
        pub static PROBE_CDC_OUT_PKTS: AtomicU32 = AtomicU32::new(0);
        #[no_mangle]
        pub static PROBE_CDC_OUT_BYTES: AtomicU32 = AtomicU32::new(0);
        #[no_mangle]
        pub static PROBE_CDC_IN_PKTS: AtomicU32 = AtomicU32::new(0);

        /// Bits still asserted on entry, sampled before `poll`: level-driven, so a set bit
        /// re-enters this handler the moment it returns.
        #[no_mangle]
        pub static PROBE_IS_EP1OUT: AtomicU32 = AtomicU32::new(0);
        #[no_mangle]
        pub static PROBE_IS_EP3OUT: AtomicU32 = AtomicU32::new(0);
        #[no_mangle]
        pub static PROBE_IS_EP3IN: AtomicU32 = AtomicU32::new(0);
        #[no_mangle]
        pub static PROBE_IS_FRAME: AtomicU32 = AtomicU32::new(0);
        #[no_mangle]
        pub static PROBE_IS_DEV: AtomicU32 = AtomicU32::new(0);
        /// DMA0 completion entries, against the transmit path's armed count.
        #[no_mangle]
        pub static PROBE_DMA0_ENTRIES: AtomicU32 = AtomicU32::new(0);

        /// Passes that found the DAP request queue full, so EP1 OUT was left unread.
        #[no_mangle]
        pub static PROBE_DAP_FULL: AtomicU32 = AtomicU32::new(0);

        /// Passes entering with EP3 OUT armed (nothing sent yet) versus with a packet waiting.
        #[no_mangle]
        pub static PROBE_EP3OUT_ARMED: AtomicU32 = AtomicU32::new(0);
        #[no_mangle]
        pub static PROBE_EP3OUT_LANDED: AtomicU32 = AtomicU32::new(0);

        /// Sample EP3 OUT's active bit: endpoint list at 0x4010_0000, EP3's first OUT buffer at
        /// +0x30, bit 31 is `A`.
        pub fn sample_ep3out() {
            // SAFETY: a plain volatile read of the controller's command/status list.
            let entry = unsafe { core::ptr::read_volatile(0x4010_0030 as *const u32) };
            if entry & (1 << 31) != 0 {
                bump(&PROBE_EP3OUT_ARMED, 1);
            } else {
                bump(&PROBE_EP3OUT_LANDED, 1);
            }
        }

        /// Charge one pass's worth of `INTSTAT`. USB1 is at 0x4009_4000 and `INTSTAT` at +0x20.
        pub fn sample_intstat() {
            // SAFETY: a plain volatile read of a memory-mapped status register.
            let bits = unsafe { core::ptr::read_volatile(0x4009_4020 as *const u32) };
            for (mask, cell) in [
                (1 << 2, &PROBE_IS_EP1OUT),
                (1 << 6, &PROBE_IS_EP3OUT),
                (1 << 7, &PROBE_IS_EP3IN),
                (1 << 30, &PROBE_IS_FRAME),
                (1 << 31, &PROBE_IS_DEV),
            ] {
                if bits & mask != 0 {
                    bump(cell, 1);
                }
            }
        }

        /// Passes the forward leg spent ring-blocked, and passes with room and nothing to send.
        #[no_mangle]
        pub static PROBE_TX_FULL: AtomicU32 = AtomicU32::new(0);
        #[no_mangle]
        pub static PROBE_TX_STARVED: AtomicU32 = AtomicU32::new(0);

        /// Free-running cycle counter, started once at boot.
        pub fn start() {
            let mut core = unsafe { cortex_m::Peripherals::steal() };
            core.DCB.enable_trace();
            core.DWT.enable_cycle_counter();
        }

        pub fn now() -> u32 {
            // A debugger detach clears DEMCR.TRCENA and freezes CYCCNT; re-arm when it drops.
            const TRCENA: u32 = 1 << 24;
            let dcb = unsafe { &*cortex_m::peripheral::DCB::PTR };
            if dcb.demcr.read() & TRCENA == 0 {
                unsafe { dcb.demcr.modify(|d| d | TRCENA) };
            }
            cortex_m::peripheral::DWT::cycle_count()
        }

        pub fn cycles_since(start: u32) -> u32 {
            now().wrapping_sub(start)
        }

        pub fn max(cell: &AtomicU32, us: u32) {
            cell.fetch_max(us, Ordering::Relaxed);
        }

        pub fn bump(cell: &AtomicU32, by: u32) {
            cell.fetch_add(by, Ordering::Relaxed);
        }
    }

    /// The reboot-to-bootloader vendor handler: no flash session exists to gate it.
    #[cfg(not(feature = "flash"))]
    type Vendor =
        VendorChain<RebootVendor<Lpc55Boot>, VendorChain<VcomBaudVendor, VcomStatsVendor>>;

    /// The bit-bang SWD+JTAG engine: Flexcomm3 SWO capture, no LEDs, reboot in the vendor slot.
    #[cfg(all(feature = "bitbang", not(feature = "flash")))]
    type Engine = BitbangDap<'static, IoPin, DynCycleDelay, Fc3Swo, Vendor>;

    /// Half-period cycle count at or below which the accel fast paths run.
    #[cfg(all(feature = "accel", not(feature = "bitbang"), not(feature = "flash")))]
    const ACCEL_FAST_HALF_CYCLES: u32 = 4;

    /// Both independently calibrated SWD profiles hand off at three half cycles.
    #[cfg(all(feature = "accel", not(feature = "bitbang"), not(feature = "flash")))]
    const ACCEL_SWD_FAST_HALF_CYCLES: u32 = 3;

    /// JTAG uses a separately calibrated profile for each core clock.
    #[cfg(all(feature = "accel", not(feature = "bitbang"), not(feature = "flash")))]
    const ACCEL_JTAG_FAST_HALF_CYCLES: u32 = if cfg!(feature = "core-96mhz") { 3 } else { 0 };

    /// Scaled JTAG floors belong to the selected core-clock profile.
    #[cfg(all(feature = "accel", not(feature = "bitbang"), not(feature = "flash")))]
    const ACCEL_JTAG_SCALED_FLOOR_CYCLES: u32 = if cfg!(feature = "core-96mhz") {
        ACCEL_FAST_HALF_CYCLES
    } else {
        6
    };
    #[cfg(all(feature = "accel", not(feature = "bitbang"), not(feature = "flash")))]
    const ACCEL_JTAG_SEQUENCE_FLOOR_CYCLES: u32 = if cfg!(feature = "core-96mhz") {
        ACCEL_FAST_HALF_CYCLES
    } else {
        7
    };

    /// NOP counts are independent calibration tuples, not scaled between clock profiles.
    #[cfg(all(feature = "accel", not(feature = "bitbang"), not(feature = "flash")))]
    const ACCEL_SWD_CONTROL_LOW_NOPS: usize = if cfg!(feature = "core-96mhz") { 5 } else { 6 };
    #[cfg(all(feature = "accel", not(feature = "bitbang"), not(feature = "flash")))]
    const ACCEL_SWD_CONTROL_HIGH_NOPS: usize = 4;
    #[cfg(all(feature = "accel", not(feature = "bitbang"), not(feature = "flash")))]
    const ACCEL_SWD_READ_LOW_NOPS: usize = 5;
    #[cfg(all(feature = "accel", not(feature = "bitbang"), not(feature = "flash")))]
    const ACCEL_SWD_READ_HIGH_NOPS: usize = 4;
    #[cfg(all(feature = "accel", not(feature = "bitbang"), not(feature = "flash")))]
    const ACCEL_SWD_WRITE_LOW_NOPS: usize = if cfg!(feature = "core-96mhz") { 5 } else { 6 };
    #[cfg(all(feature = "accel", not(feature = "bitbang"), not(feature = "flash")))]
    const ACCEL_SWD_WRITE_HIGH_NOPS: usize = 4;

    /// Nops pacing the low half of a fast JTAG scan clock: the TCK-fall to TDO-sample margin.
    #[cfg(all(feature = "accel", not(feature = "bitbang"), not(feature = "flash")))]
    const ACCEL_SCAN_LOW_NOPS: usize = 7;

    /// Nops pacing the high half: TCK-high pulse width only.
    #[cfg(all(feature = "accel", not(feature = "bitbang"), not(feature = "flash")))]
    const ACCEL_SCAN_HIGH_NOPS: usize = 1;

    /// Scaled JTAG fixed work is independently calibrated for each core clock.
    #[cfg(all(feature = "accel", not(feature = "bitbang"), not(feature = "flash")))]
    const ACCEL_JTAG_FIXED_CYCLES: [(u32, u32); 4] = if cfg!(feature = "core-96mhz") {
        [(0, 0), (8, 10), (6, 13), (0, 0)]
    } else {
        [(3, 1), (4, 10), (3, 13), (3, 1)]
    };

    /// The accel image's pacing, over the firmware-owned constants above.
    #[cfg(all(feature = "accel", not(feature = "bitbang"), not(feature = "flash")))]
    #[derive(Clone, Copy)]
    struct AccelPacing;

    #[cfg(all(feature = "accel", not(feature = "bitbang"), not(feature = "flash")))]
    impl JtagPacing for AccelPacing {
        const FAST_HALF_CYCLES: u32 = ACCEL_FAST_HALF_CYCLES;

        #[inline(always)]
        fn pace_low(&self) {
            nops::<ACCEL_SCAN_LOW_NOPS>();
        }

        #[inline(always)]
        fn pace_high(&self) {
            nops::<ACCEL_SCAN_HIGH_NOPS>();
        }
    }

    #[cfg(all(feature = "accel", not(feature = "bitbang"), not(feature = "flash")))]
    impl MaskedPacing for AccelPacing {
        const SWD_FAST_HALF_CYCLES: u32 = ACCEL_SWD_FAST_HALF_CYCLES;
        const JTAG_FAST_HALF_CYCLES: u32 = ACCEL_JTAG_FAST_HALF_CYCLES;
        const JTAG_SCALED_FLOOR_CYCLES: u32 = ACCEL_JTAG_SCALED_FLOOR_CYCLES;
        const JTAG_SEQUENCE_FLOOR_CYCLES: u32 = ACCEL_JTAG_SEQUENCE_FLOOR_CYCLES;
        const JTAG_CONTROL_FIXED_CYCLES: (u32, u32) = ACCEL_JTAG_FIXED_CYCLES[0];
        const JTAG_READ_FIXED_CYCLES: (u32, u32) = ACCEL_JTAG_FIXED_CYCLES[1];
        const JTAG_WRITE_FIXED_CYCLES: (u32, u32) = ACCEL_JTAG_FIXED_CYCLES[2];
        const JTAG_IR_FIXED_CYCLES: (u32, u32) = ACCEL_JTAG_FIXED_CYCLES[3];

        #[inline(always)]
        fn swd_control_low(&self) {
            nops::<ACCEL_SWD_CONTROL_LOW_NOPS>();
        }

        #[inline(always)]
        fn swd_control_high(&self) {
            nops::<ACCEL_SWD_CONTROL_HIGH_NOPS>();
        }

        #[inline(always)]
        fn swd_read_low(&self) {
            nops::<ACCEL_SWD_READ_LOW_NOPS>();
        }

        #[inline(always)]
        fn swd_read_high(&self) {
            nops::<ACCEL_SWD_READ_HIGH_NOPS>();
        }

        #[inline(always)]
        fn swd_write_low(&self) {
            nops::<ACCEL_SWD_WRITE_LOW_NOPS>();
        }

        #[inline(always)]
        fn swd_write_high(&self) {
            nops::<ACCEL_SWD_WRITE_HIGH_NOPS>();
        }
    }

    /// The accelerated engine: masked-GPIO SWD words and fast JTAG scans, same SWO and vendor.
    #[cfg(all(feature = "accel", not(feature = "bitbang"), not(feature = "flash")))]
    type Engine = MaskedSwdDap<'static, AccelPacing, Fc3Swo, Vendor>;

    /// The dap-rs dependency role of the one adapter the flash build arbitrates.
    #[cfg(feature = "flash")]
    type MuxDeps = LateJtagDeps<IoPin, DynCycleDelay>;
    /// The SWD driver role of [`MuxDeps`].
    #[cfg(feature = "flash")]
    type MuxSwd = LateSwdDrv<IoPin, DynCycleDelay>;
    /// The JTAG driver role of [`MuxDeps`].
    #[cfg(feature = "flash")]
    type MuxJtag = LateJtag<IoPin, DynCycleDelay>;
    /// The flash-wins arbiter owning the one physical adapter.
    #[cfg(feature = "flash")]
    type Mux = SwdMux<MuxDeps, MuxSwd, MuxJtag>;
    /// The flash-algorithm runtime over an ungated flash-side handle on the mux.
    #[cfg(feature = "flash")]
    type Backend =
        RoleMuxFlashBackend<MuxDeps, MuxSwd, MuxJtag, DynCycleDelay, PrimaryTarget, Selector>;
    /// The image-fragment sink one flash front-end owns.
    #[cfg(feature = "flash")]
    type Sink = FlashSink<Backend>;
    /// The vendor slot's flash-busy gate: the mux's own live-session flag.
    #[cfg(feature = "flash")]
    type RebootBusy = &'static Mux;
    /// The reboot-to-bootloader command: declined with `STATUS_BUSY` while a flash session is live.
    #[cfg(feature = "flash")]
    type Vendor = VendorChain<
        RebootVendor<Lpc55Boot, RebootBusy>,
        VendorChain<VendorFlash<AllDecoders, Sink>, VendorChain<VcomBaudVendor, VcomStatsVendor>>,
    >;
    /// The flash build's engine: gated by the mux, same SWO slot and vendor chain as above.
    #[cfg(feature = "flash")]
    type Engine =
        RoleMuxDapEngine<'static, MuxDeps, MuxSwd, MuxJtag, DynCycleDelay, Fc3Swo, Vendor>;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        usb: DapUsb<'static, Bus, MSC_EP, CDC_EP, CDC_BUF>,
        dap: Engine,
        /// The virtual mass-storage volume, streaming a dropped image through the flash-wins mux.
        #[cfg(feature = "flash")]
        vfs: &'static mut FlashVolume<Sink>,
        /// The previous [`ELAPSED_MS`](crate::ELAPSED_MS) reading, for a wrap-safe delta.
        #[cfg(feature = "flash")]
        last_ms: u32,
        /// Caller-owned pipelined command queue, drained and executed by the USB task.
        dap_queue: &'static mut Queue,
        uart: Vcom,
        /// Commands run since the last connection edge, logged once at `session_ended()`.
        #[cfg(feature = "defmt")]
        processed: u32,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        #[cfg(feature = "probe-timing")]
        timing::start();
        // Boot the stamp first: it arms the fault latch, and everything below can panic.
        // SAFETY: `RESET_STAMP` is only touched here, once, before anything concurrent runs.
        #[cfg(feature = "flash")]
        let stamp = daplink_vfs::ResetStamp::boot_in_place(unsafe {
            &mut *core::ptr::addr_of_mut!(crate::RESET_STAMP)
        });

        #[cfg(feature = "defmt")]
        defmt::info!("mculink: boot");

        let delay =
            DynCycleDelay::with_fixed_work(CPU_HZ, SWD_FIXED_WORK_CYCLES, MIN_HALF_PERIOD_CYCLES);
        let parts = probe_parts(cx.device, delay);
        let cfg = parts.cfg.with_product(product_string!("mculink"));
        #[cfg(feature = "defmt")]
        defmt::info!("mculink: serial {=str}", parts.serial);

        #[cfg(all(feature = "bitbang", not(feature = "flash")))]
        let dap: Engine = Engine::with_vendor(
            parts.deps,
            daplink_rs::NoopLeds,
            delay,
            parts.swo,
            DAP_FW_VERSION,
            cfg.max_packet_size,
            VendorChain::new(
                RebootVendor::new(Lpc55Boot),
                VendorChain::new(
                    VcomBaudVendor::new(vcom_baud_state),
                    VcomStatsVendor::new(vcom_counters),
                ),
            ),
        )
        .with_packet_count(DAP_PACKET_COUNT);

        #[cfg(all(feature = "accel", not(feature = "bitbang"), not(feature = "flash")))]
        let dap: Engine = {
            // Copy `.sramx_code` (placed by sramx.x) to code-bus SRAM before a transfer runs.
            // SAFETY: linker-provided bounds, and nothing executes from the destination yet.
            unsafe {
                extern "C" {
                    static mut __ssramx_code: u32;
                    static mut __esramx_code: u32;
                    static __sisramx_code: u32;
                }
                let mut dst = core::ptr::addr_of_mut!(__ssramx_code);
                let end = core::ptr::addr_of_mut!(__esramx_code);
                let mut src = core::ptr::addr_of!(__sisramx_code);
                while dst < end {
                    dst.write_volatile(src.read_volatile());
                    dst = dst.add(1);
                    src = src.add(1);
                }
            }
            Engine::with_vendor(
                masked_deps(delay, AccelPacing),
                daplink_rs::NoopLeds,
                delay,
                parts.swo,
                DAP_FW_VERSION,
                cfg.max_packet_size,
                VendorChain::new(
                    RebootVendor::new(Lpc55Boot),
                    VendorChain::new(
                        VcomBaudVendor::new(vcom_baud_state),
                        VcomStatsVendor::new(vcom_counters),
                    ),
                ),
            )
            .with_packet_count(DAP_PACKET_COUNT)
        };

        #[cfg(feature = "flash")]
        let (dap, vfs) = {
            static MUX: StaticCell<Mux> = StaticCell::new();
            static MUX_SCAN: ScanChainStorage<{ daplink_rs::daplink_core::SCAN_CHAIN_SLOTS }> =
                ScanChainStorage::new();
            // Two front-ends, two backends; the mux grants one session and refuses the other.
            static VENDOR_SCRATCH: ConstStaticCell<[u8; SCRATCH]> =
                ConstStaticCell::new([0; SCRATCH]);
            static VOLUME: VolumeStorage<Sink> = VolumeStorage::new();

            let mux: &'static Mux = MUX.init(SwdMux::new(parts.deps));
            let table = flash_table();
            let vendor_backend: Backend =
                role_build_flash_backend(mux, delay, PRIMARY, table, VENDOR_SCRATCH.take());
            let dap: Engine = role_mux_dap_engine_with_vendor(
                mux,
                delay,
                parts.swo,
                cfg.max_packet_size,
                &MUX_SCAN,
                VendorChain::new(
                    RebootVendor::with_busy_check(Lpc55Boot, mux),
                    VendorChain::new(
                        VendorFlash::new(FlashSink::new(vendor_backend)),
                        VendorChain::new(
                            VcomBaudVendor::new(vcom_baud_state),
                            VcomStatsVendor::new(vcom_counters),
                        ),
                    ),
                ),
                daplink_rs::NoopLeds,
            )
            .with_packet_count(DAP_PACKET_COUNT);
            let vfs_backend: Backend =
                role_build_flash_backend(mux, delay, PRIMARY, table, flash_scratch());
            let vfs = daplink_vfs::assemble_info(
                &VOLUME,
                FlashSink::new(vfs_backend),
                daplink_vfs::DeviceInfo {
                    version: env!("CARGO_PKG_VERSION"),
                    serial: parts.serial,
                    reset_count: stamp.reset_count(),
                },
                Some(stamp),
                "DAPLINK",
            );

            (dap, vfs)
        };

        // The 1 ms tick bounds a receive backlog's wait and is the flash volume's only clock.
        let mut syst = cx.core.SYST;
        syst.set_clock_source(SystClkSource::Core);
        // One interrupt per millisecond, expressible only at a whole number of kHz.
        const { assert!(CPU_HZ.is_multiple_of(1000)) };
        syst.set_reload(CPU_HZ / 1000 - 1);
        syst.clear_current();
        syst.enable_interrupt();
        syst.enable_counter();

        let usb = DapUsb::new(parts.usb_alloc, &cfg);

        // `ConstStaticCell` keeps the ring pair in `.bss` instead of on the init stack.
        static DAP_QUEUE: ConstStaticCell<Queue> = ConstStaticCell::new(Queue::new());
        let dap_queue = DAP_QUEUE.take();

        // Composition finished, so this boot is not a reset loop: forgive the fault streak.
        #[cfg(feature = "flash")]
        daplink_vfs::stamp::clear_fault_streak();

        (
            Shared {},
            Local {
                usb,
                dap,
                #[cfg(feature = "flash")]
                vfs,
                #[cfg(feature = "flash")]
                last_ms: 0,
                dap_queue,
                uart: parts.uart,
                #[cfg(feature = "defmt")]
                processed: 0,
            },
        )
    }

    /// Idle: the ROM entry must run from thread mode, after a grace so the DAP response drains.
    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            if reboot_armed() {
                cortex_m::asm::delay(CPU_HZ / 20);
                Lpc55Boot.enter();
            }
        }
    }

    /// UART RX: drain Flexcomm0 into the receive ring at priority 2, above any DAP command.
    #[task(binds = FLEXCOMM0, priority = 2)]
    fn on_uart(_cx: on_uart::Context) {
        #[cfg(feature = "probe-timing")]
        timing::bump(&timing::PROBE_FC0_ENTRIES, 1);
        vcom_service_irq();
        NVIC::pend(Interrupt::USB1);
    }

    /// VCOM TX: re-arm the next staging slot here rather than a pass later, to keep the line
    /// busy. SWO: count the trace chain's block completions.
    #[task(binds = DMA0, priority = 2)]
    fn on_dma(_cx: on_dma::Context) {
        #[cfg(feature = "probe-timing")]
        timing::bump(&timing::PROBE_DMA0_ENTRIES, 1);
        vcom_tx_service_irq();
        swo_dma_service_irq();
        // A completed transfer frees ring space, so it owes the stalled pass a wake.
        NVIC::pend(Interrupt::USB1);
    }

    /// USB1: bridge the VCOM port, pump the DAP queue, then advance mass storage. Sector work
    /// runs inline, so a long flash program holds the mux and gates the DAP engine.
    #[task(binds = USB1, local = [usb, dap, dap_queue, uart, #[cfg(feature = "defmt")] processed, #[cfg(feature = "flash")] vfs, #[cfg(feature = "flash")] last_ms])]
    fn on_usb(cx: on_usb::Context) {
        #[cfg(feature = "probe-timing")]
        let entered = timing::now();
        #[cfg(feature = "probe-timing")]
        {
            timing::sample_intstat();
            timing::sample_ep3out();
        }
        let usb = cx.local.usb;
        let dap = cx.local.dap;
        let dap_queue = &mut **cx.local.dap_queue;
        let uart = cx.local.uart;
        // Arm the receive channel only once enumeration is over, or the attach is lost.
        if usb.configured() {
            uart.start_rx_dma();
        }

        #[cfg(feature = "pass-trace")]
        let pass_started = pass_trace::now();
        #[cfg(feature = "pass-trace")]
        pass_trace::entry();
        let polled = usb.poll();
        #[cfg(feature = "pass-trace")]
        pass_trace::polled(polled);

        // Return UART bytes first: a receive backlog makes a divisor change unsafe.
        #[cfg(feature = "probe-timing")]
        let vcom_started = timing::now();
        vcom_return(usb, uart);
        #[cfg(feature = "probe-timing")]
        timing::max(
            &timing::PROBE_VCOM_MAX_US,
            timing::cycles_since(vcom_started),
        );

        // Follow the host's line coding between exchanges, before host traffic is consumed.
        let requested = usb.vcom_line_coding_baud();
        if requested != 0 && !uart.baud_applied(requested) {
            uart.request_baud(requested);
        }

        // Backpressure CDC OUT until the new divisor applies, or a packet goes out at the old rate.
        if requested == 0 || uart.baud_applied(requested) {
            vcom_host_traffic(usb, uart);
        }

        if polled {
            // Host gone: drop the dead session's queue and release the wire.
            if usb.session_ended() {
                #[cfg(feature = "defmt")]
                {
                    defmt::info!(
                        "dap session: {=u32} commands processed",
                        *cx.local.processed
                    );
                    *cx.local.processed = 0;
                }
                dap_queue.reset();
                dap.suspend();
            }

            // Skip the OUT endpoint while the ring is full, so it NAKs the host into pacing itself.
            #[cfg(feature = "probe-timing")]
            if dap_queue.requests_full() {
                timing::bump(&timing::PROBE_DAP_FULL, 1);
            }
            if !dap_queue.requests_full()
                && dap_queue
                    .receive_with(|slot| usb.read_command(slot))
                    .expect("a checked-non-full request slot accepts one endpoint packet")
                    .is_some()
            {
                #[cfg(feature = "pass-trace")]
                pass_trace::request();
            }

            // Run queued commands oldest first, never dequeuing with nowhere to put the response.
            #[cfg(feature = "pass-trace")]
            let dap_started = pass_trace::now();
            let mut executed = 0;
            while executed < DAP_QUEUE_DEPTH {
                if dap_queue.responses_full() {
                    break;
                }
                match dap_queue
                    .execute_next_with(|request, response| dap.process_command(request, response))
                {
                    Ok(Some(_)) => {}
                    Ok(None) => break,
                    Err(_) => {
                        debug_assert!(false, "reserved request and response slots fit one packet");
                        break;
                    }
                }
                #[cfg(feature = "pass-trace")]
                pass_trace::executed();
                #[cfg(feature = "defmt")]
                {
                    *cx.local.processed += 1;
                }
                executed += 1;
            }
            #[cfg(feature = "pass-trace")]
            pass_trace::worst(pass_trace::DAP_MAX, dap_started);

            // Send whatever is ready, stopping at the first busy-endpoint refusal.
            while let Some(response) = dap_queue.pending_response() {
                if !usb.write_response(response) {
                    break;
                }
                #[cfg(feature = "pass-trace")]
                pass_trace::response();
                dap_queue.discard_response();
            }
        }

        // Advance the volume's timers by the elapsed milliseconds, then move one bulk packet.
        #[cfg(feature = "flash")]
        {
            let now = crate::ELAPSED_MS.load(Ordering::Relaxed);
            let dt = now.wrapping_sub(*cx.local.last_ms);
            *cx.local.last_ms = now;
            let vfs = cx.local.vfs;
            vfs.tick(dt);
            usb.msc_service(vfs);
        }

        #[cfg(feature = "usb-hs-retry")]
        hs_retry::poll(crate::ELAPSED_MS.load(Ordering::Relaxed));

        // Re-pend to drain a landed packet, but never on a full ring: that spins.
        if usb.vcom_read_pending() && uart.write_capacity() != 0 {
            NVIC::pend(Interrupt::USB1);
        }

        #[cfg(feature = "pass-trace")]
        {
            pass_trace::worst(pass_trace::PASS_MAX, pass_started);
            pass_trace::log(
                crate::ELAPSED_MS.load(Ordering::Relaxed),
                pass_trace::LAST_PENDING.load(Ordering::Relaxed),
                polled,
                usb.vcom_write_capacity() as u32,
                0,
            );
        }

        #[cfg(all(feature = "pass-trace", feature = "defmt"))]
        pass_trace::report(crate::ELAPSED_MS.load(Ordering::Relaxed));

        #[cfg(feature = "probe-timing")]
        {
            let cycles = timing::cycles_since(entered);
            timing::bump(&timing::PROBE_USB1_PASSES, 1);
            timing::bump(&timing::PROBE_USB1_TOTAL_US, cycles);
            timing::max(&timing::PROBE_USB1_MAX_US, cycles);
        }
    }

    type Usb = DapUsb<'static, Bus, MSC_EP, CDC_EP, CDC_BUF>;

    /// How long a partial CDC packet waits for company.
    const CDC_COALESCE_MS: u32 = 1;

    /// When a packet was last offered to the CDC IN endpoint; the coalescing deadline runs from
    /// here, never from the staging going non-empty.
    static LAST_FLUSH_MS: AtomicU32 = AtomicU32::new(0);

    /// Bytes [`vcom_service`] took out of the receive ring.
    static VCOM_UART_RX: AtomicU32 = AtomicU32::new(0);
    /// Bytes of those staged toward the CDC IN endpoint, before the discards deducted below.
    static VCOM_CDC_IN: AtomicU32 = AtomicU32::new(0);
    /// Bytes taken off the CDC OUT endpoint for the UART transmit ring.
    static VCOM_CDC_OUT: AtomicU32 = AtomicU32::new(0);
    /// Bytes [`VCOM_CDC_IN`] credited that a port close or a bus reset then threw away.
    static VCOM_HOST_DROPPED: AtomicU32 = AtomicU32::new(0);

    /// The VCOM bridge's conservation ledger, for the `0x9D` vendor query.
    fn vcom_counters() -> VcomCounters {
        use daplink_rs::{
            vcom_episodes, vcom_grade, VCOM_GRADE_EXACT, VCOM_GRADE_LOWER_BOUND, VCOM_SLOT_CDC_IN,
            VCOM_SLOT_CDC_OUT, VCOM_SLOT_FRAME_ERR, VCOM_SLOT_HOST_DROPPED, VCOM_SLOT_OVERRUN,
            VCOM_SLOT_RING_FULL, VCOM_SLOT_RX_DMA_ERR, VCOM_SLOT_UART_RX, VCOM_SLOT_UART_TX,
        };
        let rx = vcom_rx_discards();
        // Staged bytes are credited to `cdc_in`; deducting the discards here moves that credit
        // across in one step, so the query never reads a torn pair.
        let host_dropped = VCOM_HOST_DROPPED.load(Ordering::Relaxed);
        VcomCounters {
            uart_rx: VCOM_UART_RX.load(Ordering::Relaxed),
            cdc_in: VCOM_CDC_IN
                .load(Ordering::Relaxed)
                .wrapping_sub(host_dropped),
            host_dropped,
            ring_full: rx.ring_full,
            frame_err: rx.frame_err,
            overrun: rx.overrun,
            cdc_out: VCOM_CDC_OUT.load(Ordering::Relaxed),
            uart_tx: vcom_tx_sent(),
            rx_dma_err: vcom_rx_dma_errors(),
            valid: daplink_rs::VCOM_VALID_IDENTITY
                | daplink_rs::VCOM_VALID_CDC_OUT
                | daplink_rs::VCOM_VALID_RING_FULL
                | daplink_rs::VCOM_VALID_FRAME_ERR
                | daplink_rs::VCOM_VALID_OVERRUN
                | daplink_rs::VCOM_VALID_UART_TX
                | daplink_rs::VCOM_VALID_RX_DMA_ERR,
            // Lower bounds where the hardware under-reports: the self-linked descriptor hides
            // settled laps, one pass folds several error episodes, and the transmit count deducts
            // the FIFO residue rather than crediting it.
            grade: vcom_grade(VCOM_SLOT_UART_RX, VCOM_GRADE_EXACT)
                | vcom_grade(VCOM_SLOT_CDC_IN, VCOM_GRADE_EXACT)
                | vcom_grade(VCOM_SLOT_HOST_DROPPED, VCOM_GRADE_EXACT)
                | vcom_grade(VCOM_SLOT_CDC_OUT, VCOM_GRADE_EXACT)
                | vcom_grade(VCOM_SLOT_RING_FULL, VCOM_GRADE_LOWER_BOUND)
                | vcom_grade(VCOM_SLOT_FRAME_ERR, VCOM_GRADE_LOWER_BOUND)
                | vcom_grade(VCOM_SLOT_OVERRUN, VCOM_GRADE_LOWER_BOUND)
                | vcom_grade(VCOM_SLOT_UART_TX, VCOM_GRADE_LOWER_BOUND)
                | vcom_grade(VCOM_SLOT_RX_DMA_ERR, VCOM_GRADE_EXACT),
            units: vcom_episodes(VCOM_SLOT_FRAME_ERR)
                | vcom_episodes(VCOM_SLOT_OVERRUN)
                | vcom_episodes(VCOM_SLOT_RX_DMA_ERR),
        }
    }

    /// Return UART bytes only once the CDC buffer can take them whole.
    fn vcom_return(usb: &mut Usb, uart: &mut Vcom) {
        let mut rx = [0u8; CDC_EP];
        let mut taken = 0;
        let mut forwarded = 0;
        // Unconditional and first: an unsampled receive channel cannot be resynchronized.
        uart.service_rx();
        loop {
            let room = usb.vcom_write_capacity().min(rx.len());
            if room == 0 {
                break;
            }
            let n = uart.read(&mut rx[..room]);
            if n == 0 {
                break;
            }
            taken += n;
            let staged = usb.vcom_write_buffered(&rx[..n]);
            debug_assert_eq!(staged, n);
            forwarded += staged;
        }
        // A partial packet waits for a quiet line or the coalescing deadline; a full one never does.
        let staged = usb.vcom_staged();
        let now = crate::ELAPSED_MS.load(Ordering::Relaxed);
        let waited = now.wrapping_sub(LAST_FLUSH_MS.load(Ordering::Relaxed));
        if staged >= CDC_EP || (staged > 0 && (taken == 0 || waited >= CDC_COALESCE_MS)) {
            usb.vcom_flush();
            LAST_FLUSH_MS.store(now, Ordering::Relaxed);
            #[cfg(feature = "probe-timing")]
            timing::bump(&timing::PROBE_CDC_IN_PKTS, 1);
            #[cfg(feature = "pass-trace")]
            pass_trace::flushed();
        }
        // Charged after the pass settles, so the identity holds at any instant the query can see.
        VCOM_UART_RX.fetch_add(taken as u32, Ordering::Relaxed);
        VCOM_CDC_IN.fetch_add(forwarded as u32, Ordering::Relaxed);
        // A close or a bus reset throws the stage away; the host never got those bytes.
        VCOM_HOST_DROPPED.fetch_add(usb.vcom_take_discarded() as u32, Ordering::Relaxed);
    }

    /// Consume at most one bounded CDC packet when the UART transmit ring has room.
    fn vcom_host_traffic(usb: &mut Usb, uart: &mut Vcom) {
        // Here, not in FLEXCOMM0: with the FIFO on DMA that fires only for a baud change.
        observe_rx_errors();
        // The ring stages a run of bytes, not packets, so one pass's packets coalesce.
        let mut tx = [0u8; CDC_EP];
        loop {
            let room = uart.write_capacity().min(tx.len());
            if room == 0 {
                #[cfg(feature = "probe-timing")]
                timing::bump(&timing::PROBE_TX_FULL, 1);
                break;
            }
            let n = usb.vcom_read(&mut tx[..room]);
            if n == 0 {
                // Room and nothing to take: the leg is paced by the host, not by its own ring.
                #[cfg(feature = "probe-timing")]
                timing::bump(&timing::PROBE_TX_STARVED, 1);
                break;
            }
            VCOM_CDC_OUT.fetch_add(n as u32, Ordering::Relaxed);
            let queued = uart.write(&tx[..n]);
            debug_assert_eq!(queued, n);
            #[cfg(feature = "probe-timing")]
            {
                timing::bump(&timing::PROBE_CDC_OUT_PKTS, 1);
                timing::bump(&timing::PROBE_CDC_OUT_BYTES, n as u32);
            }
        }
    }
}
