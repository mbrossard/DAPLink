//! Reboot-to-bootloader vendor command (`0x9F`): a discovery echo, then a magic-guarded arm.

use daplink_core::vendor::VendorHandler;

use crate::swd_mux::SwdMux;

/// Vendor command id for the reboot-to-bootloader command (`ID_DAP_Vendor31`).
pub const REBOOT_CMD_ID: u8 = 0x9F;

/// Discovery/echo subcommand.
const SUB_DISCOVER: u8 = 0x00;
/// Reboot subcommand.
const SUB_REBOOT: u8 = 0x01;
/// Identity magic returned by discovery: marks a daplink-rs probe.
const REPLY_MAGIC: [u8; 4] = *b"dlrs";
/// Version byte returned by discovery.
const REBOOT_VERSION: u8 = 0x01;

/// Status: reboot armed, detach imminent.
pub const STATUS_ARMED: u8 = 0x00;
/// Status: reboot unsupported on this board (`kind == 0`).
pub const STATUS_UNSUPPORTED: u8 = 0x01;
/// Status: the reboot magic was missing or wrong.
pub const STATUS_BAD_MAGIC: u8 = 0x02;
/// Status: the probe is busy (e.g. a flash transfer is in progress) and declined.
pub const STATUS_BUSY: u8 = 0x03;

/// Bootloader kind: none -- this board has no reachable ROM update path.
pub const KIND_NONE: u8 = 0x00;
/// Bootloader kind: UF2 / BOOTSEL mass-storage (RP2040 / RP2350).
pub const KIND_UF2: u8 = 0x01;
/// Bootloader kind: ST system-memory USB DFU.
pub const KIND_ST_DFU: u8 = 0x02;
/// Bootloader kind: NXP ISP HID.
pub const KIND_NXP_ISP: u8 = 0x03;

/// The board-specific ROM bootloader entry mechanism injected into [`RebootVendor`].
pub trait BootloaderEntry {
    /// Which ROM bootloader [`enter`](BootloaderEntry::enter) lands in (see the `KIND_*` constants).
    /// `KIND_NONE` makes the reboot subcommand answer [`STATUS_UNSUPPORTED`] and `enter` unreachable.
    const KIND: u8;

    /// Latch a validated reboot request, from DAP dispatch context and possibly an ISR.
    /// A no-op by default; an impl whose trigger cannot reach the live handler latches a flag here.
    fn arm(&self) {}

    /// Enter the ROM bootloader; runs from thread mode only and never returns.
    /// Not called when [`KIND`](BootloaderEntry::KIND) is `KIND_NONE`.
    fn enter(&self) -> !;
}

/// [`BootloaderEntry`] for a board with no ROM update path: discovery answers [`KIND_NONE`].
pub struct NoBootloader;

impl BootloaderEntry for NoBootloader {
    const KIND: u8 = KIND_NONE;

    fn enter(&self) -> ! {
        // Never called: KIND_NONE is answered STATUS_UNSUPPORTED. Spin rather than reboot.
        loop {
            core::hint::spin_loop();
        }
    }
}

/// Query seam: whether a flash session holds the shared SWD bus, so a mid-flash `0x9F` is declined.
/// A `&'static SwdMux` implements it from its own live-session flag; [`NeverBusy`] is the default.
pub trait FlashBusy {
    /// `true` while a flash session holds the bus and a reboot must be declined.
    fn is_flashing(&self) -> bool;
}

/// The default busy query: never busy, for a probe with no flashing.
#[derive(Clone, Copy, Debug, Default)]
pub struct NeverBusy;

impl FlashBusy for NeverBusy {
    fn is_flashing(&self) -> bool {
        false
    }
}

/// Any `Fn() -> bool` is a busy query, so a firmware may inject a closure.
impl<F: Fn() -> bool> FlashBusy for F {
    fn is_flashing(&self) -> bool {
        self()
    }
}

/// The mux's own live-session flag is the busy query, so the type stays nameable in an alias.
impl<DEPS, SWD, JTAG> FlashBusy for &'static SwdMux<DEPS, SWD, JTAG>
where
    DEPS: dap_rs::swj::Dependencies<SWD, JTAG> + 'static,
    SWD: dap_rs::swd::Swd<DEPS> + 'static,
    JTAG: dap_rs::jtag::Jtag<DEPS> + 'static,
{
    fn is_flashing(&self) -> bool {
        SwdMux::is_flashing(self)
    }
}

/// The shared reboot-to-bootloader [`VendorHandler`]: parses `0x9F` and arms the injected
/// [`BootloaderEntry`] without entering inline, unless [`FlashBusy`] reports a live session.
pub struct RebootVendor<B: BootloaderEntry, Q: FlashBusy = NeverBusy> {
    entry: B,
    busy: Q,
    pending: bool,
}

impl<B: BootloaderEntry> RebootVendor<B> {
    /// Build the handler over a board-specific [`BootloaderEntry`], with no flash-busy gate.
    pub fn new(entry: B) -> Self {
        Self {
            entry,
            busy: NeverBusy,
            pending: false,
        }
    }
}

impl<B: BootloaderEntry, Q: FlashBusy> RebootVendor<B, Q> {
    /// Build the handler over a [`BootloaderEntry`] and an injected [`FlashBusy`] query.
    /// A reboot is declined with [`STATUS_BUSY`] whenever `busy` reports a live flash session.
    pub fn with_busy_check(entry: B, busy: Q) -> Self {
        Self {
            entry,
            busy,
            pending: false,
        }
    }

    /// Whether a valid reboot request has been latched (the firmware's thread-mode trigger
    /// performs the ROM entry after a grace delay).
    pub fn pending(&self) -> bool {
        self.pending
    }

    /// Shared access to the injected entry mechanism (e.g. to call
    /// [`enter`](BootloaderEntry::enter) from the firmware's thread-mode trigger).
    pub fn entry(&self) -> &B {
        &self.entry
    }
}

impl<B: BootloaderEntry, Q: FlashBusy> VendorHandler for RebootVendor<B, Q> {
    fn handle(&mut self, report: &[u8], response: &mut [u8]) -> Option<usize> {
        // `response[0]` is pre-echoed by the facade; fill from byte 1, return the TOTAL length.
        match report {
            [REBOOT_CMD_ID, SUB_DISCOVER, ..] => {
                if response.len() < 7 {
                    return None;
                }
                response[1..5].copy_from_slice(&REPLY_MAGIC);
                response[5] = REBOOT_VERSION;
                response[6] = B::KIND;
                Some(7)
            }
            [REBOOT_CMD_ID, SUB_REBOOT, b'B', b'O', b'O', b'T', ..] if B::KIND != KIND_NONE => {
                if response.len() < 2 {
                    return None;
                }
                // A live flash session must not lose the bus mid-algorithm: decline without arming.
                if self.busy.is_flashing() {
                    response[1] = STATUS_BUSY;
                    return Some(2);
                }
                self.pending = true;
                self.entry.arm();
                response[1] = STATUS_ARMED;
                Some(2)
            }
            [REBOOT_CMD_ID, SUB_REBOOT, ..] => {
                if response.len() < 2 {
                    return None;
                }
                // Missing/wrong magic, or a board with no ROM path: a self-describing status byte.
                response[1] = if B::KIND == KIND_NONE {
                    STATUS_UNSUPPORTED
                } else {
                    STATUS_BAD_MAGIC
                };
                Some(2)
            }
            // Unknown subcommand or truncated packet: decline, so the engine answers `0xFF`.
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A mock entry that records whether it was armed, standing in for a real ROM path.
    struct MockEntry {
        armed: core::cell::Cell<bool>,
    }
    impl MockEntry {
        fn new() -> Self {
            Self {
                armed: core::cell::Cell::new(false),
            }
        }
    }
    impl BootloaderEntry for MockEntry {
        const KIND: u8 = KIND_UF2;
        fn arm(&self) {
            self.armed.set(true);
        }
        fn enter(&self) -> ! {
            unreachable!("enter must never run in the host test")
        }
    }

    /// An entry reporting `kind = none`, for the unsupported-board path.
    struct NoneEntry;
    impl BootloaderEntry for NoneEntry {
        const KIND: u8 = KIND_NONE;
        fn enter(&self) -> ! {
            unreachable!()
        }
    }

    /// frozen: the reboot vendor id and its status bytes are the wire contract a host tool
    /// hard-codes; a renumber of the id or of any status byte must fail here.
    #[test]
    fn reboot_id_and_status_bytes_are_pinned() {
        assert_eq!(REBOOT_CMD_ID, 0x9F);
        assert_eq!(STATUS_ARMED, 0x00);
        assert_eq!(STATUS_UNSUPPORTED, 0x01);
        assert_eq!(STATUS_BAD_MAGIC, 0x02);
        assert_eq!(STATUS_BUSY, 0x03);

        // Discovery reply framing, run against the real handler: id, magic, version, kind.
        let mut v = RebootVendor::new(MockEntry::new());
        let mut resp = [0u8; 64];
        resp[0] = REBOOT_CMD_ID;
        let len = v.handle(&[REBOOT_CMD_ID, SUB_DISCOVER], &mut resp).unwrap();
        assert_eq!(len, 7);
        assert_eq!(
            &resp[..7],
            &[
                REBOOT_CMD_ID,
                b'd',
                b'l',
                b'r',
                b's',
                REBOOT_VERSION,
                KIND_UF2
            ]
        );

        // A valid reboot request replies with the echoed id plus STATUS_ARMED.
        let len = v
            .handle(
                &[REBOOT_CMD_ID, SUB_REBOOT, b'B', b'O', b'O', b'T'],
                &mut resp,
            )
            .unwrap();
        assert_eq!(len, 2);
        assert_eq!(&resp[..2], &[REBOOT_CMD_ID, STATUS_ARMED]);
    }

    #[test]
    fn discovery_reports_magic_version_and_kind() {
        let mut v = RebootVendor::new(MockEntry::new());
        let mut resp = [0u8; 64];
        resp[0] = REBOOT_CMD_ID; // facade pre-echo
        let len = v.handle(&[REBOOT_CMD_ID, SUB_DISCOVER], &mut resp).unwrap();
        assert_eq!(len, 7);
        assert_eq!(&resp[1..5], b"dlrs");
        assert_eq!(resp[5], REBOOT_VERSION);
        assert_eq!(resp[6], KIND_UF2);
        assert!(!v.pending(), "discovery must not arm a reboot");
    }

    #[test]
    fn valid_magic_arms_and_latches_pending() {
        let mut v = RebootVendor::new(MockEntry::new());
        let mut resp = [0u8; 64];
        resp[0] = REBOOT_CMD_ID;
        let len = v
            .handle(
                &[REBOOT_CMD_ID, SUB_REBOOT, b'B', b'O', b'O', b'T'],
                &mut resp,
            )
            .unwrap();
        assert_eq!(len, 2);
        assert_eq!(resp[1], STATUS_ARMED);
        assert!(v.pending(), "a valid reboot request latches pending");
        assert!(v.entry().armed.get(), "the entry must be armed");
    }

    #[test]
    fn wrong_magic_is_rejected_without_arming() {
        let mut v = RebootVendor::new(MockEntry::new());
        let mut resp = [0u8; 64];
        resp[0] = REBOOT_CMD_ID;
        let len = v
            .handle(
                &[REBOOT_CMD_ID, SUB_REBOOT, b'N', b'O', b'P', b'E'],
                &mut resp,
            )
            .unwrap();
        assert_eq!(len, 2);
        assert_eq!(resp[1], STATUS_BAD_MAGIC);
        assert!(!v.pending(), "a bad magic must not arm a reboot");
    }

    #[test]
    fn truncated_reboot_is_bad_magic() {
        let mut v = RebootVendor::new(MockEntry::new());
        let mut resp = [0u8; 64];
        resp[0] = REBOOT_CMD_ID;
        let len = v.handle(&[REBOOT_CMD_ID, SUB_REBOOT], &mut resp).unwrap();
        assert_eq!(resp[1], STATUS_BAD_MAGIC);
        assert_eq!(len, 2);
        assert!(!v.pending());
    }

    #[test]
    fn none_kind_reports_unsupported_and_never_arms() {
        let mut v = RebootVendor::new(NoneEntry);
        let mut resp = [0u8; 64];
        resp[0] = REBOOT_CMD_ID;
        // Discovery still answers, with kind = none.
        let len = v.handle(&[REBOOT_CMD_ID, SUB_DISCOVER], &mut resp).unwrap();
        assert_eq!(resp[6], KIND_NONE);
        assert_eq!(len, 7);
        // Even with the correct magic, an unsupported board declines.
        let len = v
            .handle(
                &[REBOOT_CMD_ID, SUB_REBOOT, b'B', b'O', b'O', b'T'],
                &mut resp,
            )
            .unwrap();
        assert_eq!(resp[1], STATUS_UNSUPPORTED);
        assert_eq!(len, 2);
        assert!(!v.pending());
    }

    #[test]
    fn unknown_subcommand_declines_to_engine() {
        let mut v = RebootVendor::new(MockEntry::new());
        let mut resp = [0u8; 64];
        resp[0] = REBOOT_CMD_ID;
        assert_eq!(v.handle(&[REBOOT_CMD_ID, 0x7F], &mut resp), None);
        assert!(!v.pending());
    }

    /// A toggleable flash-busy query standing in for the mux's live-session flag.
    struct MockBusy {
        flashing: core::cell::Cell<bool>,
    }
    impl FlashBusy for MockBusy {
        fn is_flashing(&self) -> bool {
            self.flashing.get()
        }
    }

    /// A live flash session declines the reboot with STATUS_BUSY and does not arm the entry.
    #[test]
    fn busy_declines_reboot_without_arming() {
        let busy = MockBusy {
            flashing: core::cell::Cell::new(true),
        };
        let mut v = RebootVendor::with_busy_check(MockEntry::new(), busy);
        let mut resp = [0u8; 64];
        resp[0] = REBOOT_CMD_ID;
        let len = v
            .handle(
                &[REBOOT_CMD_ID, SUB_REBOOT, b'B', b'O', b'O', b'T'],
                &mut resp,
            )
            .unwrap();
        assert_eq!(len, 2);
        assert_eq!(resp[1], STATUS_BUSY);
        assert!(!v.pending(), "a busy probe must not latch a reboot");
        assert!(
            !v.entry().armed.get(),
            "a busy probe must not arm the entry"
        );
    }

    /// The same handler arms normally once the flash session clears (the query reports idle).
    #[test]
    fn idle_after_busy_arms_normally() {
        let busy = MockBusy {
            flashing: core::cell::Cell::new(true),
        };
        let mut v = RebootVendor::with_busy_check(MockEntry::new(), busy);
        let mut resp = [0u8; 64];
        resp[0] = REBOOT_CMD_ID;

        // First attempt while busy: declined.
        v.handle(
            &[REBOOT_CMD_ID, SUB_REBOOT, b'B', b'O', b'O', b'T'],
            &mut resp,
        )
        .unwrap();
        assert_eq!(resp[1], STATUS_BUSY);

        // The session ends; the retry arms.
        v.busy.flashing.set(false);
        let len = v
            .handle(
                &[REBOOT_CMD_ID, SUB_REBOOT, b'B', b'O', b'O', b'T'],
                &mut resp,
            )
            .unwrap();
        assert_eq!(len, 2);
        assert_eq!(resp[1], STATUS_ARMED);
        assert!(v.pending());
        assert!(v.entry().armed.get());
    }

    /// A `Fn() -> bool` closure is accepted as the busy query (the ergonomic non-mux injection).
    #[test]
    fn closure_busy_query_declines() {
        let mut v = RebootVendor::with_busy_check(MockEntry::new(), || true);
        let mut resp = [0u8; 64];
        resp[0] = REBOOT_CMD_ID;
        v.handle(
            &[REBOOT_CMD_ID, SUB_REBOOT, b'B', b'O', b'O', b'T'],
            &mut resp,
        )
        .unwrap();
        assert_eq!(resp[1], STATUS_BUSY);
        assert!(!v.pending());
    }

    /// A `&'static SwdMux` is itself the busy query, with no adapter closure.
    #[test]
    fn static_mux_reference_is_a_flash_busy_query() {
        use crate::flash_mux::FlashMux;
        use dap_rs::driver::bitbang::{BitbangAdapter, DelayCycles, InputOutputPin};
        use dap_rs::jtag::TapConfig;
        use embedded_hal::delay::DelayNs;
        use static_cell::StaticCell;

        struct Pin;
        impl InputOutputPin for Pin {
            fn set_as_output(&mut self) {}
            fn set_high(&mut self, _high: bool) {}
            fn set_as_input(&mut self) {}
            fn is_high(&mut self) -> bool {
                false
            }
        }
        #[derive(Clone, Copy)]
        struct Delay;
        impl DelayNs for Delay {
            fn delay_ns(&mut self, _ns: u32) {}
        }
        impl DelayCycles for Delay {
            fn cpu_clock(&self) -> u32 {
                1_000_000
            }
            fn delay_cycles(&mut self, _cycles: u32) {}
        }

        static SCAN: StaticCell<[TapConfig; 1]> = StaticCell::new();
        static MUX: StaticCell<FlashMux<Pin, Delay>> = StaticCell::new();
        let adapter = BitbangAdapter::new(
            Pin,
            Pin,
            Pin,
            Pin,
            Pin,
            Delay,
            SCAN.init([TapConfig::INIT; 1]),
        );
        let mux: &'static FlashMux<Pin, Delay> = MUX.init(SwdMux::new(adapter));

        let mut v = RebootVendor::with_busy_check(MockEntry::new(), mux);
        let mut resp = [0u8; 64];
        resp[0] = REBOOT_CMD_ID;

        // Idle: arms normally.
        v.handle(
            &[REBOOT_CMD_ID, SUB_REBOOT, b'B', b'O', b'O', b'T'],
            &mut resp,
        )
        .unwrap();
        assert_eq!(resp[1], STATUS_ARMED);

        // A live flash session declines a fresh handler.
        let mut v2 = RebootVendor::with_busy_check(MockEntry::new(), mux);
        let session = mux.begin_flash().unwrap();
        v2.handle(
            &[REBOOT_CMD_ID, SUB_REBOOT, b'B', b'O', b'O', b'T'],
            &mut resp,
        )
        .unwrap();
        assert_eq!(resp[1], STATUS_BUSY);
        drop(session);
    }
}
