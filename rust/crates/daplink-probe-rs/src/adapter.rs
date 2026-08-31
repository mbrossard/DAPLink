//! The `dap-rs` SWD + reset seam implemented over probe-rs raw DAP access.

use daplink_flash::dap_rs::swd::{APnDP, Config, DPRegister, Error, Swd};
use daplink_flash::reset::ResetControl;
use probe_rs::architecture::arm::dp::DpRegisterAddress;
use probe_rs::architecture::arm::{ArmError, DapError, RawDapAccess, RegisterAddress};

/// nRESET occupies bit 7 of the CMSIS-DAP pin mask (`DAP_SWJ_Pins`).
const NRESET_BIT: u32 = 1 << 7;

/// Zero-sized `DEPS` marker for the [`Swd`] trait.
///
/// The `dap-rs` [`Swd`] trait requires `Self: From<DEPS>`; this type satisfies that bound. It
/// carries no state -- a [`ProbeRsSwd`] is always built with [`ProbeRsSwd::new`], never via the
/// `From` conversion.
pub struct ProbeRsDeps;

impl From<ProbeRsDeps> for ProbeRsSwd<'_> {
    fn from(_: ProbeRsDeps) -> Self {
        // The `Swd` trait's `From<DEPS>` bound is never exercised for this adapter, which owns a
        // borrowed probe interface and can only be built through `new`.
        panic!("ProbeRsSwd must be constructed with ProbeRsSwd::new");
    }
}

/// An SWD transport backed by a probe-rs raw DAP interface.
///
/// Wraps a borrowed [`RawDapAccess`] probe and forwards `dap-rs` register and sequence
/// operations to it, translating between the seam's addressing model and probe-rs's.
pub struct ProbeRsSwd<'a> {
    interface: &'a mut dyn RawDapAccess,
    config: Config,
    /// Value of the most recent AP read, not yet handed to the caller. daplink-flash is written
    /// against raw-SWD posted-read semantics -- an AP read returns the PREVIOUS read's data and
    /// `RDBUFF` collects the last one -- but probe-rs's `raw_read_register(AP)` de-posts
    /// internally and returns the current value directly. `read_inner` therefore replays the
    /// posting: an AP read caches the fresh value and returns the previously cached one, and an
    /// `RDBUFF` read returns the cache. Returning the fresh value directly instead shifts every
    /// pipelined block read by one word.
    last_ap_read: u32,
}

impl<'a> ProbeRsSwd<'a> {
    /// Wrap a probe-rs raw DAP interface as an SWD transport.
    pub fn new(interface: &'a mut dyn RawDapAccess) -> Self {
        Self {
            interface,
            config: Config::default(),
            last_ap_read: 0,
        }
    }
}

/// Build the probe-rs byte register address for a `dap-rs` access.
///
/// The seam's [`DPRegister`] value is the ADIv5 `A[3:2]` selector (0..=3); shifting it left by
/// two recovers the register byte offset that probe-rs expects. AP accesses use
/// [`RegisterAddress::ApRegister`]; DP accesses use a bank-0 [`DpRegisterAddress`].
fn register_address(apndp: APnDP, a: DPRegister) -> RegisterAddress {
    let byte = (a as u8) << 2;
    match apndp {
        APnDP::AP => RegisterAddress::ApRegister(byte),
        APnDP::DP => RegisterAddress::DpRegister(DpRegisterAddress {
            address: byte,
            bank: None,
        }),
    }
}

/// Map a probe-rs [`ArmError`] onto the `dap-rs` SWD [`Error`] set.
fn map_error(err: ArmError) -> Error {
    match err {
        ArmError::Dap(DapError::WaitResponse) => Error::AckWait,
        ArmError::Dap(DapError::FaultResponse) => Error::AckFault,
        ArmError::Dap(DapError::NoAcknowledge) => Error::NoAck,
        ArmError::Dap(DapError::IncorrectParity) => Error::BadParity,
        _ => Error::AckProtocol,
    }
}

impl Swd<ProbeRsDeps> for ProbeRsSwd<'_> {
    fn available(_deps: &ProbeRsDeps) -> bool {
        true
    }

    fn config(&mut self) -> &mut Config {
        &mut self.config
    }

    fn set_clock(&mut self, _max_frequency: u32) -> bool {
        true
    }

    fn read_inner(&mut self, apndp: APnDP, a: DPRegister) -> Result<u32, Error> {
        // Replay raw posted-read semantics over probe-rs's de-posted interface (see
        // `last_ap_read`): RDBUFF collects the cached value without a bus read, an AP read
        // caches the fresh value and returns the previous one, DP reads pass through.
        if matches!(apndp, APnDP::DP) && matches!(a, DPRegister::RDBUFF) {
            return Ok(self.last_ap_read);
        }
        let addr = register_address(apndp, a);
        let value = self.interface.raw_read_register(addr).map_err(map_error)?;
        if matches!(apndp, APnDP::AP) {
            return Ok(core::mem::replace(&mut self.last_ap_read, value));
        }
        Ok(value)
    }

    fn write_inner(&mut self, apndp: APnDP, a: DPRegister, data: u32) -> Result<(), Error> {
        let addr = register_address(apndp, a);
        self.interface
            .raw_write_register(addr, data)
            .map_err(map_error)
    }

    fn write_sequence(&mut self, num_bits: usize, data: &[u8]) -> Result<(), Error> {
        debug_assert!(
            num_bits <= data.len() * 8,
            "write_sequence: num_bits exceeds data"
        );
        for (chunk_bits, bits) in sequence_chunks(num_bits, data) {
            self.interface
                .swj_sequence(chunk_bits, bits)
                .map_err(|_| Error::AckProtocol)?;
        }
        Ok(())
    }

    fn read_sequence(&mut self, _num_bits: usize, _data: &mut [u8]) -> Result<(), Error> {
        // probe-rs exposes no raw SWD sequence read; the flash path never needs one.
        Err(Error::AckProtocol)
    }
}

impl ResetControl for ProbeRsSwd<'_> {
    type Error = Error;

    fn set_reset(&mut self, asserted: bool) -> Result<(), Self::Error> {
        // nRESET is active-low: hold in reset -> drive the line low (0), release -> high.
        let pin_out = if asserted { 0 } else { NRESET_BIT };
        // Best-effort: probes without pin control simply report an error, which we ignore.
        let _ = self.interface.swj_pins(pin_out, NRESET_BIT, 0);
        Ok(())
    }
}

/// Split an LSB-first bit sequence into `swj_sequence`-sized chunks (at most 64 bits per call):
/// `(bit_count, bits)` pairs packed LSB-first (byte 0 in the low bits), the last pair carrying
/// the remainder. Precondition: `num_bits <= data.len() * 8` (caller `debug_assert!`s it); bits
/// beyond `num_bits` in the final byte are left unmasked, matching `swj_sequence`'s own
/// tolerance for don't-care high bits.
fn sequence_chunks(num_bits: usize, data: &[u8]) -> impl Iterator<Item = (u8, u64)> + '_ {
    let mut remaining = num_bits;
    data.chunks(8).map_while(move |chunk| {
        if remaining == 0 {
            return None;
        }
        let chunk_bits = remaining.min(chunk.len() * 8).min(64);
        let mut bits: u64 = 0;
        for (i, byte) in chunk.iter().enumerate() {
            bits |= u64::from(*byte) << (i * 8);
        }
        remaining -= chunk_bits;
        Some((chunk_bits as u8, bits))
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use probe_rs::probe::{DebugProbe, DebugProbeError};
    use probe_rs::CoreStatus;

    /// A minimal [`RawDapAccess`] stand-in that hands back a fixed value for AP reads and a
    /// different fixed value for (non-`RDBUFF`) DP reads, so a test can tell which path a read
    /// actually took. Only the read/write register calls the SWD seam issues are meaningful here;
    /// the rest of the trait is never exercised by [`ProbeRsSwd`] in this test.
    struct MockDap {
        ap_value: u32,
        /// Added to `ap_value` after every AP read, modeling a TAR auto-increment word stream.
        ap_step: u32,
        dp_value: u32,
    }

    impl RawDapAccess for MockDap {
        fn raw_read_register(&mut self, address: RegisterAddress) -> Result<u32, ArmError> {
            Ok(if address.is_ap() {
                let value = self.ap_value;
                self.ap_value = self.ap_value.wrapping_add(self.ap_step);
                value
            } else {
                self.dp_value
            })
        }

        fn raw_write_register(
            &mut self,
            _address: RegisterAddress,
            _value: u32,
        ) -> Result<(), ArmError> {
            Ok(())
        }

        fn jtag_sequence(
            &mut self,
            _cycles: u8,
            _tms: bool,
            _tdi: u64,
        ) -> Result<(), DebugProbeError> {
            Ok(())
        }

        fn swj_sequence(&mut self, _bit_len: u8, _bits: u64) -> Result<(), DebugProbeError> {
            Ok(())
        }

        fn swj_pins(
            &mut self,
            _pin_out: u32,
            _pin_select: u32,
            _pin_wait: u32,
        ) -> Result<u32, DebugProbeError> {
            Ok(0)
        }

        fn into_probe(self: Box<Self>) -> Box<dyn DebugProbe> {
            unimplemented!("not exercised by the posted-read cache test")
        }

        fn core_status_notification(&mut self, _state: CoreStatus) -> Result<(), DebugProbeError> {
            Ok(())
        }
    }

    #[test]
    fn posted_ap_read_returns_previous_and_rdbuff_collects_the_last() {
        // Pins the posted-read replay `last_ap_read` documents: an AP read returns the PREVIOUS
        // read's value (the first returns the initial cache, which callers discard) and RDBUFF
        // collects the cached value without a bus read.
        let mut mock = MockDap {
            ap_value: 0xAAAA_5555,
            ap_step: 0,
            dp_value: 0x1111_2222,
        };
        let mut swd = ProbeRsSwd::new(&mut mock);

        // First AP read: posting primes the pipeline; the returned word is discard fodder.
        let primed = swd
            .read_inner(APnDP::AP, DPRegister::SELECT)
            .expect("ap read");
        assert_eq!(primed, 0);

        // RDBUFF must return the cached posted value, not a fresh DP-register bus read (which
        // would surface dp_value instead).
        let rdbuff = swd
            .read_inner(APnDP::DP, DPRegister::RDBUFF)
            .expect("rdbuff read");
        assert_eq!(rdbuff, 0xAAAA_5555);

        // An intervening non-RDBUFF DP read must not disturb the cached AP value.
        let dp = swd
            .read_inner(APnDP::DP, DPRegister::CTRLSTAT)
            .expect("dp read");
        assert_eq!(dp, 0x1111_2222);

        let rdbuff_again = swd
            .read_inner(APnDP::DP, DPRegister::RDBUFF)
            .expect("rdbuff read");
        assert_eq!(
            rdbuff_again, 0xAAAA_5555,
            "cached AP value must survive an intervening DP read"
        );
    }

    #[test]
    fn pipelined_block_read_is_not_shifted() {
        // Regression: a de-posted transport handed the fresh word back on each AP read, so the
        // prime discarded word 0 and every block read arrived shifted one word up.
        let mut mock = MockDap {
            ap_value: 0x100,
            ap_step: 1,
            dp_value: 0,
        };
        let mut swd = ProbeRsSwd::new(&mut mock);

        // The flash host's pipeline for a 4-word block: prime, three AP reads, RDBUFF close.
        let _ = swd
            .read_inner(APnDP::AP, DPRegister::RDBUFF)
            .expect("prime");
        let mut words = [0u32; 4];
        for slot in &mut words[..3] {
            *slot = swd
                .read_inner(APnDP::AP, DPRegister::RDBUFF)
                .expect("ap read");
        }
        words[3] = swd
            .read_inner(APnDP::DP, DPRegister::RDBUFF)
            .expect("rdbuff read");
        assert_eq!(words, [0x100, 0x101, 0x102, 0x103]);
    }

    #[test]
    fn availability_is_reported() {
        assert!(<ProbeRsSwd as Swd<ProbeRsDeps>>::available(&ProbeRsDeps));
    }

    #[test]
    fn dp_register_address_shifts_selector() {
        let addr = register_address(APnDP::DP, DPRegister::SELECT);
        assert!(!addr.is_ap());
        // SELECT selector is 2 -> byte offset 0x8.
        assert_eq!(addr.lsb(), 0x8);
    }

    #[test]
    fn ap_register_address_shifts_selector() {
        let addr = register_address(APnDP::AP, DPRegister::RDBUFF);
        assert!(addr.is_ap());
        // RDBUFF selector is 3 -> byte offset 0xC.
        assert_eq!(addr.lsb(), 0xC);
    }

    #[test]
    fn line_reset_chunks_match_swd_host_init_debug() {
        // Mirrors the exact (num_bits, data) pairs daplink-flash's swd_host::init_debug feeds
        // to write_sequence: two >=50-clock line resets, the JTAG-to-SWD switch word, and an
        // idle period.
        let ones = [0xFFu8; 8];
        let chunks: Vec<_> = sequence_chunks(51, &ones).collect();
        assert_eq!(chunks, vec![(51, 0xFFFF_FFFF_FFFF_FFFFu64)]);

        let switch = 0xE79Eu16.to_le_bytes();
        let chunks: Vec<_> = sequence_chunks(16, &switch).collect();
        assert_eq!(chunks, vec![(16, 0xE79Eu64)]);

        let chunks: Vec<_> = sequence_chunks(51, &ones).collect();
        assert_eq!(chunks, vec![(51, 0xFFFF_FFFF_FFFF_FFFFu64)]);

        let idle = [0x00u8; 2];
        let chunks: Vec<_> = sequence_chunks(8, &idle).collect();
        assert_eq!(chunks, vec![(8, 0u64)]);
    }

    #[test]
    fn over_64_bits_splits_into_two_chunks() {
        // 100 bits from 13 bytes: a full 64-bit chunk, then a 36-bit remainder.
        let mut data = [0u8; 13];
        for (i, byte) in data.iter_mut().enumerate() {
            *byte = i as u8 + 1;
        }
        let chunks: Vec<_> = sequence_chunks(100, &data).collect();
        assert_eq!(chunks.len(), 2);
        assert_eq!(chunks[0].0, 64);
        assert_eq!(chunks[1].0, 36);

        let mut expected_first: u64 = 0;
        for (i, byte) in data[..8].iter().enumerate() {
            expected_first |= u64::from(*byte) << (i * 8);
        }
        assert_eq!(chunks[0].1, expected_first);

        let mut expected_second: u64 = 0;
        for (i, byte) in data[8..13].iter().enumerate() {
            expected_second |= u64::from(*byte) << (i * 8);
        }
        assert_eq!(chunks[1].1, expected_second);
    }

    #[test]
    fn non_multiple_of_8_tail_yields_single_chunk() {
        // 12 bits from 2 bytes: one chunk carrying the low 12 bits (of a full 16-bit pack).
        let data = [0xABu8, 0xCDu8];
        let chunks: Vec<_> = sequence_chunks(12, &data).collect();
        assert_eq!(chunks, vec![(12, 0xCDABu64)]);
    }
}
