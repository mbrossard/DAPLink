//! The `dap-rs` SWD + reset seam implemented over probe-rs raw DAP access.

use daplink_flash::dap_rs::swd::{APnDP, Config, DPRegister, Error, Swd};
use daplink_flash::ResetControl;
use probe_rs::architecture::arm::dp::DpRegisterAddress;
use probe_rs::architecture::arm::{ArmError, DapError, RawDapAccess, RegisterAddress};

/// nRESET occupies bit 7 of the CMSIS-DAP pin mask (`DAP_SWJ_Pins`).
const NRESET_BIT: u32 = 1 << 7;

/// Zero-sized `DEPS` marker satisfying the `dap-rs` [`Swd`] `From<DEPS>` bound; never converted.
pub struct ProbeRsDeps;

impl From<ProbeRsDeps> for ProbeRsSwd<'_> {
    /// # Panics
    /// Always: build the transport with [`ProbeRsSwd::new`].
    fn from(_: ProbeRsDeps) -> Self {
        panic!("ProbeRsSwd must be constructed with ProbeRsSwd::new");
    }
}

/// An SWD transport over a borrowed, already configured probe-rs [`RawDapAccess`], with its
/// de-posted AP reads replayed as posted; clock control and `read_sequence` are unsupported.
pub struct ProbeRsSwd<'a> {
    interface: &'a mut dyn RawDapAccess,
    config: Config,
    /// The last AP read, replayed so probe-rs's de-posted reads look posted to daplink-flash.
    last_ap_read: u32,
    transport_error: Option<ArmError>,
}

impl<'a> ProbeRsSwd<'a> {
    /// Wrap a probe-rs raw DAP interface as an SWD transport.
    pub fn new(interface: &'a mut dyn RawDapAccess) -> Self {
        Self {
            interface,
            config: Config::default(),
            last_ap_read: 0,
            transport_error: None,
        }
    }

    /// The last probe-rs failure that was not an SWD ACK, which `dap-rs` sees as `AckProtocol`.
    pub fn take_transport_error(&mut self) -> Option<ArmError> {
        self.transport_error.take()
    }

    /// Map a probe-rs [`ArmError`] onto the `dap-rs` SWD [`Error`] set, keeping a transport cause.
    fn map_error(&mut self, err: ArmError) -> Error {
        match err {
            ArmError::Dap(DapError::WaitResponse) => Error::AckWait,
            ArmError::Dap(DapError::FaultResponse) => Error::AckFault,
            ArmError::Dap(DapError::NoAcknowledge) => Error::NoAck,
            ArmError::Dap(DapError::IncorrectParity) => Error::BadParity,
            other => {
                self.transport_error = Some(other);
                Error::AckProtocol
            }
        }
    }
}

/// The probe-rs register address for an ADIv5 `A[3:2]` selector, shifted left by two.
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

impl Swd<ProbeRsDeps> for ProbeRsSwd<'_> {
    fn available(_deps: &ProbeRsDeps) -> bool {
        true
    }

    fn config(&mut self) -> &mut Config {
        &mut self.config
    }

    /// Returns false without changing the externally configured probe speed.
    fn set_clock(&mut self, _max_frequency: u32) -> bool {
        false
    }

    fn read_inner(&mut self, apndp: APnDP, a: DPRegister) -> Result<u32, Error> {
        // Replay posted-read semantics (see `last_ap_read`).
        if matches!(apndp, APnDP::DP) && matches!(a, DPRegister::RDBUFF) {
            return Ok(self.last_ap_read);
        }
        let addr = register_address(apndp, a);
        let value = match self.interface.raw_read_register(addr) {
            Ok(value) => value,
            Err(err) => return Err(self.map_error(err)),
        };
        if matches!(apndp, APnDP::AP) {
            return Ok(core::mem::replace(&mut self.last_ap_read, value));
        }
        Ok(value)
    }

    fn write_inner(&mut self, apndp: APnDP, a: DPRegister, data: u32) -> Result<(), Error> {
        let addr = register_address(apndp, a);
        self.interface
            .raw_write_register(addr, data)
            .map_err(|err| self.map_error(err))
    }

    fn write_sequence(&mut self, num_bits: usize, data: &[u8]) -> Result<(), Error> {
        if num_bits > data.len() * 8 {
            return Err(Error::AckProtocol);
        }
        for (chunk_bits, bits) in sequence_chunks(num_bits, data) {
            if let Err(err) = self.interface.swj_sequence(chunk_bits, bits) {
                return Err(self.map_error(err.into()));
            }
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
        match self.interface.swj_pins(pin_out, NRESET_BIT, 0) {
            Ok(_) => Ok(()),
            Err(err) => Err(self.map_error(err.into())),
        }
    }
}

/// Split an LSB-first bit sequence into `swj_sequence` chunks of at most 64 bits.
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
    use daplink_flash::target::{ResetType, TargetState};
    use daplink_flash::SwdHost;
    use probe_rs::probe::{DebugProbe, DebugProbeError};
    use probe_rs::CoreStatus;

    struct NoDelay;

    impl embedded_hal::delay::DelayNs for NoDelay {
        fn delay_ns(&mut self, _ns: u32) {}
    }

    /// A [`RawDapAccess`] stand-in with distinct fixed AP and DP read values.
    struct MockDap {
        ap_value: u32,
        /// Added to `ap_value` after every AP read, modeling a TAR auto-increment word stream.
        ap_step: u32,
        dp_value: u32,
        /// AP reads that succeed before every later one FAULTs; `None` never faults.
        ap_reads_before_fault: Option<u32>,
        /// Fail every `swj_pins` call as the probe's USB link would.
        fail_pins: bool,
        /// Fail every register access as the probe's USB link would.
        fail_registers: bool,
        /// Bits clocked by `swj_sequence`, in call order.
        clocked: Vec<u8>,
        accesses: usize,
    }

    impl MockDap {
        fn new() -> Self {
            Self {
                ap_value: 0,
                ap_step: 0,
                dp_value: 0,
                ap_reads_before_fault: None,
                fail_pins: false,
                fail_registers: false,
                clocked: Vec::new(),
                accesses: 0,
            }
        }
    }

    fn usb_error() -> DebugProbeError {
        DebugProbeError::Usb(std::io::Error::other("unplugged"))
    }

    impl RawDapAccess for MockDap {
        fn raw_read_register(&mut self, address: RegisterAddress) -> Result<u32, ArmError> {
            self.accesses += 1;
            if self.fail_registers {
                return Err(ArmError::Probe(usb_error()));
            }
            if address.is_ap() {
                match &mut self.ap_reads_before_fault {
                    Some(0) => return Err(ArmError::Dap(DapError::FaultResponse)),
                    Some(n) => *n -= 1,
                    None => {}
                }
            }
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
            self.accesses += 1;
            Ok(())
        }

        fn jtag_sequence(
            &mut self,
            _cycles: u8,
            _tms: bool,
            _tdi: u64,
        ) -> Result<(), DebugProbeError> {
            self.accesses += 1;
            Ok(())
        }

        fn swj_sequence(&mut self, bit_len: u8, _bits: u64) -> Result<(), DebugProbeError> {
            self.accesses += 1;
            self.clocked.push(bit_len);
            Ok(())
        }

        fn swj_pins(
            &mut self,
            _pin_out: u32,
            _pin_select: u32,
            _pin_wait: u32,
        ) -> Result<u32, DebugProbeError> {
            self.accesses += 1;
            if self.fail_pins {
                return Err(usb_error());
            }
            Ok(0)
        }

        fn into_probe(self: Box<Self>) -> Box<dyn DebugProbe> {
            unimplemented!("not exercised by the posted-read cache test")
        }

        fn core_status_notification(&mut self, _state: CoreStatus) -> Result<(), DebugProbeError> {
            self.accesses += 1;
            Ok(())
        }
    }

    #[test]
    fn clock_control_is_unsupported_without_transport_or_cache_changes() {
        let mut mock = MockDap::new();
        mock.ap_value = 0xAAAA_5555;
        let mut swd = ProbeRsSwd::new(&mut mock);
        swd.read_inner(APnDP::AP, DPRegister::SELECT)
            .expect("prime posted read");

        for frequency in [0, 1, 1_000_000, 25_000_000, u32::MAX] {
            assert!(!swd.set_clock(frequency));
            assert!(swd.take_transport_error().is_none());
            assert_eq!(
                swd.read_inner(APnDP::DP, DPRegister::RDBUFF),
                Ok(0xAAAA_5555)
            );
        }
        assert_eq!(
            mock.accesses, 1,
            "only the priming AP read reaches the probe"
        );
    }

    #[test]
    fn posted_ap_read_returns_previous_and_rdbuff_collects_the_last() {
        // An AP read returns the previous value; RDBUFF collects the cache without a bus read.
        let mut mock = MockDap {
            ap_value: 0xAAAA_5555,
            ap_step: 0,
            dp_value: 0x1111_2222,
            ap_reads_before_fault: None,
            ..MockDap::new()
        };
        let mut swd = ProbeRsSwd::new(&mut mock);

        let primed = swd
            .read_inner(APnDP::AP, DPRegister::SELECT)
            .expect("ap read");
        assert_eq!(primed, 0);

        // RDBUFF must return the cached posted value, not a fresh DP read.
        let rdbuff = swd
            .read_inner(APnDP::DP, DPRegister::RDBUFF)
            .expect("rdbuff read");
        assert_eq!(rdbuff, 0xAAAA_5555);

        // Non-RDBUFF DP reads must not disturb the cached AP value.
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
        // A de-posted transport would shift every block read one word up.
        let mut mock = MockDap {
            ap_value: 0x100,
            ap_step: 1,
            dp_value: 0,
            ap_reads_before_fault: None,
            ..MockDap::new()
        };
        let mut swd = ProbeRsSwd::new(&mut mock);

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
    fn a_faulted_ap_read_keeps_the_posted_value() {
        let mut mock = MockDap {
            ap_value: 0xAAAA_5555,
            ap_step: 0,
            dp_value: 0,
            ap_reads_before_fault: Some(1),
            ..MockDap::new()
        };
        let mut swd = ProbeRsSwd::new(&mut mock);
        swd.read_inner(APnDP::AP, DPRegister::SELECT)
            .expect("ap read");
        assert_eq!(
            swd.read_inner(APnDP::AP, DPRegister::SELECT),
            Err(Error::AckFault)
        );
        assert_eq!(
            swd.read_inner(APnDP::DP, DPRegister::RDBUFF),
            Ok(0xAAAA_5555)
        );
    }

    #[test]
    fn a_failing_swj_pins_surfaces_as_reset_failed() {
        let mut mock = MockDap::new();
        mock.fail_pins = true;
        let mut host: SwdHost<_, _, ProbeRsDeps> =
            SwdHost::new(ProbeRsSwd::new(&mut mock), NoDelay);
        assert_eq!(
            host.set_target_state(TargetState::ResetHold, ResetType::Hardware),
            Err(daplink_flash::Error::ResetFailed)
        );
    }

    #[test]
    fn a_sequence_longer_than_its_slice_is_refused_unclocked() {
        let mut mock = MockDap::new();
        let mut swd = ProbeRsSwd::new(&mut mock);
        assert_eq!(swd.write_sequence(16, &[0xFF]), Err(Error::AckProtocol));
        assert!(mock.clocked.is_empty());
    }

    #[test]
    fn a_transport_failure_is_kept_apart_from_a_protocol_ack() {
        let mut mock = MockDap::new();
        mock.fail_registers = true;
        let mut swd = ProbeRsSwd::new(&mut mock);
        assert_eq!(
            swd.read_inner(APnDP::DP, DPRegister::CTRLSTAT),
            Err(Error::AckProtocol)
        );
        assert!(matches!(
            swd.take_transport_error(),
            Some(ArmError::Probe(DebugProbeError::Usb(_)))
        ));
        assert!(swd.take_transport_error().is_none());
    }

    #[test]
    fn an_ack_failure_records_no_transport_error() {
        let mut mock = MockDap {
            ap_reads_before_fault: Some(0),
            ..MockDap::new()
        };
        let mut swd = ProbeRsSwd::new(&mut mock);
        assert_eq!(
            swd.read_inner(APnDP::AP, DPRegister::SELECT),
            Err(Error::AckFault)
        );
        assert!(swd.take_transport_error().is_none());
    }

    #[test]
    fn a_failed_reset_pin_keeps_its_probe_error() {
        let mut mock = MockDap::new();
        mock.fail_pins = true;
        let mut swd = ProbeRsSwd::new(&mut mock);
        assert!(swd.set_reset(true).is_err());
        assert!(matches!(
            swd.take_transport_error(),
            Some(ArmError::Probe(DebugProbeError::Usb(_)))
        ));
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
        // The (num_bits, data) pairs `init_debug` feeds `write_sequence`.
        let ones = [0xFFu8; 8];
        let chunks: Vec<_> = sequence_chunks(51, &ones).collect();
        assert_eq!(chunks, vec![(51, 0xFFFF_FFFF_FFFF_FFFFu64)]);

        let switch = 0xE79Eu16.to_le_bytes();
        let chunks: Vec<_> = sequence_chunks(16, &switch).collect();
        assert_eq!(chunks, vec![(16, 0xE79Eu64)]);

        let idle = [0x00u8; 2];
        let chunks: Vec<_> = sequence_chunks(8, &idle).collect();
        assert_eq!(chunks, vec![(8, 0u64)]);
    }

    #[test]
    fn over_64_bits_splits_into_two_chunks() {
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
        let data = [0xABu8, 0xCDu8];
        let chunks: Vec<_> = sequence_chunks(12, &data).collect();
        assert_eq!(chunks, vec![(12, 0xCDABu64)]);
    }
}
