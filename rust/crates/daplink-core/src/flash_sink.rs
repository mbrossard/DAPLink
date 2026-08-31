//! Flash-sink port: the seam between image transfer and target flashing.

/// Which phase of a flash operation an error occurred in.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum FlashPhase {
    /// No target attached or selected.
    NoTarget,
    /// Target init / connect.
    Init,
    /// The silicon on the wire did not match the selected target's identity fingerprint.
    Identity,
    /// The dropped image's UF2 family id does not match the selected target's.
    Family,
    /// A raw-binary transfer started on sector content alone did not re-validate as a vector table.
    Content,
    /// A decoded fragment lies outside the selected target's programmable flash.
    Range,
    /// Sector erase.
    Erase,
    /// A page write that reached the target and failed.
    Program,
    /// Post-program verify.
    Verify,
    /// Target teardown.
    Uninit,
}

/// One flash window paired with one RAM window of the same device, each half-open `[start, end)`.
/// The pairing is the point: a stack pointer and a reset vector from two parts match neither.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct TargetWindows {
    /// A programmable flash window of the device.
    pub flash: (u32, u32),
    /// A RAM window of the same device.
    pub ram: (u32, u32),
}

/// Sink that programs decoded image bytes into target flash: `begin` once, then `program` per
/// in-range fragment, then `finish(success)` once.
pub trait FlashBackend {
    /// Begin a flashing session (init/erase as needed).
    fn begin(&mut self) -> Result<(), FlashPhase>;

    /// Begin a session with the dropped image's declared UF2 family id as a selection hint.
    fn begin_with_hint(&mut self, family: Option<u32>) -> Result<(), FlashPhase> {
        let _ = family;
        self.begin()
    }

    /// Program `data` at absolute `addr` (called only for in-range fragments).
    fn program(&mut self, addr: u32, data: &[u8]) -> Result<(), FlashPhase>;

    /// Finish the session; `success = false` aborts.
    fn finish(&mut self, success: bool) -> Result<(), FlashPhase>;

    /// Finish, additionally signaling whether a *successful* flash resets the target to run.
    /// Required rather than defaulted, so a backend cannot drop the policy and still compile clean.
    fn finish_with_reset(&mut self, success: bool, reset: bool) -> Result<(), FlashPhase>;

    /// Select per-sector erase (`true`) or whole-chip erase (`false`) for sessions begun after
    /// this call; mirrors the C firmware's `flash_manager_set_page_erase`.
    fn set_sector_erase(&mut self, sector: bool) -> Result<(), FlashPhase> {
        // No capability: report the request as an erase-phase failure, never a silent success.
        let _ = sector;
        Err(FlashPhase::Erase)
    }

    /// Mass-erase every programmable region, outside any image transfer.
    fn erase_all(&mut self) -> Result<(), FlashPhase> {
        // No capability: report the request as an erase-phase failure, never a silent success.
        Err(FlashPhase::Erase)
    }

    /// Programmable `(start, end)` ranges used to range-check fragments.
    fn flash_ranges(&self) -> &[(u32, u32)];

    /// The selected target's RAM `(start, end)` ranges, which an image's stack pointer is judged
    /// against. Empty means "this backend cannot say", not "RAM is unconstrained".
    fn ram_ranges(&self) -> &[(u32, u32)] {
        &[]
    }

    /// Every [`TargetWindows`] pairing this backend may resolve, known before a session begins.
    fn candidate_windows(&self) -> &[TargetWindows] {
        &[]
    }

    /// The UF2 family id the selected target expects, or `None` if no cross-check applies.
    fn expected_uf2_family(&self) -> Option<u32> {
        None
    }

    /// The name of the target the last/current session resolved.
    fn selected_target_name(&self) -> Option<&str> {
        None
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::{FlashBackend, FlashPhase};
    use std::format;
    use std::string::String;
    use std::vec::Vec;

    /// Recording mock backend: appends a description of each call it receives.
    struct MockBackend {
        ranges: [(u32, u32); 1],
        calls: Vec<String>,
        fail_program: bool,
    }

    impl MockBackend {
        fn new(fail_program: bool) -> Self {
            MockBackend {
                ranges: [(0x0800_0000, 0x0801_0000)],
                calls: Vec::new(),
                fail_program,
            }
        }
    }

    impl FlashBackend for MockBackend {
        fn begin(&mut self) -> Result<(), FlashPhase> {
            self.calls.push(String::from("begin"));
            Ok(())
        }

        fn program(&mut self, addr: u32, data: &[u8]) -> Result<(), FlashPhase> {
            self.calls
                .push(format!("program:{:#x}:{}", addr, data.len()));
            if self.fail_program {
                Err(FlashPhase::Program)
            } else {
                Ok(())
            }
        }

        fn finish(&mut self, success: bool) -> Result<(), FlashPhase> {
            self.calls.push(format!("finish:{success}"));
            Ok(())
        }

        fn finish_with_reset(&mut self, success: bool, reset: bool) -> Result<(), FlashPhase> {
            self.calls.push(format!("finish:{success}:reset={reset}"));
            Ok(())
        }

        fn flash_ranges(&self) -> &[(u32, u32)] {
            &self.ranges
        }
    }

    #[test]
    fn flash_backend_call_order() {
        let mut backend = MockBackend::new(false);

        assert_eq!(backend.flash_ranges(), &[(0x0800_0000, 0x0801_0000)]);

        backend.begin().unwrap();
        backend.program(0x0800_0000, &[0xAA; 4]).unwrap();
        backend.program(0x0800_0004, &[0xBB; 8]).unwrap();
        backend.finish(true).unwrap();

        assert_eq!(
            backend.calls,
            [
                String::from("begin"),
                String::from("program:0x8000000:4"),
                String::from("program:0x8000004:8"),
                String::from("finish:true"),
            ]
        );
    }

    #[test]
    fn flash_phase_error_propagates() {
        let mut backend = MockBackend::new(true);

        backend.begin().unwrap();
        let err = backend.program(0x0800_0000, &[0x00; 4]).unwrap_err();

        assert_eq!(err, FlashPhase::Program);
    }

    /// A backend with no mass-erase capability surfaces the request as an erase-phase failure.
    #[test]
    fn erase_all_default_reports_erase_phase() {
        let mut backend = MockBackend::new(false);
        assert_eq!(backend.erase_all(), Err(FlashPhase::Erase));
    }

    /// `finish_with_reset` has no default body, so every backend sees the reset policy it is handed.
    #[test]
    fn finish_with_reset_carries_the_policy_to_the_backend() {
        let mut backend = MockBackend::new(false);
        backend.begin().unwrap();
        backend.finish_with_reset(true, true).unwrap();
        assert_eq!(
            backend.calls,
            [
                String::from("begin"),
                String::from("finish:true:reset=true")
            ]
        );
    }

    /// A backend with no erase-mode control refuses the request rather than silently ignoring it.
    #[test]
    fn set_sector_erase_default_reports_erase_phase() {
        let mut backend = MockBackend::new(false);
        assert_eq!(backend.set_sector_erase(true), Err(FlashPhase::Erase));
        assert_eq!(backend.set_sector_erase(false), Err(FlashPhase::Erase));
    }

    /// Both window defaults are empty, and a caller sweeping an empty set accepts nothing.
    #[test]
    fn window_defaults_are_empty_not_permissive() {
        let backend = MockBackend::new(false);
        assert!(backend.ram_ranges().is_empty());
        assert!(backend.candidate_windows().is_empty());
    }

    #[test]
    fn flash_finish_false_abort() {
        let mut backend = MockBackend::new(false);

        backend.begin().unwrap();
        let result = backend.finish(false);

        assert!(result.is_ok());
        assert_eq!(
            backend.calls,
            [String::from("begin"), String::from("finish:false")]
        );
    }
}
