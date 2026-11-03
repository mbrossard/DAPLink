//! Single-core terminal fault handlers with bounded retained recording.

/// Compose the panic and hard-fault handlers once, at the crate root; `with_location` keeps paths.
/// The invoking crate must depend on `cortex-m` and `cortex-m-rt`.
#[macro_export]
macro_rules! fault_reset_handlers {
    (with_location) => {
        $crate::__fault_reset_handlers!(|info| {
            match info.location() {
                Some(loc) => Some((loc.file(), loc.line())),
                None => None,
            }
        });
    };
    () => {
        $crate::__fault_reset_handlers!(|info| {
            let _ = info;
            None
        });
    };
}

/// Implementation detail of [`fault_reset_handlers!`]: the handlers over a chosen location form.
#[doc(hidden)]
#[macro_export]
macro_rules! __fault_reset_handlers {
    (|$info:ident| $location:block) => {
        /// Stop the part with interrupts masked, the state a debugger can still be attached to.
        fn __daplink_fault_halt() -> ! {
            ::cortex_m::interrupt::disable();
            loop {
                ::cortex_m::asm::wfi();
            }
        }

        #[panic_handler]
        fn __daplink_panic($info: &::core::panic::PanicInfo) -> ! {
            ::cortex_m::interrupt::disable();
            let location = $location;
            // SAFETY: the single-core handler never resumes interrupted code.
            if unsafe { $crate::stamp::terminal_panic(location) }.resets() {
                ::cortex_m::peripheral::SCB::sys_reset()
            }
            __daplink_fault_halt()
        }

        #[::cortex_m_rt::exception]
        unsafe fn HardFault(frame: &::cortex_m_rt::ExceptionFrame) -> ! {
            ::cortex_m::interrupt::disable();
            // SAFETY: the single-core handler never resumes interrupted code.
            if unsafe { $crate::stamp::terminal_hard_fault(frame.pc()) }.resets() {
                ::cortex_m::peripheral::SCB::sys_reset()
            }
            __daplink_fault_halt()
        }
    };
}
