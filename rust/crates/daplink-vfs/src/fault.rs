//! Fault handlers that record a reason in the [`ResetStamp`](crate::stamp::ResetStamp) and reset
//! the probe, rather than stopping it off the bus.

/// Compose the panic and hard-fault handlers once, at a firmware's crate root, and only where it
/// links no other panic handler. `with_location` keeps every panic site's path string in the image.
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
            // Never the message: rendering it needs the core formatting machinery.
            let location = $location;
            if $crate::stamp::latch_panic(location).resets() {
                ::cortex_m::peripheral::SCB::sys_reset()
            }
            __daplink_fault_halt()
        }

        #[::cortex_m_rt::exception]
        unsafe fn HardFault(frame: &::cortex_m_rt::ExceptionFrame) -> ! {
            // The stacked program counter is the whole record, resolved against the image's ELF.
            if $crate::stamp::latch_hard_fault(frame.pc()).resets() {
                ::cortex_m::peripheral::SCB::sys_reset()
            }
            __daplink_fault_halt()
        }
    };
}
