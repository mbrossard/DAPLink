//! `__pre_init` that copies `.ramtext` from its flash load image into SRAM before `main`.

/// Defines `__pre_init`; use when nothing else claims the slot.
/// `wrapping` interposes via `--wrap=__pre_init` and tail-calls the real one.
#[macro_export]
macro_rules! ram_text_pre_init {
    () => {
        $crate::__ram_text_copy_asm!("__pre_init", "pop {{pc}}");
    };
    (wrapping) => {
        $crate::__ram_text_copy_asm!("__wrap___pre_init", "bl __real___pre_init", "pop {{pc}}");
    };
}

/// Word-copies `[__sramtext, __eramtext)` from `__siramtext`, emitted into flash-resident `.Reset`.
#[doc(hidden)]
#[macro_export]
macro_rules! __ram_text_copy_asm {
    ($name:expr, $($tail:expr),+) => {
        ::core::arch::global_asm!(
            ".section .Reset, \"ax\"",
            concat!(".global ", $name),
            concat!(".type ", $name, ",%function"),
            ".thumb_func",
            concat!($name, ":"),
            "push {{lr}}",
            "ldr r0, =__sramtext",
            "ldr r1, =__eramtext",
            "ldr r2, =__siramtext",
            "0:",
            "cmp r1, r0",
            "beq 1f",
            "ldm r2!, {{r3}}",
            "stm r0!, {{r3}}",
            "b 0b",
            "1:",
            // Flush the store and discard any prefetch from before the copy landed.
            "dsb",
            "isb",
            $($tail),+,
            ".ltorg",
        );
    };
}
