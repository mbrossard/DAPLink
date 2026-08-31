//! Boot-time NVIC hygiene: disable every enabled line still pointing at `DefaultHandler`.

use cortex_m::peripheral::NVIC;

extern "C" {
    /// First device-interrupt vector, emitted by `cortex-m-rt`.
    static __INTERRUPTS: u8;

    /// The weak fallback every unoverridden vector points at.
    fn DefaultHandler();
}

/// Disable every enabled NVIC line whose vector is still `DefaultHandler`, returning the count.
///
/// Call once, immediately after `embassy_stm32::init` and before enabling interrupts of your own.
///
/// # Safety
///
/// No interrupt this disables may be serviced other than through its vector-table entry.
pub unsafe fn mask_unhandled_interrupts() -> u32 {
    // Both sides carry the Thumb bit; compare the instruction addresses.
    let unhandled = (DefaultHandler as *const () as usize) & !1;
    let vectors = (&raw const __INTERRUPTS) as *const usize;

    // SAFETY: the architecturally fixed NVIC block address, valid for the life of the program.
    let nvic = unsafe { &*NVIC::PTR };

    let mut masked = 0u32;
    for (word, iser) in nvic.iser.iter().enumerate() {
        let mut enabled = iser.read();
        while enabled != 0 {
            let bit = enabled.trailing_zeros();
            enabled &= !(1u32 << bit);
            let irq = word * 32 + bit as usize;

            // SAFETY: `cortex-m-rt` sizes the table to cover every implemented line.
            let handler = unsafe { vectors.add(irq).read_volatile() } & !1;
            if handler != unhandled {
                continue;
            }

            // SAFETY: ICER is write-1-to-clear, so this disables exactly this line.
            unsafe { nvic.icer[word].write(1u32 << bit) };
            masked += 1;
        }
    }

    // The disables must be in effect before the caller arms anything.
    cortex_m::asm::dsb();
    cortex_m::asm::isb();
    masked
}
