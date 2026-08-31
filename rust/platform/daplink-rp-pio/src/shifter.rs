//! The [`PioShifter`] seam: the primitive state-machine operations the protocol engine drives.

use daplink_rs::dap_rs::swj::Pins;

/// The PIO state-machine primitives, so the protocol engine is host-testable against a mock. The
/// SM always makes progress, so only a host-side pairing bug can wedge the blocking `put`/`get`.
pub trait PioShifter {
    /// Push one word into the TX FIFO, blocking until there is space.
    fn put(&mut self, word: u32);

    /// Pop one word from the RX FIFO, blocking until one arrives.
    fn get(&mut self) -> u32;

    /// Set the encoded integer divider; zero means 65,536 and the fractional part stays zero.
    fn set_clock_divider(&mut self, divider: u16);

    /// Enable or disable the state machine.
    fn set_enabled(&mut self, enabled: bool);

    /// Flush both FIFOs.
    fn clear_fifos(&mut self);

    /// Restart shift counters, ISR, delay/IRQ-wait state and any latched stalled instruction.
    fn restart(&mut self);

    /// Force an unconditional `jmp` to `addr`, re-seating the PC at the dispatcher.
    fn exec_jmp(&mut self, addr: u8);

    /// Force the active-low OEN output latch high before its direction can expose it.
    fn force_oen_released(&mut self);

    /// Force the SWCLK output latch low.
    fn force_swclk_low(&mut self);

    /// Force SWDIO to input and SWCLK/OEN to output after their idle levels are staged.
    fn force_pindirs_idle(&mut self);

    /// Point the shared SWCLK/SWDIO pads' function select at SIO; function select only.
    fn pads_to_sio(&mut self);

    /// Point the same pads back at PIO; function select only.
    fn pads_to_pio(&mut self);
}

/// Stage safe output levels, restore idle directions, park the dispatcher, and start the SM.
#[cfg(any(feature = "rp2040", feature = "rp235x", test))]
pub(crate) fn start_idle<S: PioShifter>(shifter: &mut S, dispatcher: u8) {
    shifter.force_oen_released();
    shifter.force_swclk_low();
    shifter.force_pindirs_idle();
    shifter.exec_jmp(dispatcher);
    shifter.set_enabled(true);
}

/// The `DAP_SWJ_Pins` half that runs over software GPIO while the shared pads are on loan.
pub trait SwjPinsSio {
    /// Apply the masked levels, dwell, and sample every pin; the caller brackets the pad loan.
    fn swj_pins(&mut self, output: Pins, mask: Pins, wait_us: u32) -> Pins;
}
