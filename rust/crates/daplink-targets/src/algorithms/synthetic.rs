//! Synthetic test-fixture algorithm: a fabricated blob and RAM window; never flash with it.

use crate::{FlashAlgorithm, SyscallParams};

/// Synthetic (fabricated) algorithm code blob. Not real flash-controller code.
static SYNTHETIC_BLOB: [u8; 32] = [
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
    0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f,
];

/// Synthetic flash algorithm: entry offsets into [`SYNTHETIC_BLOB`], addresses inside a fake RAM
/// window at `0x2000_0000..0x2000_2000`.
pub static SYNTHETIC_ALGO: FlashAlgorithm<'static> = FlashAlgorithm {
    init: 0x00,
    uninit: 0x04,
    erase_chip: 0x08,
    erase_sector: 0x0c,
    program_page: 0x10,
    // Deliberately `None`: it is what exercises the read-back verify fallback, and a test pins it.
    verify: None,
    syscall: SyscallParams {
        breakpoint: 0x2000_0001,
        static_base: 0x2000_1000,
        stack_pointer: 0x2000_1800,
    },
    program_buffer: 0x2000_0400,
    program_buffer_size: 0x400,
    algo_start: 0x2000_0000,
    algo_blob: &SYNTHETIC_BLOB,
    verify_returns_address: false,
    single_init_type: false,
    skip_chip_erase: false,
};
