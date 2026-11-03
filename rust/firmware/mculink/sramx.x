/* SRAMX: the 32 KiB zero-wait code-bus SRAM the masked SWD and JTAG bodies execute from. */
MEMORY {
    SRAMX : ORIGIN = 0x04000000, LENGTH = 32K
}

/* cortex-m-rt copies only `.data`; the firmware copies this section itself before any transfer. */
SECTIONS {
    .sramx_code : ALIGN(4) {
        __ssramx_code = .;
        *(.sramx_code .sramx_code.*);
        . = ALIGN(4);
        __esramx_code = .;
    } > SRAMX AT>FLASH
    __sisramx_code = LOADADDR(.sramx_code);
} INSERT AFTER .text;

/* Tighter than the 32 KiB region so growth is deliberate; CASPER and the power API also claim SRAMX blocks (UM11126 Table 4). */
ASSERT(SIZEOF(.sramx_code) <= 28K, "sramx: .sramx_code over its 28 KiB budget");
ASSERT(SIZEOF(.sramx_code) > 0, "sramx: no body placed in .sramx_code");
