/* SRAMX: the 32 KiB zero-wait code-bus SRAM the masked SWD word path executes from. */
MEMORY {
    SRAMX : ORIGIN = 0x04000000, LENGTH = 32K
}

/* The masked SWD word path executes from SRAMX. cortex-m-rt's startup copies only `.data`,
   so this section carries its own load-address symbols and the firmware's `init` copies it
   before any transfer runs. */
SECTIONS {
    .sramx_code : ALIGN(4) {
        __ssramx_code = .;
        *(.sramx_code .sramx_code.*);
        . = ALIGN(4);
        __esramx_code = .;
    } > SRAMX AT>FLASH
    __sisramx_code = LOADADDR(.sramx_code);
} INSERT AFTER .text;
