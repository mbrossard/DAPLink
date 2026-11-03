/* The paced bodies run from RAM: an output section of their own after `.data`, riding its copy. */
SECTIONS {
    .ramcode : ALIGN(4) {
        /* Padding here moves run and load alike, so the bodies start 16-aligned as they were fitted. */
        . = ALIGN(16);
        *(.ramcode .ramcode.*);
        . = ALIGN(4);
    } > RAM AT>FLASH
} INSERT AFTER .data;

/* `Reset` copies `.data` through `__edata` as one run, so this section must sit at the same offset in both. */
ASSERT(ADDR(.ramcode) - ADDR(.data) == LOADADDR(.ramcode) - LOADADDR(.data), "ramcode: load and run offsets from .data differ, so the copy would shift the code");
ASSERT(__edata >= ADDR(.ramcode) + SIZEOF(.ramcode), "ramcode: the .data copy ends before the section does");
ASSERT(SIZEOF(.ramcode) <= 32K, "ramcode: over its 32 KiB budget");
