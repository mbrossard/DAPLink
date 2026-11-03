/* .text runs from SRAM; .rodata stays in flash under XIP. */
SECTIONS {
    .ramtext : ALIGN(4)
    {
        __sramtext = .;
        *(.text .text.*);
        . = ALIGN(4);
        __eramtext = .;
    } > RAM AT > FLASH
    __siramtext = LOADADDR(.ramtext);
} INSERT AFTER .rodata;

/* Catches __pre_init falling back to cortex-m-rt's default, which .ramtext would claim (0x20000000 is SRAM base). */
ASSERT(__pre_init < 0x20000000, "sram-exec: __pre_init is not flash-resident");
