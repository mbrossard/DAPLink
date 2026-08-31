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

/* Catches --wrap=__pre_init silently not taking (0x20000000 is SRAM base; memory.x isn't parsed yet here). */
ASSERT(__wrap___pre_init < 0x20000000, "sram-exec: --wrap=__pre_init did not take");
