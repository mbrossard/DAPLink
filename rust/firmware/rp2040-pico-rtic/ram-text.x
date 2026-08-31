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
