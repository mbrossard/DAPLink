/* The DMA controller's one allocation, pinned at the start of RAM. `SRAMBASE` needs 512-byte
   alignment, which the type already carries, but the address it lands on must also not drift:
   this part's USB attach is sensitive to the RAM map, and leaving the block in `.bss` lets any
   unrelated static move it. Placing it before `.data` fixes it at the RAM origin for every
   feature combination. cortex-m-rt zeroes only `.bss`, so `dma::init` clears this itself. */
SECTIONS {
    .dma_mem (NOLOAD) : ALIGN(512) {
        __sdma_mem = .;
        *(.dma_mem);
        . = ALIGN(4);
        __edma_mem = .;
    } > RAM
} INSERT BEFORE .data;
