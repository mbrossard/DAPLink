/* DMA block pinned at the RAM origin, where USB attach needs it; cortex-m-rt does not zero it. */
SECTIONS {
    .dma_mem (NOLOAD) : ALIGN(512) {
        __sdma_mem = .;
        *(.dma_mem);
        . = ALIGN(4);
        __edma_mem = .;
    } > RAM
} INSERT BEFORE .data;
