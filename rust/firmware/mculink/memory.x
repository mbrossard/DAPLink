/* LPC55S69JBD64: on-chip flash at 0x0, executed in place. On the non-secure path the boot
   ROM takes a type-0 image's SP and PC straight from the vector table, so no image-header
   words need filling. One contiguous SRAM window spans SRAM0..SRAM3; the USB1 controller's
   dedicated SRAM is managed by the USBHS driver and is not part of this map. */
MEMORY {
    FLASH : ORIGIN = 0x00000000, LENGTH = 512K
    RAM   : ORIGIN = 0x20000000, LENGTH = 256K
}
