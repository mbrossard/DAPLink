# DAPLink features not carried forward

Features of the C firmware this project deliberately does not replicate. Each entry names the
reason; absence elsewhere in these docs is not a decision, this list is.

- **Drag-and-drop flashing of raw `.bin` images.** A raw binary carries no load address, so the
  base must be inferred, and hosts reorder and interleave block writes in ways the format cannot
  detect. UF2 is the supported drag-and-drop format. Unrecognized files are ignored, the same way
  OS metadata files are -- there is no per-format rejection.
- **Action files** (`ERASE.ACT`, `REFRESH.ACT`, `ASSERT.ACT`, `AUTO_RST.CFG`, and the rest of
  the 8.3 control surface). Equivalent operations are exposed as CMSIS-DAP vendor commands where
  they are needed at all.
- **Persistent configuration.** With the action files gone, nothing remains that needs to
  survive a power cycle; per-session state is set by the host each session.
- **CMSIS-DAP v1 (HID).** Only the v2 bulk interface is provided; probe-rs, pyOCD, OpenOCD and
  DAPjs all drive v2.
- **A resident bootloader.** Each probe updates through its chip's own ROM bootloader, which the
  `0x9F` vendor command enters where the ROM offers a USB update path; boards without one are
  programmed over SWD.
- **Serial line settings beyond the baud rate.** The VCOM bridge applies the baud rate and runs
  8N1; other data bits, parity and stop bits are not applied, DTR and RTS drive no pins, and no
  break is sent.
- **Atomic commands and SWD turnaround configuration.** `DAP_QueueCommands` and
  `DAP_ExecuteCommands` are not implemented (the advertised packet count serves the purpose), and
  `DAP_SWD_Configure` accepts only the default turnaround and data phase.
