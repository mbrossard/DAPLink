# RP DUT abort failures

`ERR dma_abort_timeout` rejects an RP DUT command when DMA ownership is unresolved.
It does not acknowledge completion or imply that all staged bytes reached the UART.

Failed `V1 START` or `V1 RESET` retains the previous run, transmit slots and stream
counters, and pauses data service. Retry either command to attempt TX abort again;
only a successful retry replaces or resets that state. `V1 STOP` and `STATS RESET`
also reject while service is paused. Identity, `STATS` and `V1 REPORT` remain readable.

A failed RX abort rejects `BAUD` without rearming or resetting its ring; divisors
may already have changed. Retry a valid `BAUD` to release and rearm RX before
retrying run commands. The `STATS stuck` field counts RX and TX abort timeouts,
saturating at `u32::MAX` since boot; command success and stats reset do not clear it.

These errors apply to the RP DUT images, not the probe's DMA fail-stop behavior.
