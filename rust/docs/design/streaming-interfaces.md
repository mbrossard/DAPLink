# Streaming interfaces

VCOM and SWO both cross USB and a hardware serial peripheral, but their contracts and ownership
are separate.

| Interface | Direction and transport | Buffering |
|---|---|---|
| VCOM | Bidirectional CDC-ACM byte stream bridged to a probe UART | Platform UART buffering or DMA, serviced independently in each direction |
| SWO | Target-to-host trace captured by a probe UART and read through CMSIS-DAP commands | Producer/consumer ring fed by an interrupt or DMA |

## VCOM

The USB adapter owns the CDC endpoints and line coding; the platform seam owns UART setup,
buffering and error attribution. The Embassy baud setter returns `BaudApply`, and only `Applied`
updates the bridge's cached baud; a rejected request stays retryable. An applied baud may be
clamped by hardware.

The [Embassy bridge](../../platform/daplink-embassy-usb/src/lib.rs) joins a UART/OUT future and an
IN future over a return pipe. `VcomUart::read` must be cancellation-safe: the bridge drops it
whenever another arm wins, so only a ready result may consume bytes. `write_all` hands the slice
to the TX path; `drain_tx` is the wire-idle boundary, including the shifter.

Changing baud must stop the DMA producers before rebasing cursors, clearing buffers or rearming.
If that abort fails, RP Embassy probes enter an interrupt-disabled fail-stop that only a full
device reset clears; RP DUT images instead keep their buffers and reject stream changes until a
retry succeeds. See [RP RX](../../platform/daplink-rp-embassy/src/dma_rx.rs) and
[RP TX](../../platform/daplink-rp-embassy/src/dma_tx.rs).

MCU-Link and RP2040 RTIC service polled CDC/UART staging from their own pass or interrupt; see
[USB sessions](execution-models.md#queue-and-session-ownership).

## SWO

SWO is a CMSIS-DAP engine role, not a USB class. `RingSwo` owns capture state and command
semantics, a platform `SwoBackend` configures and starts the UART, and a `SwoSource` supplies bytes
from an interrupt-fed ring or a DMA ring. Producers report overrun through their source; consumers
drain only what the host requests. Starting or stopping capture also changes the UART and, where
shared, the TDO pad owner. A DMA-backed source must keep its storage until the producer is
quiescent.
