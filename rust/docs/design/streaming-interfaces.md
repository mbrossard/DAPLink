# Streaming interfaces

VCOM and SWO both cross USB and a hardware serial peripheral, but their contracts and ownership
are deliberately separate.

| Interface | Direction and transport | Buffering |
|---|---|---|
| VCOM | Bidirectional CDC-ACM byte stream bridged to a probe UART | Platform UART buffering or DMA, serviced independently in each direction |
| SWO | Target-to-host trace captured by a probe UART and read through CMSIS-DAP commands | Producer/consumer ring fed by an interrupt or DMA |

The USB adapter owns CDC endpoint behavior and exposes line-coding changes. A platform VCOM seam
owns the UART and applies the requested baud; the bridge transfers only bytes and absorbs a UART
error at that boundary. Async and polled firmware service the bridge differently without changing
its USB-visible contract.

SWO is a CMSIS-DAP engine role, not a separate USB class. `RingSwo` owns capture state and command
semantics while a platform `SwoBackend` configures and starts the UART. A `SwoSource` supplies
captured bytes from either an interrupt-fed software ring or a DMA ring.

Buffering isolates short scheduling delays but does not make loss impossible. Producers report
overrun through their source contract; consumers drain only the response capacity requested by the
host. Firmware owns ring storage and chooses a size appropriate to its capture mechanism.
