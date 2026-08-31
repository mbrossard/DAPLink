# USB design

The project has two USB adapters with the same logical device contract: `daplink-embassy-usb` for
async firmware and `daplink-usb-device` for polled firmware. Shared identity and WinUSB descriptor
data live in `daplink-core`; each adapter owns the endpoint and class mechanics of its USB stack.

Composite firmware allocates CMSIS-DAP v2 first on interface 0, CDC-ACM on interfaces 1 and 2,
and optional mass storage last on interface 3. DAP-only firmware omits the other classes without
changing the CMSIS-DAP interface.

CMSIS-DAP uses one bulk packet per command and one per response. A fixed-capacity `DapQueue` holds
strict-FIFO request and response rings; its depth is also the packet count advertised to the host.
A full request ring applies endpoint backpressure rather than dropping a command, and
`DAP_TransferAbort` is intercepted without entering the FIFO.

Embassy firmware splits the bulk endpoints between reader and executor futures. Polled firmware
pumps the same queue from its USB interrupt. Both preserve command order and clear queued state
when a USB session ends; the engine is suspended so protocol and target-pin state cannot leak into
the next host session.

CDC data is bridged to a platform UART. Mass-storage builds append a separate BOT transport over
the virtual volume; neither service participates in CMSIS-DAP packet ordering.
