# USB design

The project has two USB adapters with the same logical device contract: `daplink-embassy-usb` for
async firmware and `daplink-usb-device` for polled firmware. Shared identity and WinUSB descriptor
data live in `daplink-core`; each adapter owns the endpoint and class mechanics of its USB stack.

Composite firmware allocates CMSIS-DAP v2 first on interface 0, CDC-ACM on interfaces 1 and 2,
and optional mass storage last on interface 3. DAP-only firmware omits the other classes without
changing the CMSIS-DAP interface.

A CMSIS-DAP command and its response are each one bulk packet, except in `dap-packet-512` builds,
where a DAP packet larger than the endpoint spans several bulk packets each way. A fixed-capacity
`DapQueue` holds strict-FIFO request and response rings; its depth is also the packet count
advertised to the host.
A full request ring applies endpoint backpressure rather than dropping a command, and
`DAP_TransferAbort` is consumed without entering the FIFO (see
[debug transport](debug-transport.md)).

Embassy and polled firmware share FIFO ordering but not teardown; see
[queue/session ownership](execution-models.md#queue-and-session-ownership).

The device advertises one BOS platform capability, MS OS 2.0, which binds interface 0 to WinUSB
on Windows. There is no WebUSB capability; browsers reach CMSIS-DAP through the same bulk
interface.

CDC data is bridged to a platform UART. Mass-storage builds append a separate BOT transport over
the virtual volume; neither service participates in CMSIS-DAP packet ordering.
