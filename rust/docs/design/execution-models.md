# Execution models

Portable crates are synchronous, `no_std`, and runtime-neutral. The CMSIS-DAP engine processes one
complete command at a time; firmware decides where that work runs.

| Model | Scheduling and ownership |
|---|---|
| Embassy | Async tasks own USB, VCOM, mass storage, and other services |
| RTIC | Static resources assigned to interrupt tasks; the USB interrupt polls classes and does bounded queue work |

Embassy's pipeline has reader, executor and writer roles over a `DapQueue`; polled firmware pumps
the queue from a USB interrupt or cooperative pass. Requests execute in FIFO order; pipelining
overlaps host USB latency, not target-wire operations.

## Queue and session ownership

The Embassy writer copies a response into writer-local scratch before awaiting the IN endpoint,
so the endpoint future never borrows the response ring.

The split Embassy composition tracks an Initial/Running/Stopping/Parked state. Beginning a session
completes any previous teardown, resets the queue, then releases the endpoint tasks; teardown
waits for each task's parking acknowledgement before the storage is reused. The cooperative
composition instead joins the three futures within one session borrow. `dap_sessions` owns this
sequence; see [daplink-embassy-usb](../../platform/daplink-embassy-usb/src/lib.rs).

Never reset queue storage while an endpoint future still owns a slice. The low-level split
executor's reset is `unsafe` for that reason; custom owners must uphold its contract.

The polled [dap_pass](../../platform/daplink-usb-device/src/dap_pass.rs) clears the queue only
when `polled` is true and the session-ended latch is set. The USB stack can report false on a bus
reset, so session cleanup is not guaranteed before software-pended work runs.
