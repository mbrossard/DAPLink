# Execution models

Portable crates are synchronous, `no_std`, and runtime-neutral. The CMSIS-DAP engine processes one
complete command at a time; firmware decides where that work runs and how USB traffic reaches it.

| Model | Scheduling and ownership |
|---|---|
| Embassy | Async tasks own USB, VCOM, mass storage, and other services; endpoint futures wait without polling |
| RTIC | Static resources are assigned to interrupt tasks; the USB interrupt polls classes and performs bounded queue work |

Embassy firmware runs a reader and executor future over separate bulk endpoint halves. RTIC
firmware pumps the same `DapQueue` from the USB interrupt. In both models, requests execute in FIFO
order and target-wire operations remain synchronous; pipelining overlaps host USB latency, not two
operations on the target adapter.

Platform crates adapt clocks, pins, UARTs, USB peripherals, and interrupt mechanics to the model.
Reusable protocol and service crates do not depend on Embassy or RTIC. A firmware chooses one
model and owns the static storage, task spawning, interrupt bindings, and service cadence it needs.

No state crosses a USB session implicitly. Session teardown clears queued packets and suspends the
debug engine; persistent state such as a reset stamp is placed explicitly in reset-surviving RAM.
