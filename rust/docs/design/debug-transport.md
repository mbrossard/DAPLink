# Debug transport

Each USB bulk OUT packet reaches `DapProbe` as one CMSIS-DAP command. Vendor-reserved commands are
offered to the configured vendor handler first; standard commands are dispatched to `dap-rs`,
which owns CMSIS-DAP decoding and SWD/JTAG protocol sequencing.

`dap-rs` separates target access into dependency, SWD, and JTAG roles. A backend moves between
those roles when the host selects a protocol, while the firmware retains one owner for the
physical adapter. The dependency role also provides SWJ pin control and reset-line access.

The project supplies three wire implementation shapes:

| Backend | Division of work |
|---|---|
| Bit-bang | The CPU drives complete SWD and JTAG bit cells through GPIO |
| RP PIO | PIO shifts SWD spans while the CPU handles protocol decisions; JTAG remains bit-bang |
| SPI-assisted | SPI shifts supported SWD phases while GPIO handles the remainder and JTAG |

An accelerated SWD backend may own pads that JTAG also needs. `SwjPadLoan` makes that handoff
explicit: selecting JTAG returns the shared pads to GPIO, and selecting SWD restores the hardware
backend. The two protocols never drive the pads concurrently.

Clock requests are upper bounds. Each backend maps a request to a realizable, safe clock using
platform-supplied timing or peripheral divisors; board-independent protocol code does not choose
MCU timing constants.
