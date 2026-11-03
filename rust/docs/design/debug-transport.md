# Debug transport

Each CMSIS-DAP command reaches `DapProbe` whole, framed by the [USB adapter](usb.md).
Vendor-reserved commands go to the configured vendor handler first; standard commands go to
`dap-rs`, which owns CMSIS-DAP decoding and SWD/JTAG sequencing.

`DapProbe` answers the `DAP_Info` identity strings -- vendor, product, serial, board vendor, board
name, firmware version, and on `0x09` the firmware row stamped at build time -- NUL-terminated, with
a length that counts the terminator; every other id is answered by `dap-rs`.

`DAP_TransferAbort` is consumed without a reply and has no other effect: a command runs to
completion before the next packet is examined, which CMSIS-DAP permits for a strictly serial probe.

`DAP_JTAG_IDCODE` addresses ARM TAPs only: it shifts the fixed opcode `0x0E`, which a non-ARM TAP
decodes as BYPASS and so reads zero. Read a non-ARM TAP's IDCODE with a raw `DAP_JTAG_Sequence`
scan. Chain index 0 is the TAP nearest TDO.

JTAG transfers speak both JTAG DP protocols. A scan leaves Shift-DR early only on WAIT; a TAP that
answers OK as `0b100` is an ADIv6 protocol-1 DP, whose `0b010` is FAULT, until the next
`DAP_Connect`, `DAP_Disconnect` or `DAP_JTAG_Configure`. FAULT reports `0x04` and no ACK `0x07`;
neither is retried.

`dap-rs` separates target access into dependency, SWD, and JTAG roles; the dependency role also
provides SWJ pin control and reset. A backend moves between roles when the host selects a
protocol, and the two protocols never drive shared pads concurrently.

| Backend | Division of work |
|---|---|
| Bit-bang | The CPU drives complete SWD and JTAG bit cells through GPIO |
| RP PIO | Generic PIO0 sequences plus complete-transfer SWD/JTAG on PIO1 |
| SPI-assisted | SPI shifts supported SWD phases while GPIO handles the remainder and JTAG |

Hardware backends map clock requests to platform timing or peripheral divisors; a request is a
ceiling, not an exact rate. See [RP PIO](rp-pio-engines.md),
[LPC55 masked GPIO](../internals/lpc55/masked-swd.md), and
[STM32 SPI sessions](../internals/stm32/spi-assisted-swd.md).
