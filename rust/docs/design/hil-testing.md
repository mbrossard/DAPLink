# Hardware-in-the-loop testing

`cargo xtask hil` resolves a probe firmware row and the wired target's DUT image from the build
registry, builds both, installs the probe firmware -- through a transport probe, or onto the
probe's own BOOTSEL drive -- and runs `daplink-test` against the wired DUT through the probe under
test. The [harness reference](../../tests/daplink-test/README.md) owns the case list, gating
rules, and effects on the target.

The build registry names firmware rows, targets and DUT images; the harness owns the rig facts --
probe-rs chip name, the memory a case may pattern, and each target's abilities.

Cases are ordered by dependency: USB identity, target connection, wire transfers, programming,
RTT, reset, VCOM, SWO, JTAG, target-flash front ends, then speed sweeps. Insert a new case at its
layer rather than appending it, so the first failure names the lowest broken layer.

A failure records and the run continues. A fatal verdict, which any case returns when it loses
state later cases depend on -- the probe connection, the target session, the DUT's RTT channel --
stops the run. Missing hardware is a skip.

Eligibility is the intersection of the rig profile, the target's capabilities, and operator
overrides. Firmware-defined services detect themselves instead: vendor flashing from the answer to
its first command, mass storage from the USB descriptors.

The harness uses the interface closest to each contract: raw CMSIS-DAP bulk for protocol behavior,
probe-rs sessions for target access, a serial port for VCOM, and raw BOT for mass storage. macOS
claims the mass-storage interface exclusively, so raw BOT cases do not exist in a macOS build.

The DUT images are small independent firmwares providing known flash and RAM state, an RTT command
channel, reset evidence, and UART echo; the one whose board routes a trace pin also emits a
sequence-tagged trace stream. Their newline-framed protocol is revisioned in
`daplink-test-protocol`, and an unsupported command fails explicitly.
