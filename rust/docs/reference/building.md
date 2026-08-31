# Building and HIL

Install Rust through `rustup`. From `rust/`, the first Cargo invocation installs the channel,
components, and embedded targets selected by `rust-toolchain.toml`; `rustup show` can provision
them before a build. No separate `rustup target add` is needed.

Install the probe-rs command-line tools when flashing or running HIL:

```text
rustup show
cargo install probe-rs-tools --locked
```

The `probe-rs` executable is used by `cargo xtask flash` and `cargo xtask hil`, but is not needed
for compilation. Linux hosts also need the probe-rs udev rules for unprivileged USB access.

Run registered builds from `rust/`:

```text
cargo xtask list
cargo xtask build <firmware-name>
```

For the Raspberry Pi boards, `cargo xtask build` also writes a drag-and-drop `.uf2` beside the
ELF; each board page gives its BOOTSEL install steps.

For a direct build in an individual workspace, keep its checked-in lockfile authoritative:

```text
cargo build --release --locked
```

To build, install, and exercise a probe on a wired HIL rig:

```text
cargo xtask hil <firmware-name> --via <transport-probe> \
    --probe <probe-under-test> --target <target-name>
```

Selectors are full `vid:pid:serial` values. `--via` identifies the transport probe used to flash
the probe under test; it is omitted for firmware updated through its own BOOTSEL drive. The command
builds the probe and DUT images, installs the probe firmware, and runs the default test cases.

See the [xtask reference](../../xtask/README.md) for command behavior and the
[HIL harness reference](../../tests/daplink-test/README.md) for cases, capability gating, direct
harness invocation, and its effects on the target.
