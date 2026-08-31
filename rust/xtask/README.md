# xtask

Build orchestration for the firmware fleet, run as `cargo xtask <verb>` from `rust/`:

- `list` -- print the firmware registry (path, target, chip);
- `build [<path>]` -- release-build one row, or with no argument the default compositions
  (rows named by their crate path; variants carry a suffix); `build-all` -- every row, the
  milestone gate;
- `flash <path> --probe <sel> [--put <sel>]` -- build, then flash through the `--probe`
  transport probe and reset, `--probe` meaning the transport exactly as it does for probe-rs,
  which this wraps. `--put` names the flashed board's own USB identity and is
  required for every row that can be a probe under test: it is what makes the re-attach and
  the settle wait possible. A DUT image, which has no USB identity, takes `--probe` alone, and a
  row flashed through its own BOOTSEL drive takes `--put` alone;
- `hil <path> --via <sel> --probe <sel> --target <name>` -- **note the different vocabulary**:
  `hil` has always named the transport `--via` and the board `--probe`. Build the firmware and the
  target's DUT image, flash the firmware through the `--via` transport probe, then run the
  daplink-test cases through `--probe` (the probe under test) against the target; the target
  registry maps the short name to its probe-rs chip and DUT image.

Flashing pre-verifies, so a byte-identical resident image is not rewritten. Selectors are
always full `vid:pid:serial`; many probes share a VID:PID.
