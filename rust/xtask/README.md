# xtask

Build orchestration for the firmware fleet, run as `cargo xtask <verb>` from `rust/`; `--help`
lists each verb's options:

- `list` -- print the firmware registry (path, target, chip);
- `build [<path>]` -- release-build one row, or with no argument the default compositions
  (rows named by their crate path; variants carry a suffix); `build-all` -- every row, then the
  feature shapes;
- `matrix [--link]` -- compile-check the feature shapes alone: every row of a `defmt`-capable
  crate crossed with `defmt`, plus the combinations in `xtask/src/matrix.rs`. `--link` builds
  instead, catching section overflows and duplicate symbols. Both this and `build-all` then check
  that each `INCOMPATIBLE` combination still fails with its `compile_error!` guard;
- `pins` -- print each dependency that several workspaces pin separately, and fail when their
  revisions differ;
- `test [--build]` -- the pin check, then `fmt --check`, `clippy -D warnings` and `test` in every
  workspace row, and with `--build` a full `build-all`;
- `flash <path> --via <sel> [--put <sel>]` -- build, then flash through the `--via`
  transport probe and reset. `--put` names the flashed board's own USB identity
  and is required for every row that can be a probe under test, so xtask can wait for it to
  re-enumerate. A DUT image takes `--via` alone. A row flashed through its own BOOTSEL drive,
  and any RP probe row given `--put` alone, goes to the `RPI-RP2` or `RP2350` volume at the
  PUT's USB port (on macOS, the only one mounted), refused if that ROM does not take the
  image's UF2 family;
- `hil <path> --via <sel> --put <sel> --target <name>` -- build the firmware and the target's
  DUT image, flash the firmware through `--via`, then run the daplink-test cases through the
  `--put` probe against the target. `--speed-extended` and `--speed-access <bytes>` pass through
  to the harness's speed cases. `--case <name>`, repeated, runs only those cases in that order,
  and `--no-put-flash` tests the probe's resident image, with no `--via`.

`--probe` is accepted in its older meanings: the transport for `flash`, as for probe-rs, and the
probe under test for `hil`, as for the harness.

Flashing pre-verifies, so a byte-identical resident image is not rewritten. Selectors are
always full `vid:pid:serial`; many probes share a VID:PID.
