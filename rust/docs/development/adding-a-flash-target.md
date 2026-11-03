# Adding a flash target

[Target flashing](../design/target-flashing.md) is split between the generic runtime in
`daplink-flash`, checked-in target records in `daplink-targets`, and the firmware that chooses which
records to include. A target record must describe one compatible chip or chip group; do not
silently shorten a larger device's memory map to represent a smaller part.

## Add the target record

Target modules under `daplink-targets/src/targets/<chip>.rs` are checked-in generated records.
A contribution must supply reviewable target and algorithm data with its generation inputs or
method; an existing blob or a guessed memory map is not evidence for another chip.

In `crates/daplink-targets`:

1. add or reuse a flash algorithm module under `src/algorithms/`;
2. add a family reset or unlock sequence under `src/families/` only when generic behavior is not
   sufficient;
3. produce `src/targets/<chip>.rs` with flash regions, RAM regions, sector geometry, identity
   probes, reset behavior, debug-port requirements, UF2 family when defined, and
   `PROGRAM_BUFFER_SIZE`;
4. export the modules from `src/lib.rs` under a public chip feature;
5. make that feature select any private `_algo_*`/`_family_*` features it needs.

Firmware sizes the shared scratch buffer from `PROGRAM_BUFFER_SIZE`. Identity checks should
reject incompatible geometry before erase or programming.

Add host tests for region bounds, sector coverage, algorithm RAM placement, identity masks, reset
selection, and debug-port metadata. Verify the redistribution terms and required notices of an
imported algorithm payload before including it.

## Compose it into firmware

Add a firmware feature such as `flash-<chip>` that enables both the common `flash` support and the
matching `daplink-targets` chip feature. Add the target to the firmware's `target_table!` rows with
its `PROGRAM_BUFFER_SIZE`. The macro derives the selector table, target count, and maximum scratch
allocation from that list, but not the primary target: firmware declares its own `PRIMARY` under
its feature cfgs, choosing among the targets it built in. When a row enables more than one target,
give every payload block a UF2 family that uniquely identifies its target -- there is no primary
fallback, so an ambiguous or missing family is rejected rather than silently routed to `PRIMARY`.

Add an xtask firmware row for every supported flash-enabled build shape. The HIL `TARGETS` table
and `tests/daplink-test/src/target.rs` list wired test targets, not the flash-target catalog.

Host-test the target feature, run `cargo xtask build-all`, then run the `flash`, `msc`, and
`vendor-flash` HIL cases against a wired target.
