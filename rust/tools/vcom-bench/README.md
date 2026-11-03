# vcom-bench

Host-side VCOM measurement bench. It derives the probe/DUT shared baud lattice from both ends'
realized rates, prunes rates the ends disagree on or the USB bus cannot serve, and searches for
the ceiling the pairing holds. Both ends transmit a marker-framed PRNG stream (the v1 stream);
each window appends a JSON line to `artifacts/vcom-bench.jsonl` at the repository root.

    vcom-bench ladder --probe 0d28:0204:<serial> [--dap representative] [--full]
    vcom-bench run    --probe <sel> --baud 2625000 [--mode duplex|credit|return-priority|out|back]
                      [--window 30 | --qualify | --soak] [--burst 512 --idle 1]
    vcom-bench echo   --probe <sel> --baud 2625000 --len 513 --chunk 64 --mode paced
    vcom-bench contract --probe <sel> [--baud 115200] [--qualify]

A rung passes when both end-of-stream frames arrived, neither receiver saw corrupt, missing,
duplicated or unresolved data, the DUT counted no UART errors, the ledgers reconcile, and each
direction claiming the line rate held 99% of its payload budget. `run` also brackets its windows
with the probe's own byte ledger and fails a window that does not reconcile. `echo` is a
regression reference, not a source of ceilings.

`contract` checks small paced exchanges alongside DAP traffic, idle-to-burst recovery, reader
pauses, and line-coding and CDC close/reopen cycles; `--qualify` runs more cycles and a longer
idle.

Needs a DUT test image serving the RTT `BAUD`, `STATS` and `V1` verbs; the probe's realized rate
and byte ledger come from its CMSIS-DAP vendor queries where the firmware serves them.

Every record carries `firmware_row` and `firmware_row_source`: the row the image answers on
`DAP_Info` `0x09` (`image`), else `--firmware-row <row>` (`operator`), else null (`none`). A
disagreement, a value outside the firmware registry or no row prints a warning, and the run goes
on. `external:<name>` names an image outside the registry.
