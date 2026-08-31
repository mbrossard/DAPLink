# vcom-bench

Host-side VCOM measurement bench. It derives the probe/DUT shared baud lattice from both ends'
realized rates, prunes rates the two ends disagree on and points the USB bus cannot serve, and
searches for the ceiling the pairing holds. Traffic is the v1 stream: both ends transmit a
marker-framed PRNG stream for a byte budget fixed before the run and stop themselves with an
end-of-stream frame, so the control channel stays silent for the whole window and every byte is
verified, classified, and charged to one direction. Each window appends a JSON line to
`artifacts/vcom-bench.jsonl` at the repository root.

    vcom-bench ladder --probe 0d28:0204:<serial> [--dap representative] [--full]
    vcom-bench run    --probe <sel> --baud 2625000 [--mode duplex|credit|return-priority|out|back]
                      [--window 30 | --qualify | --soak] [--burst 512 --idle 1]
    vcom-bench echo   --probe <sel> --baud 2625000 --len 513 --chunk 64 --mode paced
    vcom-bench contract --probe <sel> [--baud 115200] [--qualify]

A rung passes when both end-of-stream frames arrived, nothing was corrupt, missing, duplicated
or unresolved at either receiver, the DUT counted no UART errors, the ledgers reconcile, and
each direction claiming the line rate held 99% of the payload budget over the nominal window.
`run` additionally reads the probe's own byte ledger either side of its windows and fails them
if it does not reconcile or charges a byte to a loss site. `echo` is the regression reference,
not a source of ceilings. Needs a DUT test image serving the RTT `BAUD`, `STATS` and `V1` verbs;
the probe's realized rate and byte ledger come from the CMSIS-DAP vendor queries where the
firmware serves them.

`contract` checks small paced exchanges with representative DAP traffic, idle-to-burst recovery,
three application-reader pauses, 10 line-coding and CDC close/reopen cycles, and a rate change
requested behind a full transmit queue. `--qualify` uses 100 cycles and extends the idle case to
five minutes. USB enumeration cycling remains a separately authorized bench gate and is not part
of this verb.
