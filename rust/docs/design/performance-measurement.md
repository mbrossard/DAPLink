# Performance measurement

A tool result describes an experiment, not a physical clock or a diagnosis. HIL speed cases report
block-transfer throughput without a threshold; the focused instruments below vary the workload.

| Tool | Observation | Important limit |
|---|---|---|
| `dap-bench` | CMSIS-DAP block throughput versus requested clock, direction and pipeline depth | Fit and plateau labels do not identify a physical clock or bottleneck |
| `swo-bench` | Sequence-tagged ITM delivery, framing and reported overflow under optional SWD load | Missing status or unserved data can leave loss/backlog unknown |
| `vcom-bench` | Marker-framed serial delivery plus available host, DUT and probe counters | Counter coverage and units determine which losses can be attributed |

[dap-bench](../../tools/dap-bench/src/main.rs) uses raw CMSIS-DAP bulk transfers and fits elapsed
time against inverse requested clock. It can halt or write the target.

[swo-bench](../../tools/swo-bench/src/main.rs) configures its DUT over RTT and drains SWO through
CMSIS-DAP, optionally under SWD reads. Unknown or incomplete evidence is reported as such, never
as zero loss.

The [VCOM reference](../../tools/vcom-bench/README.md) owns its verbs and record fields.

When comparing records, keep the tool version, image, workload, transport parameters, requested
and realized rates, completion state and integrity verdict. An incomplete run is not a clean run.
JSON schemas and labels are compatibility surfaces. No tool grade replaces the
[HIL workflow](hil-testing.md).
