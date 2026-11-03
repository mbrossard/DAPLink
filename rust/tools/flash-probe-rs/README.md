# flash-probe-rs

Pass `--probe <selector>` when more than one debug probe is attached; omission
requires exactly one probe. `--help` exits successfully without opening a probe.

The self-test attempts teardown once after successful initialization, including
after erase, program or verify failure. A teardown failure also fails the run;
when both fail, the original operation error and teardown error are reported.
Teardown is best effort and does not prove target release. The target stays halted.
