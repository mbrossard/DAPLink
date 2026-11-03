# probe-reboot

Without `--reboot`, discovery reports the probe's claimed bootloader capability.
With `--reboot`, success requires affirmative arming followed by observed detach.
Unsupported reboot, submission failure, refusal and detach timeout fail the run.

A missing, malformed or unknown acknowledgement remains `UNCONFIRMED` and exits
nonzero even if the probe detaches. A caller may recover only by verifying the
expected post-reboot identity; detach alone does not establish reboot causation.
USB enumeration errors fail observation, rather than counting as absence.
The tool releases its USB handles before polling and never retries the command
or adopts a bootloader. Even success does not prove the expected ROM appeared.
