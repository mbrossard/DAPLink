# Linux host setup

Unprivileged access to a DAPLink probe needs a udev rule for its device node. The probe-rs rules
already cover the CMSIS-DAP identity every firmware here enumerates with; without them, this is
enough (`/etc/udev/rules.d/50-cmsis-dap.rules`, then `udevadm control --reload` and replug):

```text
SUBSYSTEM=="usb", ATTR{idVendor}=="0d28", ATTR{idProduct}=="0204", MODE="0660", TAG+="uaccess"
```

The CDC serial port and the mass-storage disk are bound by kernel drivers and need no rule.

The `reattach-suspend` test case is the one thing that needs more: it idles the probe into USB
runtime suspend by writing `auto` to the device's sysfs `power/control`, which is root-owned
regardless of the device node's mode. Without write access the case prints a warning and skips.
To let it run, hand that attribute to the user's group when the probe appears:

```text
SUBSYSTEM=="usb", ATTR{idVendor}=="0d28", ATTR{idProduct}=="0204", \
  RUN+="/bin/sh -c 'chgrp <group> %S%p/power/control && chmod g+w %S%p/power/control'"
```

The case restores `power/control` to what it found, so the probe does not stay on `auto`.
