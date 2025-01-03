DAPLink
---

Compiling software:
```
$ python tools/progen_compile.py -t make_gcc_arm --clean --parallel stm32l412xx_lpc55s69_if
```

Run test:
```
$ python test/run_test.py --targetdir P:/git/daplink/test_file/ --noloadif
```

If the board is not part of PyOCD, PyOCD can be ported. Another solution is to modify `test/tests/test_hid.py`, line 41 to add your target as option when PyOCD is called:
```
with ConnectHelper.session_with_chosen_probe(options={"target_override": "lpc55s69"}) as session:
```


zdv
--
Test file for DAPLink testing, generated from [zdv](https://github.com/H4w4k4/zdv)

Compile test software:
- mbed api has been shutdown
- Use "set" instead of "export" for PowerShell
- GNUARMEMB_TOOLCHAIN_PATH cannot have withspace
```
$ virtualenv venv
$ source venv/bin/activate
$ mkdir zdv-workspace
$ cd zdv-workspace
$ export ZEPHYR_TOOLCHAIN_VARIANT=gnuarmemb
$ export GNUARMEMB_TOOLCHAIN_PATH=/<CHANGE_ME>/gcc-arm-none-eabi-10.3-2021.10
$ west init -m https://github.com/mbrossard/zdv.git
$ west update
$ pip3 install -r zephyr/scripts/requirements.txt
$ make -f zdv/build.mk
```


PyOCD
---
Connecting to target:
```
pyocd cmd --target lpc55s69
```


