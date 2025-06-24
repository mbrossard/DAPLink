DAPLink
---

Installing make (assuming you have git bash):
- Go to [ezwinports](https://sourceforge.net/projects/ezwinports/files/).
- Download `make-4.1-2-without-guile-w32-bin.zip` (get the version without guile).
- Extract zip.
- Copy the contents to your `Git\mingw64\` merging the folders, but do NOT overwrite/replace any existing files. 

Install gcc:
- [GNU Arm Embedded Toolchain](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads) . This compiler will be identified as `gcc_arm`. It is recommended to use version `10.3-2021.10`. 
- Install to: `C:\gcc\10.2021.10\`

Install Python and run:
```
$ python -m venv .venv
$ source venv\Scripts\activate
$ pip install -r .\requirements.txt
$ pyocd pack update
$ pyocd pack install STM32H5xx
```


Compiling software:
- In the system path variable, make sure you have `c:/gcc/10.2021.10/bin/`, then run:
```
$ source venv/Scripts/activate
$ python tools/progen_compile.py -t make_gcc_arm --clean --parallel stm32l412xx_stm32h523re_if
```

Run integration test:
```
$ python test/run_test.py --targetdir {targetdirPath} --noloadif
```

If the board is not part of PyOCD and PyOCD is throwing an error, PyOCD can be ported. Another solution is to modify:
- `test/tests/test_hid.py` line 41
- `test/board.py` line 367
- `test/test/test_msd.py` line 431
 and add your target as option when PyOCD is called:
```
with ConnectHelper.session_with_chosen_probe(options={"target_override": "lpc55s69"}) as session:
```

Target test program for testing
--
Official DAPLink target program must be generated from [zdv](https://github.com/H4w4k4/zdv),
as the mbed api has been shutdown.

Compile test software from zdv:
- Use "set" instead of "export" for PowerShell
- GNUARMEMB_TOOLCHAIN_PATH cannot have whitespace
```
$ virtualenv venv
$ source venv/bin/activate
$ mkdir zdv-workspace
$ cd zdv-workspace
$ export ZEPHYR_TOOLCHAIN_VARIANT=gnuarmemb
$ export GNUARMEMB_TOOLCHAIN_PATH=/<CHANGE_ME>/gcc-arm-none-eabi-10.3-2021.10
$ west init -m https://github.com/H4w4k4/zdv
$ west update
$ pip3 install -r zephyr/scripts/requirements.txt
$ make -f zdv/build.mk
```

For stm32h523retx see [here](https://github.com/H4w4k4/DAPLink_Test).

