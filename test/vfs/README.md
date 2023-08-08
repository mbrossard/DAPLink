# VFS Tests

## Building

From this directory run:
```
$ cmake -S . -B build
$ make -C build
```

## Test files

From DAPLink top-level directory:
```
$ python tools/progen_compile.py --parallel --clean \
    nrf52820_microbit_bl nrf52820_microbit_if \
    kl27z_microbit_bl kl27z_microbit_if kl26z_microbit_if
$ mkdir -p test/vfs/files
$ cp \
  ./projectfiles/make_gcc_arm/nrf52820_microbit_if/build/nrf52820_microbit_if_crc.bin \
  ./projectfiles/make_gcc_arm/nrf52820_microbit_if/build/nrf52820_microbit_if_crc.hex \
  ./projectfiles/make_gcc_arm/nrf52820_microbit_bl/build/nrf52820_microbit_bl_crc.bin \
  ./projectfiles/make_gcc_arm/nrf52820_microbit_bl/build/nrf52820_microbit_bl_crc.hex \
  ./projectfiles/make_gcc_arm/kl27z_microbit_bl/build/kl27z_microbit_bl_crc.bin \
  ./projectfiles/make_gcc_arm/kl27z_microbit_bl/build/kl27z_microbit_bl_crc.hex \
  ./projectfiles/make_gcc_arm/kl27z_microbit_if/build/kl27z_microbit_if_crc.bin \
  ./projectfiles/make_gcc_arm/kl27z_microbit_if/build/kl27z_microbit_if_crc.hex \
  ./projectfiles/make_gcc_arm/kl26z_microbit_if/build/kl26z_microbit_if_crc.bin \
  ./projectfiles/make_gcc_arm/kl26z_microbit_if/build/kl26z_microbit_if_crc.hex \
  test/vfs/files
```

Download `Out of box experience.hex` from [Out of box experience](https://microbit.org/get-started/user-guide/out-of-box-experience/) into `test/vfs` directory. Use the split universal tool then put the `ih-9900.hex` and `ih-9903.hex` file in `test/vfs/files`.

```
$ ghead -n -4 files/ih-9900.hex > files/OOBE-9900.hex
$ tail -n 1 files/ih-9900.hex >> files/OOBE-9900.hex
$ arm-none-eabi-objcopy --gap-fill 0xFF -Iihex -Obinary files/OOBE-9900.hex files/OOBE-9900.bin
$ ghead -n -4 files/ih-9903.hex > files/OOBE-9903.hex
$ tail -n 1 files/ih-9903.hex >> files/OOBE-9903.hex
$ arm-none-eabi-objcopy --gap-fill 0xFF -Iihex -Obinary files/OOBE-9903.hex files/OOBE-9903.bin
```

## Running the tests

From this directory run the different test binaries:
```
$ ./build/vfs_test_kl26z_if
[...]
$ ./build/vfs_test_kl27z_bl
[...]
$ ./build/vfs_test_kl27z_if
[...]
$ ./build/vfs_test_nrf52820_bl
[...]
$ ./build/vfs_test_nrf52820_if
[...]
```
