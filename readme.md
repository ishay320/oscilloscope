# STM32 device setup

## tools

download `https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads` and put in in place with `PATH` (msys64 or something like that)

### compile

1. arm-none-eabi-gcc
1. make

use `make` command to compile

### upload

1. st-flash - flash the device

use `make upload` command to upload

example:

```
$ st-flash.exe write <file path>.bin 0x8000000
```

### debug

1. install `cortex-debug` vscode extension
1. arm-none-eabi-gdb >= 9.0
1. st-util - for debug server

use vscode to debug

### monitor

- putty
- arduinoIDE

use `make monitor` command to monitor
