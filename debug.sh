#!/bin/env sh

arm-none-eabi-gdb zig-out/bin/kbd-1-fw -ex "set confirm off" -ex 'target extended-remote | openocd -f interface/stlink.cfg -f target/stm32l0.cfg -c "gdb_port pipe; log_output openocd.log"'
