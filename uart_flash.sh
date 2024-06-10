#!/bin/sh
stm32flash -w build/j1850_reader.bin -v -g 0x0 /dev/ttyUSB0
