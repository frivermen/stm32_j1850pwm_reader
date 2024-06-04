#!/bin/sh
st-flash --reset write build/j1850_reader.bin 0x8000000
