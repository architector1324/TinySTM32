#!/bin/bash

# usage: ./flash.sh blink

st-flash --reset write out/$1.bin 0x8000000
