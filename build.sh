#!/bin/bash

# usage: ./build.sh example/blink.c blink

mkdir -p out

arm-none-eabi-gcc -O3 -Wall \
                  -fdata-sections \
                  -ffunction-sections  \
                  -mcpu=cortex-m3 -mthumb \
                  -D HSE_VALUE=8000000 \
                  -D HSI_VALUE=8000000 \
                  -D STM32F103xB \
                  -I cmsis\
                  -Wa,-a,-ad,-alms=out/$2.lst \
                  -T cmsis/STM32F103C8Tx_FLASH.ld \
                  -specs=nano.specs \
                  -specs=nosys.specs \
                  -lc -lm -lnosys \
                  -Wl,-Map=out/$2.map,--cref -Wl,--gc-sections \
                  cmsis/startup_stm32f103xb.s \
                  cmsis/system_stm32f1xx.c \
                  $1 \
                  -o out/$2.elf

arm-none-eabi-size out/$2.elf
arm-none-eabi-objcopy -O binary -S out/$2.elf out/$2.bin
