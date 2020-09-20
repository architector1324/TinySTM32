#!/bin/bash

mkdir -p out

arm-none-eabi-gcc -O3 -Wall \
                  -g -gdwarf-2 \
                  -fdata-sections \
                  -ffunction-sections  \
                  -mcpu=cortex-m3 -mthumb \
                  -D HSE_VALUE=8000000 \
                  -D HSI_VALUE=8000000 \
                  -D STM32F103xB \
                  -I cmsis\
                  -Wa,-a,-ad,-alms=out/main.lst \
                  -T cmsis/STM32F103C8Tx_FLASH.ld \
                  -specs=nano.specs \
                  -lc -lm -lnosys \
                  -Wl,-Map=out/main.map,--cref -Wl,--gc-sections \
                  cmsis/startup_stm32f103xb.s \
                  cmsis/system_stm32f1xx.c \
                  main.c \
                  -o out/main.elf

arm-none-eabi-size out/main.elf
arm-none-eabi-objcopy -O binary -S out/main.elf out/main.bin	
