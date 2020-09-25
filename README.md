# TinySTM32
My collection of tiny libraries for stm32 and other peripheral.

Supported devices:
- stm32f103c8
- st7735s (80x160 IPS TFT)
- st7789 (240x240 IPS TFT)

Collection:
| Library  | Description |
| ------------- | ------------- |
| `hal.h`      |  Tiny hardware abstraction layer for stm32f103c8. |
| `font5x7.h`  | Ascii and tiny unicode (U+0020..U+303F) 5x7 fonts. |
| `st7735s.h`  | Driver for 80x160 IPS TFT based on st7735s controller. |
|  `st7789.h` | Driver for 240x240 IPS TFT based on st7789 controller. |
| `fixed.h`   | Fast fixed point library. |

Requirements:
- arm-none-eabi-gcc
- stlink
