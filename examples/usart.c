#include "../lib/hal.h"


int main() {
    SystemInit();
    SystemCoreClockUpdate();

    hal_use_afio();
    hal_use_gpio(HAL_GPIOA);
    hal_use_uart(HAL_UART1);

    hal_uart_cfg_t uart = hal_uart_cfg_new(true, true, 9600);
    hal_uart_setup(HAL_UART1, uart);
    hal_uart_on(HAL_UART1);

    hal_uart_printf(HAL_UART1, "clock: %uHz\n", SystemCoreClock);
    hal_uart_print("Hello, World!\n", HAL_UART1);

    while(true) {
        hal_uart_print("% ", HAL_UART1);

        char buf[80] = {0};
        hal_uart_read(buf, '\n', true, HAL_UART1);
        hal_uart_print(buf, HAL_UART1);
    }

    return 0;
}
