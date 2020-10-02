#include <stm32f1xx.h>
#include "lib/hal.h"

void SysTick_Handler() {
    hal_gpio_inv(HAL_GPIOC, 13);
}

int main() {
    SystemInit();
    SystemCoreClockUpdate();

    hal_gpio_en(HAL_GPIOC);
    hal_gpio_setup(HAL_GPIOC, 13, hal_gpio_cfg_new(HAL_GPIO_OUT, HAL_GPIO_2MHz));

    SysTick_Config(SystemCoreClock); // 1Hz
    while(1);

    return 0;
}
