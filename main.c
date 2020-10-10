#include <stm32f1xx.h>
#include "lib/hal.h"


void tim_hlr() {
    hal_gpio_inv(HAL_GPIOC, 13);
}

int main() {
    SystemInit();
    SystemCoreClockUpdate();

    hal_use_gpio(HAL_GPIOC);
    hal_use_timer(HAL_TIMER2);

    hal_gpio_cfg pin = hal_gpio_cfg_new(HAL_GPIO_OUT, HAL_GPIO_2MHz);
    hal_timer_cfg tim = hal_timer_cfg_simple(1); // 1Hz
    tim.irq = tim_hlr;

    hal_gpio_setup(HAL_GPIOC, 13, pin);
    hal_timer_setup(HAL_TIMER2, tim);
    hal_timer_start(HAL_TIMER2);

    while(1);

    return 0;
}
