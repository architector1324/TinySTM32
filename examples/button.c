#include <stm32f1xx.h>
#include "../lib/hal.h"


void EXTI1_IRQHandler() {
    EXTI->PR |= EXTI_PR_PR1;
    hal_gpio_inv(HAL_GPIOA, 10);
}

int main() {
    hal_use_afio();
    hal_use_gpio(HAL_GPIOA);
    hal_use_gpio(HAL_GPIOB);

    hal_gpio_cfg_t led = hal_gpio_cfg_new(HAL_GPIO_OUT, HAL_GPIO_2MHz);
    hal_gpio_cfg_t but = hal_gpio_cfg_new(HAL_GPIO_IN_PULL, HAL_GPIO_INMODE);

    hal_gpio_setup(HAL_GPIOA, 10, led); // led
    hal_gpio_setup(HAL_GPIOB, 1, but); // button
    GPIOB->ODR |= GPIO_ODR_ODR1; // pull-up

    // button interrupt
    AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI1_PB;
    EXTI->IMR |= EXTI_IMR_MR1;
    EXTI->RTSR |= EXTI_RTSR_TR1;
	EXTI->FTSR &= ~EXTI_FTSR_TR1;
    NVIC_EnableIRQ(EXTI1_IRQn);

    // led on
    hal_gpio_w(HAL_GPIOA, 10, HAL_GPIO_HIGH);
    while(1);

    return 0;
}
