#include <stm32f1xx.h>


void SysTick_Handler() {
    GPIOC->ODR ^= GPIO_ODR_ODR13;
}

int main() {
    SystemInit();
    SystemCoreClockUpdate();

    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
    GPIOC->CRH |= GPIO_CRH_MODE13_1; // 2Mhz
    GPIOC->CRH &= ~GPIO_CRH_CNF13; // push-pull

    SysTick_Config(SystemCoreClock); // 1Hz

    while(1);

    return 0;
}
