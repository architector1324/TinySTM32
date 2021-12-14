#include <stm32f1xx.h>
#include "../lib/hal.h"


void adc_hlr() {
    uint16_t val = 1000 * (100 * (hal_adc_r(HAL_ADC1) / 100)) / 4095;
    hal_timer_set_dc(HAL_TIMER2, val);
}

int main() {
    SystemInit();
    SystemCoreClockUpdate();

    hal_use_afio();
    hal_use_gpio(HAL_GPIOA);
    hal_use_timer(HAL_TIMER2);
    hal_use_adc(HAL_ADC1);

    hal_gpio_cfg_t in_cfg = hal_gpio_cfg_new(HAL_GPIO_AIN, HAL_GPIO_INMODE);
    hal_gpio_cfg_t out_cfg = hal_gpio_cfg_new(HAL_GPIO_AOUT, HAL_GPIO_2MHz);

    hal_adc_cfg_t adc_cfg = hal_adc_cfg_new(0, HAL_ADC_ONCE);
    adc_cfg.irq = adc_hlr;

    hal_timer_cfg_t tim_cfg = hal_timer_cfg_simple(1000); // 1KHz
    tim_cfg.pwm = HAL_PWM2;

    hal_gpio_setup(HAL_GPIOA, 0, in_cfg); // potentiometer
    hal_gpio_setup(HAL_GPIOA, 1, out_cfg); // led
    hal_timer_setup(HAL_TIMER2, tim_cfg);
    hal_adc_setup(HAL_ADC1, adc_cfg);

    hal_timer_on(HAL_TIMER2);
    hal_adc_on(HAL_ADC1);

    while(1)
        hal_adc_go(HAL_ADC1);

    return 0;
}
