#include <stm32f1xx.h>
#include "../lib/sdhc.h"


int main() {
    SystemInit();
    SystemCoreClockUpdate();

    hal_use_afio();
    hal_use_gpio(HAL_GPIOB);
    hal_use_gpio(HAL_GPIOC);
    hal_use_spi(HAL_SPI2);

    hal_gpio_cfg_t cfg = hal_gpio_cfg_new(HAL_GPIO_OUT, HAL_GPIO_50MHz);
    hal_spi_cfg_t spi_cfg = hal_spi_cfg_new(HAL_SPI_MASTER, HAL_SPI_FULL_DUPLEX, HAL_SPI_BAUD64);  // 125 KHz (system clock = 8MHz)
    hal_gpio_cfg_t led_cfg = hal_gpio_cfg_new(HAL_GPIO_OUT, HAL_GPIO_2MHz);

    hal_gpio_setup(HAL_GPIOC, 13, led_cfg);
    hal_gpio_setup(HAL_GPIOB, 12, cfg); // cs
    hal_spi_setup(HAL_SPI2, spi_cfg); // pb13(sck), pb14(miso), pb15(mosi)

    hal_spi_on(HAL_SPI2);
    hal_gpio_w(HAL_GPIOC, 13, HAL_GPIO_HIGH);

    sdhc_t sd = {
        .spi = HAL_SPI2,
        .cs = hal_gpio_pin_new(HAL_GPIOB, 12),
    };

    // init sd
    hal_gpio_w(HAL_GPIOC, 13, HAL_GPIO_LOW);

    sdhc_status_t res = sdhc_init(&sd);
    if(res != SD_OK) return 0;

    hal_gpio_w(HAL_GPIOC, 13, HAL_GPIO_HIGH);

    // reconfigure spi
    spi_cfg.baud_factor = HAL_SPI_BAUD2;
    hal_spi_off(HAL_SPI2);
    hal_spi_setup(HAL_SPI2, spi_cfg);
    hal_spi_on(HAL_SPI2);


    // IO
    static const char dat[512] = "Hello, MicroSD!";
    static char dat2[512];

    hal_gpio_w(HAL_GPIOC, 13, HAL_GPIO_LOW);
    res = sdhc_w((const uint8_t*)dat, 0, &sd);
    hal_gpio_w(HAL_GPIOC, 13, HAL_GPIO_HIGH);

    hal_gpio_w(HAL_GPIOC, 13, HAL_GPIO_LOW);
    res = sdhc_r((uint8_t*)dat2, 0, &sd);
    hal_gpio_w(HAL_GPIOC, 13, HAL_GPIO_HIGH);

    return 0;
}
