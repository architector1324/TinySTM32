#include <stm32f1xx.h>

#include "../lib/hal.h"
#include "../lib/font5x7.h"
#include "../lib/images.h"
#include "../lib/fixed.h"

#define ST7789_USE_GFX_DRV
#include "../lib/st7789.h"


uint16_t color_f(int16_t x, int16_t y) {
    // 256 * (cos(x) + sin(y))
    return fxd_to_int(fxd_muli(256, fxd_cos(x) + fxd_sin(y)));
}

int main() {
    SystemCoreClockUpdate();

    hal_use_afio();
    hal_use_gpio(HAL_GPIOA);
    hal_use_spi(HAL_SPI1);

    hal_gpio_cfg_t cfg = hal_gpio_cfg_new(HAL_GPIO_OUT, HAL_GPIO_50MHz);
    hal_spi_cfg_t spi_cfg = hal_spi_cfg_new(HAL_SPI_MASTER, HAL_SPI_SIMPLEX, HAL_SPI_BAUD2);

    hal_gpio_setup(HAL_GPIOA, 0, cfg); // dc
    hal_gpio_setup(HAL_GPIOA, 2, cfg); // rst
    hal_spi_setup(HAL_SPI1, spi_cfg); // pa5(sck), pa6(miso), pa7(mosi)

    hal_spi_on(HAL_SPI1);

    // init
    st7789_t disp = {
        .spi = HAL_SPI1,
        .dc = hal_gpio_pin_new(HAL_GPIOA, 0),
        .rst = hal_gpio_pin_new(HAL_GPIOA, 2)
    };
    st7789_init(&disp);

    gfx_drv_t gfx;
    st7789_drv_init(&gfx, &disp);

    // text
    gfx_fill(0x0000, &gfx);
    gfx_text("Text", 120 - 6 * 4, 120 - 8, 2, 0xffff, font5x7_ascii, &gfx);
    hal_delay(2000);

    // ascii table
    gfx_fill(0x0000, &gfx);
    for(uint8_t ch = 0; ch < 255; ch++)
        gfx_char(ch, (ch % 30) * 8, 240 - 8 - (ch / 30) * 8, 1, 0xffff, font5x7_ascii, &gfx);
    hal_delay(2000);

    // sprite
    gfx_fill(0x0000, &gfx);
    gfx_sprite(sonic32x32, 120 - 16, 120 - 16, 32, 32, &gfx);
    hal_delay(5000);

    // math
    gfx_func(120 - 32, 120 - 32, 64, 64, color_f, &gfx);
    hal_delay(5000);

    return 0;
}
