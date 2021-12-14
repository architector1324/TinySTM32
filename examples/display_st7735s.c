#include <stm32f1xx.h>

#include "../lib/hal.h"
#include "../lib/font5x7.h"
#include "../lib/images.h"
#include "../lib/fixed.h"

#define ST7735s_USE_GFX_DRV
#include "../lib/st7735s.h"


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
    hal_gpio_setup(HAL_GPIOA, 1, cfg); // cs
    hal_gpio_setup(HAL_GPIOA, 2, cfg); // rst
    hal_spi_setup(HAL_SPI1, spi_cfg); // pa5(sck), pa7(mosi)

    hal_spi_on(HAL_SPI1);

    // init
    st7735s_t disp = {
        .spi = HAL_SPI1,
        .dc = hal_gpio_pin_new(HAL_GPIOA, 0),
        .cs = hal_gpio_pin_new(HAL_GPIOA, 1),
        .rst = hal_gpio_pin_new(HAL_GPIOA, 2),
    };
    st7735s_init(&disp);

    gfx_drv_t gfx;
    st7735s_drv_init(&gfx, &disp);

    // text
    gfx_fill(0x0000, &gfx);
    gfx_text("Text", 80 - 6 * 4, 40 - 8, 2, 0xffff, font5x7_ascii, &gfx);
    hal_delay(2500);

    // ascii table
    gfx_fill(0x0000, &gfx);
    for(uint8_t ch = 0; ch < 200; ch++)
        gfx_char(ch, (ch % 20) * 8, 72 - (ch / 20) * 8, 1, 0xffff, font5x7_ascii, &gfx);
    hal_delay(5000);

    // sprite
    gfx_fill(0x0000, &gfx);
    gfx_sprite(sonic32x32, 80 - 16, 40 - 16, 32, 32, &gfx);
    hal_delay(5000);

    // math
    gfx_func(80 - 32, 40 - 32, 64, 64, color_f, &gfx);
    hal_delay(5000);

    while(1) {
        gfx_fill_img(jojo160x80, &gfx);
        hal_delay(5000);

        gfx_fill_img(naruto160x80, &gfx);
        hal_delay(5000);
    }

    return 0;
}
