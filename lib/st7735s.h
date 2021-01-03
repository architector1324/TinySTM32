#ifndef _ST7735S_H_
#define _ST7735S_H_

#include <stm32f1xx.h>
#include "hal.h"

#ifdef ST7735s_USE_GFX_DRV
#include "gfx.h"
#endif

#define ST7735s_W 160
#define ST7735s_H 80

////////////////////////////////
//         DEFINITION         //
////////////////////////////////
typedef struct {
    hal_spi_t spi;
    hal_gpio_pin_t dc;
    hal_gpio_pin_t cs;
    hal_gpio_pin_t rst;
} st7735s_t;

void st7735s_cmd(uint8_t cmd, const st7735s_t* disp);
void st7735s_dat(uint8_t dat, const st7735s_t* disp);
void st7735s_dat16(uint16_t dat, const st7735s_t* disp);

void st7735s_init(const st7735s_t* disp);

void st7735s_win(int16_t x0, int16_t y0, uint8_t w, uint8_t h, const st7735s_t* disp);
void st7735s_cur(int16_t x, int16_t y, const st7735s_t* disp);
void st7735s_def(const st7735s_t* disp);

void st7735s_wrt(const st7735s_t* disp);
void st7735s_pixel(int16_t x, int16_t y, uint16_t color, const st7735s_t* disp);

#ifdef ST7735s_USE_GFX_DRV
void st7735s_drv_init(gfx_drv_t* drv, st7735s_t* disp);
#endif

////////////////////////////////
//       IMPLEMENTATION       //
////////////////////////////////

void st7735s_cmd(uint8_t cmd, const st7735s_t* disp) {
    hal_gpio_w(disp->dc.port, disp->dc.pin, HAL_GPIO_LOW);
    hal_spi_sel(&disp->cs);

    hal_spi_w(cmd, disp->spi);

    hal_spi_desel(&disp->cs);
}

void st7735s_dat(uint8_t dat, const st7735s_t* disp) {
    hal_gpio_w(disp->dc.port, disp->dc.pin, HAL_GPIO_HIGH);
    hal_spi_sel(&disp->cs);

    hal_spi_w(dat, disp->spi);

    hal_spi_desel(&disp->cs);
}

void st7735s_dat16(uint16_t dat, const st7735s_t* disp){
    hal_gpio_w(disp->dc.port, disp->dc.pin, HAL_GPIO_HIGH);
    hal_spi_sel(&disp->cs);

	hal_spi_w16(dat, disp->spi);

    hal_spi_desel(&disp->cs);
}

void st7735s_init(const st7735s_t* disp) {
    // reset display
    hal_gpio_w(disp->rst.port, disp->rst.pin, HAL_GPIO_LOW);
    hal_delay(10);

    hal_gpio_w(disp->rst.port, disp->rst.pin, HAL_GPIO_HIGH);
    hal_delay(10);

    st7735s_cmd(0x01, disp); // software reset
    hal_delay(120);

    st7735s_cmd(0x28, disp); // off
    hal_delay(120);

    st7735s_cmd(0x11, disp); // sleep out
    hal_delay(120);

    st7735s_cmd(0xb1, disp); // framerate
    st7735s_dat(0x00, disp);
    st7735s_dat(0x18, disp);

    st7735s_cmd(0x36, disp); // set orientation
    st7735s_dat(0x8, disp);

    st7735s_def(disp);
    st7735s_cmd(0x21, disp); // inverse colors

    st7735s_cmd(0x26, disp); // gamma
    st7735s_dat(0x2, disp);

    st7735s_cmd(0x3a, disp); // rgb565 color mode
    st7735s_dat(0x05, disp);

    st7735s_cmd(0x29, disp); // on
    hal_delay(100);
}

void st7735s_win(int16_t x0, int16_t y0, uint8_t w, uint8_t h, const st7735s_t* disp) {
    st7735s_cmd(0x2a, disp);
    st7735s_dat16(y0 + 26, disp);
    st7735s_dat16(h + y0 - 1 + 26, disp);

    st7735s_cmd(0x2b, disp);
    st7735s_dat16(x0 + 1, disp);
    st7735s_dat16(w + x0 - 1 + 1, disp);
}

void st7735s_cur(int16_t x, int16_t y, const st7735s_t* disp) {
    st7735s_win(x, y, 1, 1, disp);
}

void st7735s_def(const st7735s_t* disp) {
    st7735s_win(0, 0, ST7735s_W, ST7735s_H, disp);
}

void st7735s_wrt(const st7735s_t* disp) {
    st7735s_cmd(0x2c, disp);
}

void st7735s_pixel(int16_t x, int16_t y, uint16_t color, const st7735s_t* disp) {
    st7735s_cur(x, y, disp);
    st7735s_cmd(0x2c, disp);
    st7735s_dat16(color, disp);

    st7735s_def(disp);
}

#ifdef ST7735s_USE_GFX_DRV
void st7735s_drv_init(gfx_drv_t* drv, st7735s_t* disp) {
    drv->w = ST7735s_W;
    drv->h = ST7735s_H;
    drv->disp = (void*)disp;

    drv->win = st7735s_win;
    drv->cur = st7735s_cur;

    drv->pix = st7735s_pixel;
    drv->wrt = st7735s_wrt;
    drv->dat8 = st7735s_dat;
    drv->dat16 = st7735s_dat16;
}
#endif

#endif
