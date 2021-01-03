#ifndef _ST7789_H_
#define _ST7789_H_

#include <stm32f1xx.h>
#include "hal.h"

#ifdef ST7789_USE_GFX_DRV
#include "gfx.h"
#endif

#define ST7789_W 240
#define ST7789_H 240

////////////////////////////////
//         DEFINITION         //
////////////////////////////////

typedef struct {
    hal_spi_t spi;
    hal_gpio_pin_t dc;
    hal_gpio_pin_t rst;
} st7789_t;

void st7789_cmd(uint8_t cmd, const st7789_t* disp);
void st7789_dat(uint8_t dat, const st7789_t* disp);
void st7789_dat16(uint16_t dat, const st7789_t* disp);

void st7789_init(st7789_t* disp);

void st7789_win(int16_t x0, int16_t y0, uint8_t w, uint8_t h, const st7789_t* disp);
void st7789_cur(int16_t x, int16_t y, const st7789_t* disp);
void st7789_def(const st7789_t* disp);

void st7789_wrt(const st7789_t* disp);
void st7789_pixel(int16_t x, int16_t y, uint16_t color, const st7789_t* disp);

#ifdef ST7789_USE_GFX_DRV
void st7789_drv_init(gfx_drv_t* drv, st7789_t* disp);
#endif

////////////////////////////////
//       IMPLEMENTATION       //
////////////////////////////////

void st7789_cmd(uint8_t cmd, const st7789_t* disp) {
    hal_gpio_w(disp->dc.port, disp->dc.pin, HAL_GPIO_LOW);
    hal_spi_w(cmd, disp->spi);
}

void st7789_dat(uint8_t dat, const st7789_t* disp) {
    hal_gpio_w(disp->dc.port, disp->dc.pin, HAL_GPIO_HIGH);
    hal_spi_w(dat, disp->spi);
}

void st7789_dat16(uint16_t dat, const st7789_t* disp) {
    hal_gpio_w(disp->dc.port, disp->dc.pin, HAL_GPIO_HIGH);
	hal_spi_w16(dat, disp->spi);
}

void st7789_init(st7789_t* disp) {
    // reset display
    hal_gpio_w(disp->rst.port, disp->rst.pin, HAL_GPIO_LOW);
    hal_delay(10);

    hal_gpio_w(disp->rst.port, disp->rst.pin, HAL_GPIO_HIGH);
    hal_delay(10);

    st7789_cmd(0x01, disp); // software reset
    hal_delay(120);

    st7789_cmd(0x28, disp); // off
    hal_delay(120);

    st7789_cmd(0x11, disp); // sleep out
    hal_delay(120);

    st7789_cmd(0xb1, disp); // framerate
    st7789_dat(0x00, disp);
    st7789_dat(0x18, disp);

    st7789_cmd(0x36, disp); // set orientation
    st7789_dat(0, disp);

    st7789_def(disp);
    st7789_cmd(0x21, disp); // inverse colors

    st7789_cmd(0x26, disp); // gamma
    st7789_dat(0x2, disp);

    st7789_cmd(0x3a, disp); // rgb565 color mode
    st7789_dat(0x05, disp);

    st7789_cmd(0x29, disp); // on
    hal_delay(100);
}

void st7789_win(int16_t x0, int16_t y0, uint8_t w, uint8_t h, const st7789_t* disp) {
    st7789_cmd(0x2a, disp);
    st7789_dat16(y0, disp);
    st7789_dat16(h + y0 - 1, disp);

    st7789_cmd(0x2b, disp);
    st7789_dat16(x0, disp);
    st7789_dat16(w + x0 - 1, disp);
}

void st7789_cur(int16_t x, int16_t y, const st7789_t* disp) {
    st7789_win(x, y, 1, 1, disp);
}

void st7789_def(const st7789_t* disp) {
    st7789_win(0, 0, ST7789_W, ST7789_H, disp);
}

void st7789_wrt(const st7789_t* disp) {
    st7789_cmd(0x2c, disp);
}

void st7789_pixel(int16_t x, int16_t y, uint16_t color, const st7789_t* disp) {
    st7789_cur(x, y, disp);
    st7789_cmd(0x2c, disp);
    st7789_dat16(color, disp);

    st7789_def(disp);
}

#ifdef ST7789_USE_GFX_DRV
void st7789_drv_init(gfx_drv_t* drv, st7789_t* disp) {
    drv->w = ST7789_W;
    drv->h = ST7789_H;
    drv->disp = (void*)disp;

    drv->win = st7789_win;
    drv->cur = st7789_cur;

    drv->pix = st7789_pixel;
    drv->wrt = st7789_wrt;
    drv->dat8 = st7789_dat;
    drv->dat16 = st7789_dat16;
}
#endif

#endif