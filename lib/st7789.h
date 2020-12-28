#ifndef _ST7789_H_
#define _ST7789_H_

#include <stm32f1xx.h>
#include "hal.h"


////////////////////////////////
//         DEFINITION         //
////////////////////////////////
typedef struct {
    hal_spi_t spi;
    hal_gpio_pin_t dc;
    hal_gpio_pin_t rst;
    uint8_t w;
    uint8_t h;
} st7789_t;

void st7789_cmd(uint8_t cmd, const st7789_t* disp);
void st7789_dat(uint8_t dat, const st7789_t* disp);
void st7789_dat16(uint16_t dat, const st7789_t* disp);

void st7789_init(const st7789_t* disp);

void st7789_setWindow(int16_t x0, int16_t y0, uint8_t w, uint8_t h, const st7789_t* disp);
void st7789_setCursor(int16_t x, int16_t y, const st7789_t* disp);
static inline void st7789_default(const st7789_t* disp);

void st7789_fill(uint16_t color, const st7789_t* disp);
void st7789_drawPixel(int16_t x, int16_t y, uint16_t color, const st7789_t* disp);
void st7789_drawRect(int16_t x0, int16_t y0, uint8_t w, uint8_t h, uint16_t color, const st7789_t* disp);

void st7789_drawFont5x7(const uint8_t* buf, int16_t x0, int16_t y0, uint8_t size, uint16_t color, const st7789_t* disp);
void st7789_drawChar(char ch, int16_t x0, int16_t y0, uint8_t size, uint16_t color, const uint8_t* ascii_font, const st7789_t* disp);
void st7789_drawText(const char* text, int16_t x, int16_t y, uint8_t size, uint16_t color, const uint8_t* ascii_font, const st7789_t* disp);

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

void st7789_init(const st7789_t* disp) {
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

    st7789_default(disp);
    st7789_cmd(0x21, disp); // inverse colors

    st7789_cmd(0x26, disp); // gamma
    st7789_dat(0x2, disp);

    st7789_cmd(0x3a, disp); // rgb565 color mode
    st7789_dat(0x05, disp);

    st7789_cmd(0x29, disp); // on
    hal_delay(100);
}

void st7789_setWindow(int16_t x0, int16_t y0, uint8_t w, uint8_t h, const st7789_t* disp) {
    st7789_cmd(0x2a, disp);
    st7789_dat16(y0, disp);
    st7789_dat16(h + y0 - 1, disp);

    st7789_cmd(0x2b, disp);
    st7789_dat16(x0, disp);
    st7789_dat16(w + x0 - 1, disp);
}

void st7789_setCursor(int16_t x, int16_t y, const st7789_t* disp) {
    st7789_setWindow(x, y, 1, 1, disp);
}

static inline void st7789_default(const st7789_t* disp) {
    st7789_setWindow(0, 0, disp->w, disp->h, disp);
}

void st7789_fill(uint16_t color, const st7789_t* disp) {
    st7789_cmd(0x2c, disp);
    for(uint16_t i = 0; i < disp->h * disp->w; i++)
        st7789_dat16(color, disp);
}

void st7789_drawPixel(int16_t x, int16_t y, uint16_t color, const st7789_t* disp) {
    st7789_setCursor(x, y, disp);
    st7789_cmd(0x2c, disp);
    st7789_dat16(color, disp);

    st7789_default(disp);
}

void st7789_drawRect(int16_t x0, int16_t y0, uint8_t w, uint8_t h, uint16_t color, const st7789_t* disp) {
    uint16_t size = w * h;
    st7789_setWindow(x0, y0, w, h, disp);

    st7789_cmd(0x2c, disp);
    for(uint16_t i = 0; i < size; i++) st7789_dat16(color, disp);

    st7789_default(disp);
}

void st7789_drawFont5x7(const uint8_t* buf, int16_t x0, int16_t y0, uint8_t size, uint16_t color, const st7789_t* disp) {
    if(size == 1) {
        for(uint8_t x = 0; x < 5; x++) {
            for(uint8_t y = 0; y < 8; y++) {
                if(buf[x] & (1 << y))
                    st7789_drawPixel(x0 + x, y0 + (7 - y), color, disp);
            }
        }
    } else {
        for(uint8_t x = 0; x < 5; x++) {
            for(uint8_t y = 0; y < 8; y++) {
                if(buf[x] & (1 << y))
                    st7789_drawRect(x0 + size * x, y0 + size * (7 - y), size, size, color, disp);
            }
        }
    }
}

void st7789_drawChar(char ch, int16_t x0, int16_t y0, uint8_t size, uint16_t color, const uint8_t* ascii_font, const st7789_t* disp) {
    st7789_drawFont5x7(ascii_font + 5 * (uint16_t)ch, x0, y0, size, color, disp);
}

void st7789_drawText(const char* text, int16_t x, int16_t y, uint8_t size, uint16_t color, const uint8_t* ascii_font, const st7789_t* disp) {
    uint8_t i = 0;
    uint8_t step = (size * 6);

    while(text[i]) {
        st7789_drawChar(text[i], x + i * step, y, size, color, ascii_font, disp);
        i++;
    }
}

#endif