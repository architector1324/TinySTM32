#ifndef _ST7735S_H_
#define _ST7735S_H_

#include <stm32f1xx.h>
#include "hal.h"


////////////////////////////////
//         DEFINITION         //
////////////////////////////////
typedef struct {
    hal_spi spi;
    hal_gpio_pin dc;
    hal_gpio_pin cs;
    hal_gpio_pin rst;
    uint8_t w;
    uint8_t h;
} st7735s;

uint16_t misc_rgb565(uint8_t r, uint8_t g, uint8_t b);

void st7735s_cmd(uint8_t cmd, const st7735s* disp);
void st7735s_dat(uint8_t dat, const st7735s* disp);
void st7735s_dat16(uint16_t dat, const st7735s* disp);

void st7735s_setWindow(int16_t x0, int16_t y0, uint8_t w, uint8_t h, const st7735s* disp);
void st7735s_setCursor(int16_t x, int16_t y, const st7735s* disp);

static inline void st7735s_default(const st7735s* disp);
void st7735s_init(const st7735s* disp);

void st7735s_fill(uint16_t color, const st7735s* disp);
void st7735s_fillImage(const uint16_t* img, const st7735s* disp);

void st7735s_drawPixel(int16_t x, int16_t y, uint16_t color, const st7735s* disp);
void st7735s_drawFunc(int16_t x0, int16_t y0, uint8_t w, uint8_t h, uint16_t (*color_f)(int16_t x, int16_t y), const st7735s* disp);
void st7735s_drawRect(int16_t x0, int16_t y0, uint8_t w, uint8_t h, uint16_t color, const st7735s* disp);
void st7735s_drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color, const st7735s* disp);

void st7735s_drawSprite(const uint16_t* img, int16_t x0, int16_t y0, uint8_t w, uint8_t h, const st7735s* disp);
void st7735s_drawSpriteWithKey(const uint16_t* img, int16_t x0, int16_t y0, uint8_t w, uint8_t h, uint16_t key, const st7735s* disp);
void st7735s_drawSpriteWithBack(const uint16_t* img, const uint16_t* back, int16_t x0, int16_t y0, uint8_t w, uint8_t h, uint16_t key, const st7735s* disp);

void st7735s_drawFont5x7(const uint8_t* buf, int16_t x0, int16_t y0, uint8_t size, uint16_t color, const st7735s* disp);
void st7735s_drawChar(char ch, int16_t x0, int16_t y0, uint8_t size, uint16_t color, const uint8_t* ascii_font, const st7735s* disp);
void st7735s_drawText(const char* text, int16_t x, int16_t y, uint8_t size, uint16_t color, const uint8_t* ascii_font, const st7735s* disp);


////////////////////////////////
//       IMPLEMENTATION       //
////////////////////////////////

inline uint16_t misc_rgb565(uint8_t r, uint8_t g, uint8_t b) {
    return (((uint32_t)r & 0xf8) << 8) + (((uint32_t)g & 0xfc) << 3) + ((uint32_t)b >> 3);
}

void st7735s_cmd(uint8_t cmd, const st7735s* disp) {
    hal_gpio_w(disp->dc.port, disp->dc.pin, HAL_GPIO_LOW);
    hal_gpio_w(disp->cs.port, disp->cs.pin, HAL_GPIO_LOW);

    hal_spi_w(cmd, disp->spi);

    hal_gpio_w(disp->cs.port, disp->cs.pin, HAL_GPIO_HIGH);
}

void st7735s_dat(uint8_t dat, const st7735s* disp) {
    hal_gpio_w(disp->dc.port, disp->dc.pin, HAL_GPIO_HIGH);
    hal_gpio_w(disp->cs.port, disp->cs.pin, HAL_GPIO_LOW);

    hal_spi_w(dat, disp->spi);

    hal_gpio_w(disp->cs.port, disp->cs.pin, HAL_GPIO_HIGH);
}

void st7735s_dat16(uint16_t dat, const st7735s* disp){
    hal_gpio_w(disp->dc.port, disp->dc.pin, HAL_GPIO_HIGH);
    hal_gpio_w(disp->cs.port, disp->cs.pin, HAL_GPIO_LOW);

	hal_spi_w16(dat, disp->spi);

    hal_gpio_w(disp->cs.port, disp->cs.pin, HAL_GPIO_HIGH);
}

void st7735s_setWindow(int16_t x0, int16_t y0, uint8_t w, uint8_t h, const st7735s* disp) {
    st7735s_cmd(0x2a, disp);
    st7735s_dat16(y0 + 26, disp);
    st7735s_dat16(h + y0 - 1 + 26, disp);

    st7735s_cmd(0x2b, disp);
    st7735s_dat16(x0 + 1, disp);
    st7735s_dat16(w + x0 - 1 + 1, disp);
}

void st7735s_setCursor(int16_t x, int16_t y, const st7735s* disp) {
    st7735s_setWindow(x, y, 1, 1, disp);
}

static inline void st7735s_default(const st7735s* disp) {
    st7735s_setWindow(0, 0, disp->w, disp->h, disp);
}

void st7735s_init(const st7735s* disp) {
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

    st7735s_default(disp);
    st7735s_cmd(0x21, disp); // inverse colors

    st7735s_cmd(0x26, disp); // gamma
    st7735s_dat(0x2, disp);

    st7735s_cmd(0x3a, disp); // rgb565 color mode
    st7735s_dat(0x05, disp);

    st7735s_cmd(0x29, disp); // on
    hal_delay(100);
}


void st7735s_fill(uint16_t color, const st7735s* disp) {
    st7735s_cmd(0x2c, disp);
    for(uint16_t i = 0; i < disp->h * disp->w; i++)
        st7735s_dat16(color, disp);
}

void st7735s_fillImage(const uint16_t* img, const st7735s* disp) {
    st7735s_cmd(0x2c, disp);
    for(uint16_t i = 0; i < disp->h * disp->w; i++) st7735s_dat16(img[i], disp);
}

void st7735s_drawPixel(int16_t x, int16_t y, uint16_t color, const st7735s* disp) {
    st7735s_setCursor(x, y, disp);
    st7735s_cmd(0x2c, disp);
    st7735s_dat16(color, disp);

    st7735s_default(disp);
}

void st7735s_drawFunc(int16_t x0, int16_t y0, uint8_t w, uint8_t h, uint16_t (*color_f)(int16_t x, int16_t y), const st7735s* disp) {
    st7735s_setWindow(x0, y0, w, h, disp);
    st7735s_cmd(0x2c, disp);

    for(uint8_t x = 0; x < w; x++) {
        for(uint8_t y = 0; y < h; y++) st7735s_dat16(color_f(x, y), disp);
    }
    st7735s_default(disp);
}

void st7735s_drawRect(int16_t x0, int16_t y0, uint8_t w, uint8_t h, uint16_t color, const st7735s* disp) {
    uint16_t size = w * h;
    st7735s_setWindow(x0, y0, w, h, disp);

    st7735s_cmd(0x2c, disp);
    for(uint16_t i = 0; i < size; i++) st7735s_dat16(color, disp);

    st7735s_default(disp);
}

void st7735s_drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color, const st7735s* disp) {
    // Bresenhams line algorithm
    int16_t max_x = x1 - x0;
    int16_t dx = max_x >= 0 ? 1 : -1;
    max_x = max_x > 0 ? max_x : -max_x;

    int16_t max_y = y1 - y0;
    int16_t dy = max_y >= 0 ? 1 : -1;
    max_y = max_y > 0 ? max_y : -max_y;

    int16_t max = max_x < max_y ? max_y : max_x;

    // draw pixel
    if(max == 0) {
        st7735s_drawPixel(x0, y0, color, disp);
        return;
    }

    if(max_x >= max_y) {
        int16_t x = x0;
        int16_t y = y0;
        int16_t d = -max_x;

        max++;
        while(max--) {
            st7735s_drawPixel(x, y, color, disp);
            x += dx;
            d += max_y << 1;

            if(d > 0) {
                d -= max_x << 1;
                y += dy;
            }
        }
    } else {
        int16_t x = x0;
        int16_t y = y0;
        int16_t d = -max_y;

        max++;
        while(max--) {
            st7735s_drawPixel(x, y, color, disp);
            y += dy;
            d += max_x << 1;

            if(d > 0) {
                d -= max_y << 1;
                x += dx;
            }
        }
    }
}

void st7735s_drawSprite(const uint16_t* img, int16_t x0, int16_t y0, uint8_t w, uint8_t h, const st7735s* disp) {
    if(x0 >= disp->w || (x0 + w) <= 0 || y0 >= disp->h || (y0 + h) <= 0) return;

    uint16_t size = w * h;
    st7735s_setWindow(x0, y0, w, h, disp);

    st7735s_cmd(0x2c, disp);
    for(uint16_t i = 0; i < size; i++) st7735s_dat16(img[i], disp);

    st7735s_default(disp);
}

void st7735s_drawSpriteWithKey(const uint16_t* img, int16_t x0, int16_t y0, uint8_t w, uint8_t h, uint16_t key, const st7735s* disp) {
    if(x0 >= disp->w || (x0 + w) <= 0 || y0 >= disp->h || (y0 + h) <= 0) return;

    uint8_t max_x = (x0 + w) <= disp->w ? w : (disp->w - x0);
    uint8_t max_y = (y0 + h) <= disp->h ? h : (disp->h - y0);

    max_x = (x0 + w) <= disp->w ? w : (disp->w - x0);
    max_y = (y0 + h) <= disp->h ? h : (disp->h - y0);

    for(uint8_t x = 0; x < max_x; x++) {
        for(uint8_t y = 0; y < max_y; y++) {
            uint16_t color = img[y + x * h];
            if(color != key)
                st7735s_drawPixel(x0 + x, y0 + y, color, disp);
        }
    }
}

void st7735s_drawSpriteWithBack(const uint16_t* img, const uint16_t* back, int16_t x0, int16_t y0, uint8_t w, uint8_t h, uint16_t key, const st7735s* disp) {
    if(x0 >= disp->w || (x0 + w) <= 0 || y0 >= disp->h || (y0 + h) <= 0) return;
 
    uint8_t max_x = (x0 + w) <= disp->w ? w : (disp->w - x0);
    uint8_t max_y = (y0 + h) <= disp->h ? h : (disp->h - y0);

    st7735s_setWindow(x0, y0, max_x, max_y, disp);
    st7735s_cmd(0x2c, disp);

    for(uint8_t x = 0; x < max_x; x++) {
        for(uint8_t y = 0; y < max_y; y++) {
            uint16_t color = img[y + h * x];
            if(color != key) st7735s_dat16(color, disp);
            else st7735s_dat16(back[y0 + y + disp->h * (x0 + x)], disp);
        }
    }
    st7735s_default(disp);
}

void st7735s_drawFont5x7(const uint8_t* buf, int16_t x0, int16_t y0, uint8_t size, uint16_t color, const st7735s* disp) {
    if(size == 1) {
        for(uint8_t x = 0; x < 5; x++) {
            for(uint8_t y = 0; y < 8; y++) {
                if(buf[x] & (1 << y))
                    st7735s_drawPixel(x0 + x, y0 + (7 - y), color, disp);
            }
        }
    } else {
        for(uint8_t x = 0; x < 5; x++) {
            for(uint8_t y = 0; y < 8; y++) {
                if(buf[x] & (1 << y))
                    st7735s_drawRect(x0 + size * x, y0 + size * (7 - y), size, size, color, disp);
            }
        }
    }
}

void st7735s_drawChar(char ch, int16_t x0, int16_t y0, uint8_t size, uint16_t color, const uint8_t* ascii_font, const st7735s* disp) {
    st7735s_drawFont5x7(ascii_font + 5 * (uint16_t)ch, x0, y0, size, color, disp);
}

void st7735s_drawText(const char* text, int16_t x, int16_t y, uint8_t size, uint16_t color, const uint8_t* ascii_font, const st7735s* disp) {
    uint8_t i = 0;
    uint8_t step = (size * 6);

    while(text[i]) {
        st7735s_drawChar(text[i], x + i * step, y, size, color, ascii_font, disp);
        i++;
    }
}

#endif
