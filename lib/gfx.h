#ifndef _GFX_H_
#define _GFX_H_

#include <stdint.h>
#include <stddef.h>


////////////////////////////////
//         DEFINITION         //
////////////////////////////////

typedef struct _gfx_drv_t {
    size_t w, h;
    void* disp;

    void (*win)(int16_t x, int16_t y, uint8_t w, uint8_t h, const void* disp);
    void (*cur)(int16_t x, int16_t y, const void* disp);

    void (*pix)(int16_t x, int16_t y, uint16_t color, const void* disp);
    void (*wrt)(const void* disp);
    void (*dat8)(uint8_t val, const void* disp);
    void (*dat16)(uint16_t val, const void* disp);
} gfx_drv_t;

void gfx_fill(uint16_t color, const gfx_drv_t* drv);
void gfx_fill_img(const uint16_t* img, const gfx_drv_t* drv);

void gfx_pixel(int16_t x, int16_t y, uint16_t color, const gfx_drv_t* drv);
void gfx_rect(int16_t x, int16_t y, uint8_t w, uint8_t h, uint16_t color, const gfx_drv_t* drv);
void gfx_func(int16_t x0, int16_t y0, uint8_t w, uint8_t h, uint16_t (*color_f)(int16_t x, int16_t y), const gfx_drv_t* drv);
void gfx_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color, const gfx_drv_t* drv);

void gfx_glyph5x7(const uint8_t glyph[5], int16_t x0, int16_t y0, uint8_t size, uint16_t color, const gfx_drv_t* drv);
void gfx_char(char ch, int16_t x0, int16_t y0, uint8_t size, uint16_t color, const uint8_t* ascii_font, const gfx_drv_t* drv);
void gfx_text(const char* text, int16_t x, int16_t y, uint8_t size, uint16_t color, const uint8_t* ascii_font, const gfx_drv_t* drv);

void gfx_sprite(const uint16_t* img, int16_t x0, int16_t y0, uint8_t w, uint8_t h, const gfx_drv_t* drv);
void gfx_sprite_with_key(const uint16_t* img, int16_t x0, int16_t y0, uint8_t w, uint8_t h, uint16_t key, const gfx_drv_t* drv);
void gfx_sprite_with_back(const uint16_t* img, const uint16_t* back, int16_t x0, int16_t y0, uint8_t w, uint8_t h, uint16_t key, const gfx_drv_t* drv);

// misc
uint16_t gfx_rgb565(uint8_t r, uint8_t g, uint8_t b);

////////////////////////////////
//       IMPLEMENTATION       //
////////////////////////////////

void gfx_fill(uint16_t color, const gfx_drv_t* drv) {
    drv->win(0, 0, drv->w, drv->h, drv->disp);
    drv->wrt(drv->disp);
    for(size_t i = 0; i < drv->w * drv->h; i++) drv->dat16(color, drv->disp);
}

void gfx_fill_img(const uint16_t* img, const gfx_drv_t* drv) {
    drv->win(0, 0, drv->w, drv->h, drv->disp);
    drv->wrt(drv->disp);
    for(size_t i = 0; i < drv->w * drv->h; i++) drv->dat16(img[i], drv->disp);
}

void gfx_pixel(int16_t x, int16_t y, uint16_t color, const gfx_drv_t* drv) {
    drv->pix(x, y, color, drv->disp);
}

void gfx_rect(int16_t x, int16_t y, uint8_t w, uint8_t h, uint16_t color, const gfx_drv_t* drv) {
    drv->win(x, y, w, h, drv->disp);
    drv->wrt(drv->disp);
    for(size_t i = 0; i < w * h; i++) drv->dat16(color, drv->disp);
}

void gfx_func(int16_t x0, int16_t y0, uint8_t w, uint8_t h, uint16_t (*color_f)(int16_t x, int16_t y), const gfx_drv_t* drv) {
    drv->win(x0, y0, w, h, drv->disp);
    drv->wrt(drv->disp);

    for(uint8_t x = 0; x < w; x++) {
        for(uint8_t y = 0; y < h; y++)
            drv->dat16(color_f(x, y), drv->disp);
    }
}

void gfx_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color, const gfx_drv_t* drv) {
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
        drv->pix(x0, y0, color, drv->disp);
        return;
    }

    if(max_x >= max_y) {
        int16_t x = x0;
        int16_t y = y0;
        int16_t d = -max_x;

        max++;
        while(max--) {
            drv->pix(x, y, color, drv->disp);
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
            drv->pix(x, y, color, drv->disp);
            y += dy;
            d += max_x << 1;

            if(d > 0) {
                d -= max_y << 1;
                x += dx;
            }
        }
    }
}

void gfx_glyph5x7(const uint8_t glyph[5], int16_t x0, int16_t y0, uint8_t size, uint16_t color, const gfx_drv_t* drv) {
    if(size == 1) {
        for(uint8_t x = 0; x < 5; x++) {
            for(uint8_t y = 0; y < 8; y++) {
                if(glyph[x] & (1 << y))
                    gfx_pixel(x0 + x, y0 + (7 - y), color, drv);
            }
        }
    } else {
        for(uint8_t x = 0; x < 5; x++) {
            for(uint8_t y = 0; y < 8; y++) {
                if(glyph[x] & (1 << y))
                    gfx_rect(x0 + size * x, y0 + size * (7 - y), size, size, color, drv);
            }
        }
    }
}

void gfx_char(char ch, int16_t x0, int16_t y0, uint8_t size, uint16_t color, const uint8_t* ascii_font, const gfx_drv_t* drv) {
    gfx_glyph5x7(ascii_font + 5 * (uint16_t)ch, x0, y0, size, color, drv);
}

void gfx_text(const char* text, int16_t x, int16_t y, uint8_t size, uint16_t color, const uint8_t* ascii_font, const gfx_drv_t* drv) {
    uint8_t i = 0;
    uint8_t step = (size * 6);

    while(text[i]) {
        gfx_char(text[i], x + i * step, y, size, color, ascii_font, drv);
        i++;
    }
}

void gfx_sprite(const uint16_t* img, int16_t x0, int16_t y0, uint8_t w, uint8_t h, const gfx_drv_t* drv) {
    if(x0 >= drv->w || (x0 + w) <= 0 || y0 >= drv->h || (y0 + h) <= 0) return;

    drv->win(x0, y0, w, h, drv->disp);
    drv->wrt(drv->disp);
    for(uint16_t i = 0; i < w * h; i++) drv->dat16(img[i], drv->disp);
}

void gfx_sprite_with_key(const uint16_t* img, int16_t x0, int16_t y0, uint8_t w, uint8_t h, uint16_t key, const gfx_drv_t* drv) {
    if(x0 >= drv->w || (x0 + w) <= 0 || y0 >= drv->h || (y0 + h) <= 0) return;

    uint8_t max_x = (x0 + w) <= drv->w ? w : (drv->w - x0);
    uint8_t max_y = (y0 + h) <= drv->h ? h : (drv->h - y0);

    max_x = (x0 + w) <= drv->w ? w : (drv->w - x0);
    max_y = (y0 + h) <= drv->h ? h : (drv->h - y0);

    for(uint8_t x = 0; x < max_x; x++) {
        for(uint8_t y = 0; y < max_y; y++) {
            uint16_t color = img[y + x * h];
            if(color != key)
                gfx_pixel(x0 + x, y0 + y, color, drv);
        }
    }
}

void gfx_sprite_with_back(const uint16_t* img, const uint16_t* back, int16_t x0, int16_t y0, uint8_t w, uint8_t h, uint16_t key, const gfx_drv_t* drv) {
    if(x0 >= drv->w || (x0 + w) <= 0 || y0 >= drv->h || (y0 + h) <= 0) return;
 
    uint8_t max_x = (x0 + w) <= drv->w ? w : (drv->w - x0);
    uint8_t max_y = (y0 + h) <= drv->h ? h : (drv->h - y0);

    drv->win(x0, y0, max_x, max_y, drv->disp);
    drv->wrt(drv->disp);

    for(uint8_t x = 0; x < max_x; x++) {
        for(uint8_t y = 0; y < max_y; y++) {
            uint16_t color = img[y + h * x];
            if(color != key) drv->dat16(color, drv->disp);
            else drv->dat16(back[y0 + y + drv->h * (x0 + x)], drv->disp);
        }
    }
}

// misc
uint16_t gfx_rgb565(uint8_t r, uint8_t g, uint8_t b) {
    return (((uint32_t)r & 0xf8) << 8) + (((uint32_t)g & 0xfc) << 3) + ((uint32_t)b >> 3);
}

#endif