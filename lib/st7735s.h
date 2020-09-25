#ifndef _ST7735S_H_
#define _ST7735S_H_

#include <stm32f1xx.h>


#define ST7735S_DC_L GPIOA->ODR &= ~GPIO_ODR_ODR0
#define ST7735S_DC_H GPIOA->ODR |= GPIO_ODR_ODR0

#define ST7735S_CS_L GPIOA->ODR &= ~GPIO_ODR_ODR1
#define ST7735S_CS_H GPIOA->ODR |= GPIO_ODR_ODR1

#define ST7735S_RST_L GPIOA->ODR &= ~GPIO_ODR_ODR2
#define ST7735S_RST_H GPIOA->ODR |= GPIO_ODR_ODR2


////////////////////////////////
//         DEFINITION         //
////////////////////////////////
static inline void delay(uint32_t ms);

void st7735s_cmd(uint8_t cmd);
void st7735s_dat(uint8_t dat);
void st7735s_dat16(uint16_t dat);

void st7735s_setWindow(uint8_t x0, uint8_t y0, uint8_t w, uint8_t h);
void st7735s_setCursor(uint8_t x, uint8_t y);

static inline void st7735s_default();
void st7735s_init();

void st7735s_fill(uint16_t color);
void st7735s_fillImage(const uint16_t* img);

void st7735s_drawPixel(uint8_t x, uint8_t y, uint16_t color);
void st7735s_fillFunc(uint8_t x0, uint8_t y0, uint8_t w, uint8_t h, uint16_t (*color_f)(uint8_t x, uint8_t y));
void st7735s_drawRect(uint8_t x0, uint8_t y0, uint8_t w, uint8_t h, uint16_t color);
void st7735s_drawSprite(const uint16_t* img, uint8_t x0, uint8_t y0, uint8_t w, uint8_t h);

void st7735s_drawFont5x7(const uint8_t* buf, uint8_t x0, uint8_t y0, uint8_t size, uint16_t color);
void st7735s_drawChar(char ch, uint8_t x0, uint8_t y0, uint8_t size, uint16_t color, const uint8_t* ascii_font);
void st7735s_drawText(const char* text, uint8_t x, uint8_t y, uint8_t size, uint16_t color, const uint8_t* ascii_font);


////////////////////////////////
//       IMPLEMENTATION       //
////////////////////////////////
static inline void delay(uint32_t ms) {
    for(uint32_t i = 0; i < ms * 2000; i++) __NOP();
}

void st7735s_cmd(uint8_t cmd) {
    ST7735S_DC_L;
    ST7735S_CS_L;

    while(!(SPI1->SR & SPI_SR_TXE));
    SPI1->DR = cmd;

    while(!(SPI1->SR & SPI_SR_TXE));
    while((SPI1->SR & SPI_SR_BSY));

    ST7735S_CS_H;
}

void st7735s_dat(uint8_t dat) {
    ST7735S_DC_H;
    ST7735S_CS_L;

    while(!(SPI1->SR & SPI_SR_TXE));
    SPI1->DR = dat;

    while(!(SPI1->SR & SPI_SR_TXE));
    while((SPI1->SR & SPI_SR_BSY));

    ST7735S_CS_H;
}

void st7735s_dat16(uint16_t dat){
	ST7735S_DC_H;
	ST7735S_CS_L;

	SPI1->CR1 |= SPI_CR1_DFF;

	while (!(SPI1->SR & SPI_SR_TXE));
	SPI1->DR = dat;

	while(!(SPI1->SR & SPI_SR_TXE));
	while((SPI1->SR & SPI_SR_BSY));

	ST7735S_CS_H;
    SPI1->CR1 &= ~SPI_CR1_DFF;
}

void st7735s_setWindow(uint8_t x0, uint8_t y0, uint8_t w, uint8_t h) {
    st7735s_cmd(0x2a);
    st7735s_dat16(y0 + 26);
    st7735s_dat16(h + y0 - 1 + 26);

    st7735s_cmd(0x2b);
    st7735s_dat16(x0 + 1);
    st7735s_dat16(w + x0 - 1 + 1);
}

void st7735s_setCursor(uint8_t x, uint8_t y) {
    st7735s_setWindow(x, y, 1, 1);
}

static inline void st7735s_default() {
    st7735s_setWindow(0, 0, 160, 80);
}

void st7735s_init() {
    // reset display
    ST7735S_RST_L;
    delay(10);

    ST7735S_RST_H;
    delay(10);

    st7735s_cmd(0x01); // software reset
    delay(120);

    st7735s_cmd(0x11); // sleep out
    delay(120);

    st7735s_cmd(0x36); // set orientation
    st7735s_dat(0x8);

    st7735s_default();
    st7735s_cmd(0x21); // inverse colors

    st7735s_cmd(0x26); // gamma
    st7735s_dat(0x2);

    st7735s_cmd(0x3a); // rgb565 color mode
    st7735s_dat(0x05);

    st7735s_cmd(0x29); // on
    delay(100);
}


void st7735s_fill(uint16_t color) {
    st7735s_cmd(0x2c);
    for(uint16_t i = 0; i < 80 * 160; i++)
        st7735s_dat16(color);
}

void st7735s_fillImage(const uint16_t* img) {
    st7735s_cmd(0x2c);
    for(uint16_t i = 0; i < 80 * 160; i++) st7735s_dat16(img[i]);
}

void st7735s_drawPixel(uint8_t x, uint8_t y, uint16_t color) {
    st7735s_setCursor(x, y);
    st7735s_cmd(0x2c);
    st7735s_dat16(color);

    st7735s_default();
}

void st7735s_fillFunc(uint8_t x0, uint8_t y0, uint8_t w, uint8_t h, uint16_t (*color_f)(uint8_t x, uint8_t y)) {
    st7735s_setWindow(x0, y0, w, h);
    st7735s_cmd(0x2c);

    for(uint8_t x = 0; x < w; x++) {
        for(uint8_t y = 0; y < h; y++) st7735s_dat16(color_f(x, y));
    }
    st7735s_default();
}

void st7735s_drawRect(uint8_t x0, uint8_t y0, uint8_t w, uint8_t h, uint16_t color) {
    uint16_t size = w * h;
    st7735s_setWindow(x0, y0, w, h);

    st7735s_cmd(0x2c);
    for(uint16_t i = 0; i < size; i++) st7735s_dat16(color);

    st7735s_default();
}

void st7735s_drawSprite(const uint16_t* img, uint8_t x0, uint8_t y0, uint8_t w, uint8_t h) {
    uint16_t size = w * h;
    st7735s_setWindow(x0, y0, w, h);

    st7735s_cmd(0x2c);
    for(uint16_t i = 0; i < size; i++) st7735s_dat16(img[i]);

    st7735s_default();
}

void st7735s_drawFont5x7(const uint8_t* buf, uint8_t x0, uint8_t y0, uint8_t size, uint16_t color) {
    if(size == 1) {
        for(uint8_t x = 0; x < 5; x++) {
            for(uint8_t y = 0; y < 8; y++) {
                if(buf[x] & (1 << y))
                    st7735s_drawPixel(x0 + x, y0 + (7 - y), color);
            }
        }
    } else {
        for(uint8_t x = 0; x < 5; x++) {
            for(uint8_t y = 0; y < 8; y++) {
                if(buf[x] & (1 << y))
                    st7735s_drawRect(x0 + size * x, y0 + size * (7 - y), size, size, color);
            }
        }
    }
}

void st7735s_drawChar(char ch, uint8_t x0, uint8_t y0, uint8_t size, uint16_t color, const uint8_t* ascii_font) {
    st7735s_drawFont5x7(ascii_font + 5 * (uint16_t)ch, x0, y0, size, color);
}

void st7735s_drawText(const char* text, uint8_t x, uint8_t y, uint8_t size, uint16_t color, const uint8_t* ascii_font) {
    uint8_t i = 0;
    uint8_t step = (size * 6);

    while(text[i])
        st7735s_drawChar(text[i], x + (i++) * step, y, size, color, ascii_font);
}

#endif
