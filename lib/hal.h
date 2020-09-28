#ifndef _TINY_HAL_H_
#define _TINY_HAL_H_

#include <stm32f1xx.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdarg.h>


// misc
static inline void hal_delay(uint32_t ms) {
    if(!(DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk)) DWT->CTRL = DWT_CTRL_CYCCNTENA_Msk;

    uint32_t pre = SystemCoreClock / 1000;
    uint32_t start = DWT->CYCCNT;
    while((DWT->CYCCNT - start) < ms * pre);
}

// usart
typedef enum {
    UART_RE = 1 << 0,
    UART_TE = 1 << 1
} uart_conf;

static inline void hal_uart1_init(uint32_t baud, uart_conf conf) {
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN | RCC_APB2ENR_USART1EN;

    // receive
    if(conf & UART_RE) {
        GPIOA->CRH &= (~GPIO_CRH_MODE10) & (~GPIO_CRH_CNF10_0);
        GPIOA->CRH |= GPIO_CRH_CNF10_1;
        GPIOA->ODR |= GPIO_ODR_ODR10;

        USART1->CR1 |= USART_CR1_RE;
    }

    // transmit
    if(conf & UART_TE) {
        GPIOA->CRH &= ~GPIO_CRH_CNF9_0;
        GPIOA->CRH |= GPIO_CRH_CNF9_1;
        GPIOA->CRH |= GPIO_CRH_MODE9; // 50Mhz

        USART1->CR1 |= USART_CR1_TE;
    }

    USART1->BRR = SystemCoreClock / baud;
    USART1->CR1 |= USART_CR1_UE;
}

static inline void hal_uart1_printc(char ch) {
    while(!(USART1->SR & USART_SR_TC));
    USART1->DR = ch;
}

static void hal_uart1_print(const char* s) {
    while(*s)
        hal_uart1_printc(*(s++));
}

static inline void hal_uart1_printf(const char* fmt, ...) {
    va_list args;
    static char buf[256] = {0};

    va_start(args, fmt);
    vsprintf(buf, fmt, args);
    va_end(args);

    hal_uart1_print(buf);
}

static inline char hal_uart1_readc() {
    while(!(USART1->SR & USART_SR_RXNE));
    return USART1->DR;
}

static inline void hal_uart1_read(char* buf, char sep, bool echo) {
    uint32_t pos = 0;
    char last = 0;

    while(last != sep) {
        last = hal_uart1_readc();
        if(echo) hal_uart1_printc(last);
        buf[pos++] = last;
    }
}

// spi
static inline void hal_spi1_init() {
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    // pa0(dc)
    GPIOA->CRL &= ~GPIO_CRL_CNF0;
    GPIOA->CRL |= GPIO_CRL_MODE0; // 50Mhz

    // pa1(cs)
    GPIOA->CRL &= ~GPIO_CRL_CNF1;
    GPIOA->CRL |= GPIO_CRL_MODE1; // 50Mhz

    // pa1(rst)
    GPIOA->CRL &= ~GPIO_CRL_CNF2;
    GPIOA->CRL |= GPIO_CRL_MODE2; // 50Mhz

    // pa5(scl)
    GPIOA->CRL &= ~GPIO_CRL_CNF5;
    GPIOA->CRL |= GPIO_CRL_CNF5_1;
    GPIOA->CRL |= GPIO_CRL_MODE5; // 50Mhz

    // pa7(mosi)
    GPIOA->CRL &= ~GPIO_CRL_CNF7;
    GPIOA->CRL |= GPIO_CRL_CNF7_1;
    GPIOA->CRL |= GPIO_CRL_MODE7; // 50Mhz

    // spi
    SPI1->CR1 |= SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE;
    SPI1->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_MSTR ; // master

    SPI1->CR1 |= SPI_CR1_SPE; // enable spi
}

#endif // _TINY_HAL