#ifndef _TINY_HAL_H_
#define _TINY_HAL_H_

#include <stm32f1xx.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdarg.h>


// misc
static inline void hal_delay(uint32_t ms) {
    for(uint32_t i = 0; i < ms * 2000; i++) __NOP();
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

#endif // _TINY_HAL