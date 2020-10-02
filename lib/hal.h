#ifndef _TINY_HAL_H_
#define _TINY_HAL_H_

#include <stm32f1xx.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdarg.h>


// misc
static inline void hal_delay(uint32_t ms) {
    if(!(DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk)) DWT->CTRL = DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT = 0;

    uint32_t pre = SystemCoreClock / 1000;
    while(DWT->CYCCNT< ms * pre);
}

// gpio
typedef enum {
    HAL_GPIO_IN,
    HAL_GPIO_OUT,
    HAL_GPIO_AIN,
    HAL_GPIO_AOUT
} hal_gpio_type;

typedef enum {
    HAL_GPIO_INMODE,
    HAL_GPIO_10MHz,
    HAL_GPIO_2MHz,
    HAL_GPIO_50MHz
} hal_gpio_mode;

typedef enum {
    HAL_GPIO_LOW = 0,
    HAL_GPIO_HIGH,
    HAL_GPIO_UNDEF
} hal_gpio_val;

typedef struct {
    hal_gpio_type type;
    hal_gpio_mode mode;
} hal_gpio_cfg;

static inline hal_gpio_cfg hal_gpio_cfg_new(hal_gpio_type type, hal_gpio_mode mode) {
    return (hal_gpio_cfg) {
        .type = type,
        .mode = mode
    };
}

typedef enum {
    HAL_GPIOA = RCC_APB2ENR_IOPAEN,
    HAL_GPIOB = RCC_APB2ENR_IOPBEN,
    HAL_GPIOC = RCC_APB2ENR_IOPCEN
} hal_gpio_port;

static inline void hal_gpio_en(hal_gpio_port port) {
    RCC->APB2ENR |= port;
}

static inline void hal_gpio_dis(hal_gpio_port port) {
    RCC->APB2ENR &= ~port;
}

GPIO_TypeDef* _hal_get_native_port(hal_gpio_port port) {
    switch(port) {
        case HAL_GPIOA:
            return GPIOA;
        case HAL_GPIOB:
            return GPIOB;
        case HAL_GPIOC:
            return GPIOC;
        default:
            // unreachable
            return NULL;
    }
}

static inline bool hal_gpio_setup(hal_gpio_port port, uint8_t pin, hal_gpio_cfg cfg) {
    if(pin > 15) return false;

    GPIO_TypeDef* _port = _hal_get_native_port(port);
    volatile uint32_t* CR = pin < 8 ? &_port->CRL : &_port->CRH;
    uint32_t mode_offs = pin < 8 ? (pin << 2) : ((pin - 8) << 2);
    uint32_t type_offs = mode_offs + 2;

    if(cfg.type == HAL_GPIO_OUT || cfg.type == HAL_GPIO_AOUT) {
        *CR &= ~(1 << type_offs);

        if(cfg.type == HAL_GPIO_OUT) *CR &= ~(1 << (type_offs + 1));
        else *CR |= 1 << (type_offs + 1);

        switch (cfg.mode) {
        case HAL_GPIO_10MHz:
            *CR &= ~(1 << mode_offs);
            *CR |= 1 << (mode_offs + 1);
            return true;
        case HAL_GPIO_2MHz:
            *CR |= 1 << mode_offs;
            *CR &= ~(1 << (mode_offs + 1));
            return true;
        case HAL_GPIO_50MHz:
            *CR |= 3 << mode_offs;
            return true;
        default:
            return false;
        }
    } else if(cfg.type == HAL_GPIO_IN || cfg.type == HAL_GPIO_AIN) {
        *CR &= ~(1 << type_offs);

        if(cfg.type == HAL_GPIO_IN) *CR |= 1 << (type_offs + 1);
        else *CR &= ~(1 << (type_offs + 1));

        *CR &= ~(3 << mode_offs);
    }

    return false;
}

static inline bool hal_gpio_w(hal_gpio_port port, uint8_t pin, hal_gpio_val val) {
    if(pin > 15) return false;

    GPIO_TypeDef* _port = _hal_get_native_port(port);

    if(val == HAL_GPIO_HIGH) _port->ODR |= (1 << pin);
    else if(val == HAL_GPIO_LOW) _port->ODR &= ~(1 << pin);
    else return false;

    return true;
}

static inline hal_gpio_val hal_gpio_r(hal_gpio_port port, uint8_t pin) {
    if(pin > 15) return HAL_GPIO_UNDEF;

    GPIO_TypeDef* _port = _hal_get_native_port(port);
    volatile uint32_t* CR = pin < 8 ? &_port->CRL : &_port->CRH;

    uint32_t mode_offs = pin < 8 ? (pin << 2) : ((pin - 8) << 2);
    volatile uint32_t* val = (*CR) & (2 << mode_offs) ? &_port->ODR : &_port->IDR;

    if((*val) & (1 << pin)) return HAL_GPIO_HIGH;
    return HAL_GPIO_LOW;
}

static inline bool hal_gpio_inv(hal_gpio_port port, uint8_t pin) {
    if(pin > 15) return false;

    GPIO_TypeDef* _port = _hal_get_native_port(port);
    _port->ODR ^= (1 << pin);

    return true;
}

// usart
typedef enum {
    HAL_UART_RE = 1 << 0,
    HAL_UART_TE = 1 << 1
} hal_uart_conf;

// typedef struct {
//     hal_gpio 
// } hal_uart;


static inline void hal_uart1_init(uint32_t baud, hal_uart_conf conf) {
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN | RCC_APB2ENR_USART1EN;

    // receive
    if(conf & HAL_UART_RE) {
        GPIOA->CRH &= (~GPIO_CRH_MODE10) & (~GPIO_CRH_CNF10_0);
        GPIOA->CRH |= GPIO_CRH_CNF10_1;
        GPIOA->ODR |= GPIO_ODR_ODR10;

        USART1->CR1 |= USART_CR1_RE;
    }

    // transmit
    if(conf & HAL_UART_TE) {
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