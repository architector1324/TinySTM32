#ifndef _TINY_HAL_H_
#define _TINY_HAL_H_

#include <stm32f1xx.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdarg.h>


////////////////////////////////
//         DEFINITION         //
////////////////////////////////

// misc
typedef void (*hal_irq)();
static inline void hal_delay(uint32_t ms);

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

typedef enum {
    HAL_GPIOA = RCC_APB2ENR_IOPAEN,
    HAL_GPIOB = RCC_APB2ENR_IOPBEN,
    HAL_GPIOC = RCC_APB2ENR_IOPCEN
} hal_gpio_port;

static inline hal_gpio_cfg hal_gpio_cfg_new(hal_gpio_type type, hal_gpio_mode mode);
static inline void hal_use_gpio(hal_gpio_port port);
static inline void hal_not_use_gpio(hal_gpio_port port);
static inline void hal_use_afio();
static inline bool hal_gpio_setup(hal_gpio_port port, uint8_t pin, hal_gpio_cfg cfg);
static inline bool hal_gpio_w(hal_gpio_port port, uint8_t pin, hal_gpio_val val);
static inline hal_gpio_val hal_gpio_r(hal_gpio_port port, uint8_t pin);
static inline bool hal_gpio_inv(hal_gpio_port port, uint8_t pin);

// uart
typedef enum {
    HAL_UART_RE = 1 << 0,
    HAL_UART_TE = 1 << 1
} hal_uart_conf;

// typedef struct {
//     hal_gpio 
// } hal_uart;

static inline void hal_uart1_init(uint32_t baud, hal_uart_conf conf);
static inline void hal_uart1_printc(char ch);
static void hal_uart1_print(const char* s);
static inline void hal_uart1_printf(const char* fmt, ...);
static inline char hal_uart1_readc();
static inline void hal_uart1_read(char* buf, char sep, bool echo);

// spi
typedef enum {
    HAL_SPI1 = RCC_APB2ENR_SPI1EN,
    HAL_SPI2 = RCC_APB1ENR_SPI2EN
} hal_spi;

static inline void hal_use_spi(hal_spi spi);
static inline void hal_spi_init(hal_spi spi);

// timer
typedef enum {
    HAL_TIMER1 = RCC_APB2ENR_TIM1EN,
    HAL_TIMER2 = RCC_APB1ENR_TIM2EN,
    HAL_TIMER3 = RCC_APB1ENR_TIM3EN,
    HAL_TIMER4 = RCC_APB1ENR_TIM4EN
} hal_timer;

typedef enum {
    HAL_NONE_PWM,
    HAL_PWM1 = TIM_CCER_CC1E,
    HAL_PWM2 = TIM_CCER_CC2E,
    HAL_PWM3 = TIM_CCER_CC3E,
    HAL_PWM4 = TIM_CCER_CC4E
} hal_timer_pwm;

typedef struct {
    uint32_t prescaler;
    uint32_t period;
    uint32_t duty_cycle;
    hal_timer_pwm pwm;
    hal_irq irq;
} hal_timer_cfg;

static hal_irq _hal_timer_irq[4][4] = {
    {NULL, NULL, NULL, NULL},
    {NULL, NULL, NULL, NULL},
    {NULL, NULL, NULL, NULL},
    {NULL, NULL, NULL, NULL}
};

static inline hal_timer_cfg hal_timer_cfg_simple(uint32_t freq); // max freq = (SystemCoreClock / 1000) Hz
static inline hal_timer_cfg hal_timer_cfg_new(uint32_t prescaler, uint32_t period, uint32_t duty_cycle);

static inline void hal_use_timer(hal_timer tim);
static inline void hal_timer_setup(hal_timer tim, hal_timer_cfg cfg);
static inline void hal_timer_start(hal_timer tim);
static inline void hal_timer_stop(hal_timer tim);

// adc
typedef enum {
    HAL_ADC1 = RCC_APB2ENR_ADC1EN,
    HAL_ADC2 = RCC_APB2ENR_ADC2EN
} hal_adc;

static inline void hal_use_adc(hal_adc adc);


////////////////////////////////
//       IMPLEMENTATION       //
////////////////////////////////

// misc
static inline void hal_delay(uint32_t ms) {
    if(!(DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk)) DWT->CTRL = DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT = 0;

    uint32_t pre = SystemCoreClock / 1000;
    while(DWT->CYCCNT< ms * pre);
}

// gpio
static inline hal_gpio_cfg hal_gpio_cfg_new(hal_gpio_type type, hal_gpio_mode mode) {
    return (hal_gpio_cfg) {
        .type = type,
        .mode = mode
    };
}

static inline void hal_use_gpio(hal_gpio_port port) {
    RCC->APB2ENR |= port;
}

static inline void hal_use_afio() {
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
}

static inline void hal_not_use_gpio(hal_gpio_port port) {
    RCC->APB2ENR &= ~port;
}

GPIO_TypeDef* _hal_get_cmsis_port(hal_gpio_port port) {
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

    GPIO_TypeDef* cmsis_port = _hal_get_cmsis_port(port);
    volatile uint32_t* CR = pin < 8 ? &cmsis_port->CRL : &cmsis_port->CRH;
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

    GPIO_TypeDef* cmsis_port = _hal_get_cmsis_port(port);

    if(val == HAL_GPIO_HIGH) cmsis_port->ODR |= (1 << pin);
    else if(val == HAL_GPIO_LOW) cmsis_port->ODR &= ~(1 << pin);
    else return false;

    return true;
}

static inline hal_gpio_val hal_gpio_r(hal_gpio_port port, uint8_t pin) {
    if(pin > 15) return HAL_GPIO_UNDEF;

    GPIO_TypeDef* cmsis_port = _hal_get_cmsis_port(port);
    volatile uint32_t* CR = pin < 8 ? &cmsis_port->CRL : &cmsis_port->CRH;

    uint32_t mode_offs = pin < 8 ? (pin << 2) : ((pin - 8) << 2);
    volatile uint32_t* val = (*CR) & (2 << mode_offs) ? &cmsis_port->ODR : &cmsis_port->IDR;

    if((*val) & (1 << pin)) return HAL_GPIO_HIGH;
    return HAL_GPIO_LOW;
}

static inline bool hal_gpio_inv(hal_gpio_port port, uint8_t pin) {
    if(pin > 15) return false;

    GPIO_TypeDef* cmsis_port = _hal_get_cmsis_port(port);
    cmsis_port->ODR ^= (1 << pin);

    return true;
}

// uart
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
SPI_TypeDef* _hal_get_cmsis_spi(hal_spi spi) {
    switch(spi) {
        case HAL_SPI1:
            return SPI1;
        case HAL_SPI2:
            return SPI2;
        default:
            // unreachable
            return NULL;
    }
}

static inline void hal_use_spi(hal_spi spi) {
    if(spi == HAL_SPI1) RCC->APB2ENR |= spi;
    else if(spi == HAL_SPI2) RCC->APB1ENR |= spi;
}

static inline void hal_spi_init(hal_spi spi) {
    SPI_TypeDef* cmsis_spi = _hal_get_cmsis_spi(spi);

    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    // pa5(scl)
    GPIOA->CRL &= ~GPIO_CRL_CNF5;
    GPIOA->CRL |= GPIO_CRL_CNF5_1;
    GPIOA->CRL |= GPIO_CRL_MODE5; // 50Mhz

    // pa7(mosi)
    GPIOA->CRL &= ~GPIO_CRL_CNF7;
    GPIOA->CRL |= GPIO_CRL_CNF7_1;
    GPIOA->CRL |= GPIO_CRL_MODE7; // 50Mhz

    // spi
    cmsis_spi->CR1 |= SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE;
    cmsis_spi->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_MSTR ; // master

    cmsis_spi->CR1 |= SPI_CR1_SPE; // enable spi
}

// timer
static inline void hal_use_timer(hal_timer tim) {
    if(tim == HAL_TIMER1) RCC->APB2ENR |= tim;
    else RCC->APB1ENR |= tim;
}

TIM_TypeDef* _hal_get_cmsis_timer(hal_timer tim) {
    switch(tim) {
        case HAL_TIMER1:
            return TIM1;
        case HAL_TIMER2:
            return TIM2;
        case HAL_TIMER3:
            return TIM3;
        case HAL_TIMER4:
            return TIM4;
        default:
            // unreachable
            return NULL;
    }
}

static inline hal_timer_cfg hal_timer_cfg_simple(uint32_t freq) {
    return (hal_timer_cfg) {
        .prescaler = (SystemCoreClock / 1000) / freq,
        .period = 1000,
        .duty_cycle = 1000,
        .pwm = HAL_NONE_PWM,
        .irq = NULL
    };
}

static inline hal_timer_cfg hal_timer_cfg_new(uint32_t prescaler, uint32_t period, uint32_t duty_cycle) {
    return (hal_timer_cfg) {
        .prescaler = prescaler,
        .period = period,
        .duty_cycle = duty_cycle,
        .pwm = HAL_NONE_PWM,
        .irq = NULL
    };
}

void _hal_set_cmsis_timer_pwm(TIM_TypeDef* tim, hal_timer_pwm pwm) {
    switch(pwm) {
            case HAL_PWM1:
                tim->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;
                break;
            case HAL_PWM2:
                tim->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;
                break;
            case HAL_PWM3:
                tim->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2;
                break;
            case HAL_PWM4:
                tim->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2;
                break;
            default:
                // unreachable
                return;
        }
}

IRQn_Type _hal_get_cmsis_timer_irq(hal_timer tim) {
    switch(tim) {
        case HAL_TIMER1:
            return TIM1_CC_IRQn;
        case HAL_TIMER2:
            return TIM2_IRQn;
        case HAL_TIMER3:
            return TIM3_IRQn;
        case HAL_TIMER4:
            return TIM4_IRQn;
        default:
            // unreachable
            return 0;
    }
}

void _hal_set_cmsis_timer_irq(hal_timer tim, hal_irq hlr) {
    switch(tim) {
        case HAL_TIMER1:
            _hal_timer_irq[0][0] = hlr;
            return;
        case HAL_TIMER2:
            _hal_timer_irq[1][0] = hlr;
            return;
        case HAL_TIMER3:
            _hal_timer_irq[2][0] = hlr;
            return;
        case HAL_TIMER4:
            _hal_timer_irq[3][0] = hlr;
            return;
        default:
            // unreachable
            return;
    }
}

static inline void hal_timer_setup(hal_timer tim, hal_timer_cfg cfg) {
    TIM_TypeDef* cmsis_timer = _hal_get_cmsis_timer(tim);

    cmsis_timer->PSC = cfg.prescaler - 1; // prescaler: freq = clock / (prescaler + 1)
    cmsis_timer->ARR = cfg.period; // period
    cmsis_timer->CCR2 = cfg.duty_cycle; // duty cycle

    if(cfg.pwm != HAL_NONE_PWM) {
        _hal_set_cmsis_timer_pwm(cmsis_timer, cfg.pwm);
        cmsis_timer->CCER |= cfg.pwm; // enable channel
    }

    if(cfg.irq != NULL) {
        cmsis_timer->DIER |= TIM_DIER_UIE;
        NVIC_EnableIRQ(_hal_get_cmsis_timer_irq(tim));
        _hal_set_cmsis_timer_irq(tim, cfg.irq);
    }
}

static inline void hal_timer_start(hal_timer tim) {
    TIM_TypeDef* cmsis_timer = _hal_get_cmsis_timer(tim);
    cmsis_timer->CR1 |= TIM_CR1_CEN;
}

static inline void hal_timer_stop(hal_timer tim) {
    TIM_TypeDef* cmsis_timer = _hal_get_cmsis_timer(tim);
    cmsis_timer->CR1 &= ~TIM_CR1_CEN;
}

void TIM1_CC_IRQHandler() {
    TIM1->SR &= ~TIM_SR_UIF;
    _hal_timer_irq[0][0]();
}

void TIM2_IRQHandler() {
    TIM2->SR &= ~TIM_SR_UIF;
    _hal_timer_irq[1][0]();
}

void TIM3_IRQHandler() {
    TIM3->SR &= ~TIM_SR_UIF;
    _hal_timer_irq[2][0]();
}

void TIM4_IRQHandler() {
    TIM4->SR &= ~TIM_SR_UIF;
    _hal_timer_irq[3][0]();
}

// adc
static inline void hal_use_adc(hal_adc adc) {
    RCC->APB2ENR |= adc;
}

#endif // _TINY_HAL