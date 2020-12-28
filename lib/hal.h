#ifndef _TINY_HAL_H_
#define _TINY_HAL_H_

#include <stm32f1xx.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdarg.h>


////////////////////////////////
//          SETTINGS          //
////////////////////////////////
#define UART_BUFFER 256


////////////////////////////////
//         DEFINITION         //
////////////////////////////////

// misc
typedef void (*hal_irq_t)();
void hal_delay(uint32_t ms);

// gpio
typedef enum {
    HAL_GPIO_IN,
    HAL_GPIO_OUT,
    HAL_GPIO_AIN,
    HAL_GPIO_AOUT
} hal_gpio_type_t;

typedef enum {
    HAL_GPIO_INMODE,
    HAL_GPIO_10MHz,
    HAL_GPIO_2MHz,
    HAL_GPIO_50MHz
} hal_gpio_mode_t;

typedef enum {
    HAL_GPIO_LOW = 0,
    HAL_GPIO_HIGH,
    HAL_GPIO_UNDEF
} hal_gpio_val_t;

typedef struct {
    hal_gpio_type_t type;
    hal_gpio_mode_t mode;
} hal_gpio_cfg_t;

typedef enum {
    HAL_GPIOA = RCC_APB2ENR_IOPAEN,
    HAL_GPIOB = RCC_APB2ENR_IOPBEN,
    HAL_GPIOC = RCC_APB2ENR_IOPCEN
} hal_gpio_t;

typedef struct {
    hal_gpio_t port;
    uint8_t pin;
} hal_gpio_pin_t;

hal_gpio_pin_t hal_gpio_pin_new(hal_gpio_t port, uint8_t pin);
hal_gpio_cfg_t hal_gpio_cfg_new(hal_gpio_type_t type, hal_gpio_mode_t mode);

void hal_use_gpio(hal_gpio_t port);
void hal_not_use_gpio(hal_gpio_t port);
void hal_use_afio();
bool hal_gpio_setup(hal_gpio_t port, uint8_t pin, hal_gpio_cfg_t cfg);
bool hal_gpio_w(hal_gpio_t port, uint8_t pin, hal_gpio_val_t val);
hal_gpio_val_t hal_gpio_r(hal_gpio_t port, uint8_t pin);
bool hal_gpio_inv(hal_gpio_t port, uint8_t pin);

// uart
typedef enum {
    HAL_UART1 = RCC_APB2ENR_USART1EN,
    HAL_UART2 = RCC_APB1ENR_USART2EN,
    HAL_UART3 = RCC_APB1ENR_USART3EN
} hal_uart_t;

typedef struct {
    bool rx;
    bool tx;
    uint32_t baud;
} hal_uart_cfg_t;

hal_uart_cfg_t hal_uart_cfg_new(bool rx, bool tx, uint32_t baud);

void hal_use_uart(hal_uart_t uart);
void hal_uart_setup(hal_uart_t uart, hal_uart_cfg_t cfg);
void hal_uart_on(hal_uart_t uart);
void hal_uart_w(uint8_t val, hal_uart_t uart);
uint8_t hal_uart_r(hal_uart_t uart);

void hal_uart_printc(char ch, hal_uart_t uart);
void hal_uart_print(const char* s, hal_uart_t uart);
void hal_uart_printf(hal_uart_t uart, const char* fmt, ...);
char hal_uart_readc(hal_uart_t uart);
void hal_uart_read(char* buf, char sep, bool echo, hal_uart_t uart);

// spi
typedef enum {
    HAL_SPI1 = RCC_APB2ENR_SPI1EN,
    HAL_SPI2 = RCC_APB1ENR_SPI2EN
} hal_spi_t;

typedef enum {
    HAL_SPI_SLAVE = SPI_CR1_SSM,
    HAL_SPI_MASTER = SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_MSTR
} hal_spi_role_t;

typedef enum {
    HAL_SPI_SIMPLEX = SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE,
    HAL_SPI_HALF_DUPLEX = SPI_CR1_BIDIMODE,
    HAL_SPI_FULL_DUPLEX = 0
} hal_spi_mode_t;

typedef enum {
    HAL_SPI_BAUD2 = 0,
    HAL_SPI_BAUD4 = SPI_CR1_BR_0,
    HAL_SPI_BAUD8 = SPI_CR1_BR_1,
    HAL_SPI_BAUD16 = SPI_CR1_BR_0 | SPI_CR1_BR_1,
    HAL_SPI_BAUD32 = SPI_CR1_BR_2,
    HAL_SPI_BAUD64 = SPI_CR1_BR_0 | SPI_CR1_BR_2,
    HAL_SPI_BAUD128 = SPI_CR1_BR_1 | SPI_CR1_BR_2,
    HAL_SPI_BAUD256 = SPI_CR1_BR_0 | SPI_CR1_BR_1 | SPI_CR1_BR_2
} hal_spi_baud_factor_t;

typedef struct {
    hal_spi_role_t role;
    hal_spi_mode_t mode;
    hal_spi_baud_factor_t baud_factor;
} hal_spi_cfg_t;

hal_spi_cfg_t hal_spi_cfg_new(hal_spi_role_t role, hal_spi_mode_t mode, hal_spi_baud_factor_t factor);

void hal_use_spi(hal_spi_t spi);
void hal_spi_setup(hal_spi_t spi, hal_spi_cfg_t cfg);
void hal_spi_on(hal_spi_t spi);

void hal_spi_sel(const hal_gpio_pin_t* pin);
void hal_spi_desel(const hal_gpio_pin_t* pin);

void hal_spi_w(uint8_t val, hal_spi_t spi);
uint8_t hal_spi_r(hal_spi_t spi);
void hal_spi_w16(uint16_t val, hal_spi_t spi);
uint16_t hal_spi_r16(hal_spi_t spi);

// timer
typedef enum {
    HAL_TIMER1 = RCC_APB2ENR_TIM1EN,
    HAL_TIMER2 = RCC_APB1ENR_TIM2EN,
    HAL_TIMER3 = RCC_APB1ENR_TIM3EN,
    HAL_TIMER4 = RCC_APB1ENR_TIM4EN
} hal_timer_t;

typedef enum {
    HAL_NONE_PWM,
    HAL_PWM1 = TIM_CCER_CC1E,
    HAL_PWM2 = TIM_CCER_CC2E,
    HAL_PWM3 = TIM_CCER_CC3E,
    HAL_PWM4 = TIM_CCER_CC4E
} hal_timer_pwm_t;

typedef struct {
    uint32_t prescaler;
    uint32_t period;
    uint32_t duty_cycle;
    hal_timer_pwm_t pwm;
    hal_irq_t irq;
} hal_timer_cfg_t;

static hal_irq_t _hal_timer_irq[4][4] = {
    {NULL, NULL, NULL, NULL},
    {NULL, NULL, NULL, NULL},
    {NULL, NULL, NULL, NULL},
    {NULL, NULL, NULL, NULL}
};

hal_timer_cfg_t hal_timer_cfg_simple(uint32_t freq); // max freq = (SystemCoreClock / 1000) Hz
hal_timer_cfg_t hal_timer_cfg_new(uint32_t prescaler, uint32_t period, uint32_t duty_cycle);

void hal_use_timer(hal_timer_t tim);
void hal_timer_setup(hal_timer_t tim, hal_timer_cfg_t cfg);
void hal_timer_set_dc(hal_timer_t tim, uint16_t duty_cycle);
void hal_timer_on(hal_timer_t tim);
void hal_timer_off(hal_timer_t tim);

// adc
typedef enum {
    HAL_ADC1 = RCC_APB2ENR_ADC1EN,
    HAL_ADC2 = RCC_APB2ENR_ADC2EN
} hal_adc_t;

typedef enum {
    HAL_ADC_ONCE,
    HAL_ADC_CONT
} hal_adc_mode_t;

typedef struct {
    uint8_t ch;
    hal_adc_mode_t mode;
    hal_irq_t irq;
} hal_adc_cfg_t;

static hal_irq_t _hal_adc_irq[2][16] = {
    {
        NULL, NULL, NULL, NULL,
        NULL, NULL, NULL, NULL,
        NULL, NULL, NULL, NULL,
        NULL, NULL, NULL, NULL,
    },
    {
        NULL, NULL, NULL, NULL,
        NULL, NULL, NULL, NULL,
        NULL, NULL, NULL, NULL,
        NULL, NULL, NULL, NULL,
    }
};

hal_adc_cfg_t hal_adc_cfg_new(uint8_t ch, hal_adc_mode_t mode);

void hal_use_adc(hal_adc_t adc);
void hal_adc_setup(hal_adc_t adc, hal_adc_cfg_t cfg);
uint16_t hal_adc_r(hal_adc_t adc);
void hal_adc_on(hal_adc_t adc);
void hal_adc_go(hal_adc_t adc);

////////////////////////////////
//       IMPLEMENTATION       //
////////////////////////////////

// misc
void hal_delay(uint32_t ms) {
    if(!(DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk)) DWT->CTRL = DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT = 0;

    uint32_t pre = SystemCoreClock / 1000;
    while(DWT->CYCCNT< ms * pre);
}

// gpio
hal_gpio_pin_t hal_gpio_pin_new(hal_gpio_t port, uint8_t pin) {
    return (hal_gpio_pin_t) {
        .port = port,
        .pin = pin
    };
}

hal_gpio_cfg_t hal_gpio_cfg_new(hal_gpio_type_t type, hal_gpio_mode_t mode) {
    return (hal_gpio_cfg_t) {
        .type = type,
        .mode = mode
    };
}

void hal_use_gpio(hal_gpio_t port) {
    RCC->APB2ENR |= port;
}

void hal_use_afio() {
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
}

void hal_not_use_gpio(hal_gpio_t port) {
    RCC->APB2ENR &= ~port;
}

GPIO_TypeDef* _hal_get_cmsis_port(hal_gpio_t port) {
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

bool hal_gpio_setup(hal_gpio_t port, uint8_t pin, hal_gpio_cfg_t cfg) {
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

bool hal_gpio_w(hal_gpio_t port, uint8_t pin, hal_gpio_val_t val) {
    if(pin > 15) return false;

    GPIO_TypeDef* cmsis_port = _hal_get_cmsis_port(port);

    if(val == HAL_GPIO_HIGH) cmsis_port->ODR |= (1 << pin);
    else if(val == HAL_GPIO_LOW) cmsis_port->ODR &= ~(1 << pin);
    else return false;

    return true;
}

hal_gpio_val_t hal_gpio_r(hal_gpio_t port, uint8_t pin) {
    if(pin > 15) return HAL_GPIO_UNDEF;

    GPIO_TypeDef* cmsis_port = _hal_get_cmsis_port(port);
    volatile uint32_t* CR = pin < 8 ? &cmsis_port->CRL : &cmsis_port->CRH;

    uint32_t mode_offs = pin < 8 ? (pin << 2) : ((pin - 8) << 2);
    volatile uint32_t* val = (*CR) & (2 << mode_offs) ? &cmsis_port->ODR : &cmsis_port->IDR;

    if((*val) & (1 << pin)) return HAL_GPIO_HIGH;
    return HAL_GPIO_LOW;
}

bool hal_gpio_inv(hal_gpio_t port, uint8_t pin) {
    if(pin > 15) return false;

    GPIO_TypeDef* cmsis_port = _hal_get_cmsis_port(port);
    cmsis_port->ODR ^= (1 << pin);

    return true;
}

// uart
void hal_use_uart(hal_uart_t uart) {
    if(uart == HAL_UART1) RCC->APB2ENR |= uart;
    else RCC->APB1ENR |= uart;
}

hal_uart_cfg_t hal_uart_cfg_new(bool rx, bool tx, uint32_t baud) {
    return (hal_uart_cfg_t) {
        .rx = rx,
        .tx = tx,
        .baud = baud
    };
}

USART_TypeDef* _hal_get_cmsis_uart(hal_uart_t uart) {
    switch(uart) {
        case HAL_UART1:
            return USART1;
        case HAL_UART2:
            return USART2;
        case HAL_UART3:
            return USART3;
        default:
            // unreachable
            return NULL;
    }
}

void _hal_set_cmsis_uart_rx_gpio(hal_uart_t uart) {
    hal_gpio_cfg_t rx = hal_gpio_cfg_new(HAL_GPIO_IN, HAL_GPIO_INMODE);

    switch(uart) {
        case HAL_UART1:
            hal_gpio_setup(HAL_GPIOA, 10, rx);
            return;
        case HAL_UART2:
            hal_gpio_setup(HAL_GPIOA, 3, rx);
            return;
        case HAL_UART3:
            hal_gpio_setup(HAL_GPIOB, 11, rx);
            return;
        default:
            // unreachable
            return;
    }
}

void _hal_set_cmsis_uart_tx_gpio(hal_uart_t uart) {
    hal_gpio_cfg_t tx = hal_gpio_cfg_new(HAL_GPIO_AOUT, HAL_GPIO_50MHz);

    switch(uart) {
        case HAL_UART1:
            hal_gpio_setup(HAL_GPIOA, 9, tx);
            return;
        case HAL_UART2:
            hal_gpio_setup(HAL_GPIOA, 2, tx);
            return;
        case HAL_UART3:
            hal_gpio_setup(HAL_GPIOB, 10, tx);
            return;
        default:
            // unreachable
            return;
    }
}

void hal_uart_setup(hal_uart_t uart, hal_uart_cfg_t cfg) {
    USART_TypeDef* cmsis_uart = _hal_get_cmsis_uart(uart);

    cmsis_uart->CR1 = 0;

    if(cfg.rx) {
        _hal_set_cmsis_uart_rx_gpio(uart);
        cmsis_uart->CR1 |= USART_CR1_RE;
    }

    if(cfg.tx) {
        _hal_set_cmsis_uart_tx_gpio(uart);
        cmsis_uart->CR1 |= USART_CR1_TE;
    }

    cmsis_uart->BRR = SystemCoreClock / cfg.baud;
}

void hal_uart_on(hal_uart_t uart) {
    USART_TypeDef* cmsis_uart = _hal_get_cmsis_uart(uart);
    cmsis_uart->CR1 |= USART_CR1_UE;
}

void hal_uart_w(uint8_t val, hal_uart_t uart) {
    USART_TypeDef* cmsis_uart = _hal_get_cmsis_uart(uart);
    while(!(cmsis_uart->SR & USART_SR_TC));
    cmsis_uart->DR = val;
}

uint8_t hal_uart_r(hal_uart_t uart) {
    USART_TypeDef* cmsis_uart = _hal_get_cmsis_uart(uart);
    while(!(cmsis_uart->SR & USART_SR_RXNE));
    return cmsis_uart->DR;
}

void hal_uart_printc(char ch, hal_uart_t uart){
    hal_uart_w((uint8_t)ch, uart);
}

void hal_uart_print(const char* s, hal_uart_t uart){
    while(*s)
        hal_uart_printc(*(s++), uart);
}

void hal_uart_printf(hal_uart_t uart, const char* fmt, ...) {
    va_list args;
    static char buf[UART_BUFFER] = {0};

    va_start(args, fmt);
    vsprintf(buf, fmt, args);
    va_end(args);

    hal_uart_print(buf, uart);
}

char hal_uart_readc(hal_uart_t uart) {
    return (char)hal_uart_r(uart);
}

void hal_uart_read(char* buf, char sep, bool echo, hal_uart_t uart) {
    uint32_t pos = 0;
    char last = 0;

    while(last != sep) {
        last = hal_uart_readc(uart);
        if(echo) hal_uart_printc(last, uart);
        buf[pos++] = last;
    }
}

// spi
SPI_TypeDef* _hal_get_cmsis_spi(hal_spi_t spi) {
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

void _hal_set_cmsis_spi_gpio(hal_spi_t spi) {
    hal_gpio_cfg_t out_cfg = hal_gpio_cfg_new(HAL_GPIO_AOUT, HAL_GPIO_2MHz);
    hal_gpio_cfg_t in_cfg = hal_gpio_cfg_new(HAL_GPIO_AIN, HAL_GPIO_INMODE);

    switch(spi) {
        case HAL_SPI1:
            hal_gpio_setup(HAL_GPIOA, 5, out_cfg); // sck
            hal_gpio_setup(HAL_GPIOA, 6, in_cfg); // miso
            hal_gpio_setup(HAL_GPIOA, 7, out_cfg); // mosi

            GPIOA->ODR |= GPIO_ODR_ODR6;
            return;
        case HAL_SPI2:
            hal_gpio_setup(HAL_GPIOB, 13, out_cfg); // sck
            hal_gpio_setup(HAL_GPIOB, 14, in_cfg); // miso
            hal_gpio_setup(HAL_GPIOB, 15, out_cfg); // mosi
            return;
        default:
            // unreachable
            return;
    }
}

hal_spi_cfg_t hal_spi_cfg_new(hal_spi_role_t role, hal_spi_mode_t mode, hal_spi_baud_factor_t factor) {
    return (hal_spi_cfg_t) {
        .role = role,
        .mode = mode,
        .baud_factor = factor
    };
}

void hal_use_spi(hal_spi_t spi) {
    if(spi == HAL_SPI1) RCC->APB2ENR |= spi;
    else if(spi == HAL_SPI2) RCC->APB1ENR |= spi;
}

void hal_spi_setup(hal_spi_t spi, hal_spi_cfg_t cfg) {
    SPI_TypeDef* cmsis_spi = _hal_get_cmsis_spi(spi);

    _hal_set_cmsis_spi_gpio(spi);

    cmsis_spi->CR1 = 0;

    cmsis_spi->CR1 |= cfg.mode;
    cmsis_spi->CR1 |= cfg.role;

    cmsis_spi->CR1 |= SPI_CR1_CPHA | SPI_CR1_CPOL;
    cmsis_spi->CR1 |= cfg.baud_factor;
}

void hal_spi_on(hal_spi_t spi) {
    SPI_TypeDef* cmsis_spi = _hal_get_cmsis_spi(spi);
    cmsis_spi->CR1 |= SPI_CR1_SPE;
}

void hal_spi_sel(const hal_gpio_pin_t* pin) {
    hal_gpio_w(pin->port, pin->pin, HAL_GPIO_LOW);
}

void hal_spi_desel(const hal_gpio_pin_t* pin) {
    hal_gpio_w(pin->port, pin->pin, HAL_GPIO_HIGH);
}

void hal_spi_w(uint8_t val, hal_spi_t spi) {
    SPI_TypeDef* cmsis_spi = _hal_get_cmsis_spi(spi);

    while((cmsis_spi->SR & SPI_SR_TXE) != SPI_SR_TXE);
    cmsis_spi->DR = val;

    while((cmsis_spi->SR & SPI_SR_TXE) != SPI_SR_TXE);
    while((cmsis_spi->SR & SPI_SR_BSY) == SPI_SR_BSY);
}

void hal_spi_w16(uint16_t val, hal_spi_t spi) {
    SPI_TypeDef* cmsis_spi = _hal_get_cmsis_spi(spi);

    cmsis_spi->CR1 |= SPI_CR1_DFF;

    while((cmsis_spi->SR & SPI_SR_TXE) != SPI_SR_TXE);
    cmsis_spi->DR = val;

    while((cmsis_spi->SR & SPI_SR_TXE) != SPI_SR_TXE);
    while((cmsis_spi->SR & SPI_SR_BSY) == SPI_SR_BSY);

    cmsis_spi->CR1 &= ~SPI_CR1_DFF;
}

uint8_t hal_spi_r(hal_spi_t spi) {
    SPI_TypeDef* cmsis_spi = _hal_get_cmsis_spi(spi);

    while(!(cmsis_spi->SR & SPI_SR_RXNE));
    return cmsis_spi->DR;
}

uint16_t hal_spi_r16(hal_spi_t spi) {
    SPI_TypeDef* cmsis_spi = _hal_get_cmsis_spi(spi);

    cmsis_spi->CR1 |= SPI_CR1_DFF;
    while(!(cmsis_spi->SR & SPI_SR_RXNE));

    uint16_t res = cmsis_spi->DR;
    cmsis_spi->CR1 &= ~SPI_CR1_DFF;

    return res;
}


// timer
void hal_use_timer(hal_timer_t tim) {
    if(tim == HAL_TIMER1) RCC->APB2ENR |= tim;
    else RCC->APB1ENR |= tim;
}

TIM_TypeDef* _hal_get_cmsis_timer(hal_timer_t tim) {
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

hal_timer_cfg_t hal_timer_cfg_simple(uint32_t freq) {
    return (hal_timer_cfg_t) {
        .prescaler = (SystemCoreClock / 1000) / freq,
        .period = 1000,
        .duty_cycle = 1000,
        .pwm = HAL_NONE_PWM,
        .irq = NULL
    };
}

hal_timer_cfg_t hal_timer_cfg_new(uint32_t prescaler, uint32_t period, uint32_t duty_cycle) {
    return (hal_timer_cfg_t) {
        .prescaler = prescaler,
        .period = period,
        .duty_cycle = duty_cycle,
        .pwm = HAL_NONE_PWM,
        .irq = NULL
    };
}

void _hal_set_cmsis_timer_pwm(TIM_TypeDef* tim, hal_timer_pwm_t pwm) {
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

IRQn_Type _hal_get_cmsis_timer_irq(hal_timer_t tim) {
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

void _hal_set_cmsis_timer_irq(hal_timer_t tim, hal_irq_t hlr) {
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

void hal_timer_setup(hal_timer_t tim, hal_timer_cfg_t cfg) {
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

void hal_timer_set_dc(hal_timer_t tim, uint16_t duty_cycle) {
    TIM_TypeDef* cmsis_timer = _hal_get_cmsis_timer(tim);
    cmsis_timer->CCR2 = duty_cycle;
}

void hal_timer_on(hal_timer_t tim) {
    TIM_TypeDef* cmsis_timer = _hal_get_cmsis_timer(tim);
    cmsis_timer->CR1 |= TIM_CR1_CEN;
}

void hal_timer_off(hal_timer_t tim) {
    TIM_TypeDef* cmsis_timer = _hal_get_cmsis_timer(tim);
    cmsis_timer->CR1 &= ~TIM_CR1_CEN;
}

void TIM1_CC_IRQHandler() {
    TIM1->SR &= ~TIM_SR_UIF;

    if(_hal_timer_irq[0][0])
        _hal_timer_irq[0][0]();
}

void TIM2_IRQHandler() {
    TIM2->SR &= ~TIM_SR_UIF;

    if(_hal_timer_irq[1][0])
        _hal_timer_irq[1][0]();
}

void TIM3_IRQHandler() {
    TIM3->SR &= ~TIM_SR_UIF;

    if(_hal_timer_irq[2][0])
        _hal_timer_irq[2][0]();
}

void TIM4_IRQHandler() {
    TIM4->SR &= ~TIM_SR_UIF;

    if(_hal_timer_irq[3][0])
        _hal_timer_irq[3][0]();
}

// adc
void hal_use_adc(hal_adc_t adc) {
    RCC->APB2ENR |= adc;
}

ADC_TypeDef* _hal_get_cmsis_adc(hal_adc_t adc) {
    switch(adc) {
        case HAL_ADC1:
            return ADC1;
        case HAL_ADC2:
            return ADC2;
        default:
            // unreachable
            return NULL;
    }
}

void _hal_set_cmsis_adc_ch(ADC_TypeDef* adc, uint8_t ch) {
    size_t smp_pos = 3 * (ch % 10);

    // enable channel
    adc->SQR3 |= ch % 16;

    // sampling
    if(ch < 10) {
        adc->SMPR2 |= 0x7 << smp_pos; // sampling 239.5 cycles
    } else if(ch < 18) {
        adc->SMPR1 |= 0x7 << smp_pos;
    }
}

hal_adc_cfg_t hal_adc_cfg_new(uint8_t ch, hal_adc_mode_t mode) {
    return (hal_adc_cfg_t) {
        .ch = ch,
        .mode = mode
    };
}

void _hal_set_cmsis_adc_irq(hal_adc_t adc, hal_irq_t hlr) {
    switch(adc) {
        case HAL_ADC1:
            _hal_adc_irq[0][0] = hlr;
            return;
        case HAL_ADC2:
            _hal_adc_irq[1][0] = hlr;
            return;
        default:
            // unreachable
            return;
    }
}

void hal_adc_setup(hal_adc_t adc, hal_adc_cfg_t cfg) {
    ADC_TypeDef* cmsis_adc = _hal_get_cmsis_adc(adc);

    cmsis_adc->CR2 = 0;

    cmsis_adc->CR2 |= ADC_CR2_CAL; // calibration
    while(!(cmsis_adc->CR2 & ADC_CR2_CAL));

    if(cfg.mode == HAL_ADC_CONT)
        cmsis_adc->CR2 |= ADC_CR2_CONT;
    else
        cmsis_adc->CR2 &= ~ADC_CR2_CONT;

    cmsis_adc->CR2 |= ADC_CR2_EXTSEL; // conv by flag SWSTART
    cmsis_adc->CR2 |= ADC_CR2_EXTTRIG; // conv by EXTI

    _hal_set_cmsis_adc_ch(cmsis_adc, cfg.ch);

    if(cfg.irq != NULL) {
        cmsis_adc->CR1 |= ADC_CR1_EOCIE; // enable interrupt on EOC
        NVIC_EnableIRQ(ADC1_2_IRQn);
        _hal_set_cmsis_adc_irq(adc, cfg.irq);
    }
}

uint16_t hal_adc_r(hal_adc_t adc) {
    ADC_TypeDef* cmsis_adc = _hal_get_cmsis_adc(adc);
    while(!(cmsis_adc->SR & ADC_SR_EOC));
    return cmsis_adc->DR;
}

void hal_adc_on(hal_adc_t adc) {
    ADC_TypeDef* cmsis_adc = _hal_get_cmsis_adc(adc);
    cmsis_adc->CR2 |= ADC_CR2_ADON;
}

void hal_adc_go(hal_adc_t adc) {
    ADC_TypeDef* cmsis_adc = _hal_get_cmsis_adc(adc);
    cmsis_adc->CR2 |= ADC_CR2_SWSTART;
}

void ADC1_2_IRQHandler() {
    if(ADC1->SR & ADC_SR_EOC) {
        if(_hal_adc_irq[0][0])
            _hal_adc_irq[0][0]();
        ADC1->SR &= ~ADC_SR_EOC;
    } else if (ADC2->SR & ADC_SR_EOC) {
        if(_hal_adc_irq[1][0])
            _hal_adc_irq[1][0]();
        ADC2->SR &= ~ADC_SR_EOC;
    }
}

#endif // _TINY_HAL