#include "bench_i2c.h"
#include "stm32h7xx.h"
#include <stddef.h>

// I2C1 pins PB8 SCL and PB9 SDA, used as open-drain GPIO for the bench test.
#define SCL (1u << 8)
#define SDA (1u << 9)
static void delay(void) {
    uint32_t until = DWT->CYCCNT + 640;
    while ((int32_t)(DWT->CYCCNT - until) < 0) {}
}
static void high(uint32_t pin) { GPIOB->BSRR = pin; delay(); }
static void low(uint32_t pin) { GPIOB->BSRR = pin << 16; delay(); }
static bool sda(void) { return (GPIOB->IDR & SDA) != 0; }
static void start(void) { high(SDA); high(SCL); low(SDA); low(SCL); }
static void stop(void) { low(SDA); high(SCL); high(SDA); }
static bool put(uint8_t value) {
    for (unsigned bit = 0; bit < 8; ++bit) {
        if (value & 0x80) high(SDA); else low(SDA);
        high(SCL); low(SCL); value <<= 1;
    }
    high(SDA); high(SCL);
    bool ack = !sda();
    low(SCL);
    return ack;
}
static uint8_t get(bool acknowledge) {
    uint8_t value = 0;
    high(SDA);
    for (unsigned bit = 0; bit < 8; ++bit) {
        high(SCL); value = (uint8_t)((value << 1) | sda()); low(SCL);
    }
    if (acknowledge) low(SDA); else high(SDA);
    high(SCL); low(SCL); high(SDA);
    return value;
}
void bench_i2c_init(void) {
    RCC->AHB4ENR |= RCC_AHB4ENR_GPIOBEN;
    (void)RCC->AHB4ENR;
    GPIOB->OTYPER |= SCL | SDA;
    GPIOB->PUPDR = (GPIOB->PUPDR & ~((3u << 16) | (3u << 18))) |
                   (1u << 16) | (1u << 18);
    GPIOB->BSRR = SCL | SDA;
    GPIOB->MODER = (GPIOB->MODER & ~((3u << 16) | (3u << 18))) |
                   (1u << 16) | (1u << 18);
}
int8_t bench_i2c_read(uint8_t reg, uint8_t *data, uint32_t length, void *context) {
    uint8_t address = *(uint8_t *)context;
    start();
    bool ack = put(address << 1) && put(reg);
    if (ack) {
        start();
        ack = put((address << 1) | 1);
    }
    if (ack) {
        for (uint32_t i = 0; i < length; ++i) data[i] = get(i + 1 < length);
    }
    stop();
    return ack ? 0 : -1;
}

int8_t bench_i2c_write(uint8_t reg, const uint8_t *data, uint32_t length, void *context) {
    uint8_t address = *(uint8_t *)context;
    start();
    bool ack = put(address << 1) && put(reg);
    for (uint32_t i = 0; ack && i < length; ++i) ack = put(data[i]);
    stop();
    return ack ? 0 : -1;
}

void bench_wait_us(uint32_t us, void *context) {
    (void)context;
    uint32_t start_time = DWT->CYCCNT;
    while (DWT->CYCCNT - start_time < us * (SystemCoreClock / 1000000u)) {}
}

bool bench_i2c_idle(void) {
    return (GPIOB->IDR & (SDA | SCL)) == (SDA | SCL);
}

void bench_check_pullups(bool *scl, bool *sda_high) {
    // With both outputs released, check for external pull-ups on the two lines.
    uint32_t saved_pulls = GPIOB->PUPDR;
    GPIOB->PUPDR = (saved_pulls & ~((3u << 16) | (3u << 18))) |
                   (2u << 16) | (2u << 18);
    bench_wait_us(1000, NULL);
    uint32_t levels = GPIOB->IDR;
    GPIOB->PUPDR = saved_pulls;
    delay();
    stop();
    *scl = (levels & SCL) != 0;
    *sda_high = (levels & SDA) != 0;
}
