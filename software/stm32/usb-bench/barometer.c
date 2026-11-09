#include "stm32h7xx.h"
#include "bmp3.h"
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

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
void barometer_pins_init(void) {
    RCC->AHB4ENR |= RCC_AHB4ENR_GPIOBEN;
    (void)RCC->AHB4ENR;
    GPIOB->OTYPER |= SCL | SDA;
    GPIOB->PUPDR = (GPIOB->PUPDR & ~((3u << 16) | (3u << 18))) |
                   (1u << 16) | (1u << 18);
    GPIOB->BSRR = SCL | SDA;
    GPIOB->MODER = (GPIOB->MODER & ~((3u << 16) | (3u << 18))) |
                   (1u << 16) | (1u << 18);
}
static int8_t read_register(uint8_t reg, uint8_t *data, uint32_t length, void *context) {
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

static int8_t write_register(uint8_t reg, const uint8_t *data, uint32_t length, void *context) {
    uint8_t address = *(uint8_t *)context;
    start();
    bool ack = put(address << 1) && put(reg);
    for (uint32_t i = 0; ack && i < length; ++i) ack = put(data[i]);
    stop();
    return ack ? 0 : -1;
}

static void wait_us(uint32_t us, void *context) {
    (void)context;
    uint32_t start_time = DWT->CYCCNT;
    while (DWT->CYCCNT - start_time < us * (SystemCoreClock / 1000000u)) {}
}

void barometer_measure(char *report, size_t size) {
    if ((GPIOB->IDR & (SDA | SCL)) != (SDA | SCL)) {
        snprintf(report, size, "BAROMETER,BUS_LOW\n");
        return;
    }

    uint8_t id = 0, address = 0x77;
    if (read_register(BMP3_REG_CHIP_ID, &id, 1, &address)) {
        address = 0x76;
        if (read_register(BMP3_REG_CHIP_ID, &id, 1, &address)) {
            // With both outputs released, check for external pull-ups on the two lines.
            uint32_t saved_pulls = GPIOB->PUPDR;
            GPIOB->PUPDR = (saved_pulls & ~((3u << 16) | (3u << 18))) |
                           (2u << 16) | (2u << 18);
            wait_us(1000, NULL);
            uint32_t levels = GPIOB->IDR;
            GPIOB->PUPDR = saved_pulls;
            delay();
            stop();
            snprintf(report, size, "BAROMETER,NO_ACK,weak_pulldown_B8=%u,B9=%u\n",
                     (unsigned)((levels & SCL) != 0), (unsigned)((levels & SDA) != 0));
            return;
        }
    }
    if (id != BMP3_CHIP_ID && id != BMP390_CHIP_ID) {
        snprintf(report, size, "BAROMETER,UNKNOWN_ID,0x%02X,0x%02X\n", address, id);
        return;
    }
    struct bmp3_dev dev = {
        .intf = BMP3_I2C_INTF, .intf_ptr = &address,
        .read = read_register, .write = write_register, .delay_us = wait_us
    };
    struct bmp3_settings settings = {
        .press_en = BMP3_ENABLE, .temp_en = BMP3_ENABLE, .op_mode = BMP3_MODE_FORCED
    };
    struct bmp3_status status = {0};
    struct bmp3_data data = {0};
    int rc = bmp3_init(&dev);
    if (rc == BMP3_OK)
        rc = bmp3_set_sensor_settings(BMP3_SEL_PRESS_EN | BMP3_SEL_TEMP_EN, &settings, &dev);
    if (rc == BMP3_OK) rc = bmp3_set_op_mode(&settings, &dev);
    for (unsigned i = 0; rc == BMP3_OK && i < 10; ++i) {
        wait_us(5000, NULL);
        rc = bmp3_get_status(&status, &dev);
        if (status.sensor.drdy_press && status.sensor.drdy_temp) break;
    }
    if (rc == BMP3_OK && !(status.sensor.drdy_press && status.sensor.drdy_temp)) {
        snprintf(report, size, "BAROMETER,NOT_READY\n");
        return;
    }
    if (rc == BMP3_OK) rc = bmp3_get_sensor_data(BMP3_PRESS_TEMP, &data, &dev);
    if (rc != BMP3_OK) {
        snprintf(report, size, "BAROMETER,DRIVER_ERROR,%d\n", rc);
        return;
    }
    snprintf(report, size, "BAROMETER,%s,0x%02X,%.2f hPa,%.2f C\n",
             id == BMP3_CHIP_ID ? "BMP388" : "BMP390", address, data.pressure / 100.0, data.temperature);
}
