#pragma once
#include <stdbool.h>
#include <stdint.h>

void bench_i2c_init(void);
bool bench_i2c_idle(void);
void bench_check_pullups(bool *scl, bool *sda);
int8_t bench_i2c_read(uint8_t reg, uint8_t *data, uint32_t length, void *address);
int8_t bench_i2c_write(uint8_t reg, const uint8_t *data, uint32_t length, void *address);
void bench_wait_us(uint32_t us, void *context);
