#include "bench_i2c.h"
#include "bench_sensors.h"
#include "bmp3.h"
#include <stdio.h>

void barometer_measure(char *report, size_t size) {
    if (!bench_i2c_idle()) {
        snprintf(report, size, "BAROMETER,BUS_LOW\n");
        return;
    }

    uint8_t id = 0, address = 0x77;
    if (bench_i2c_read(BMP3_REG_CHIP_ID, &id, 1, &address)) {
        address = 0x76;
        if (bench_i2c_read(BMP3_REG_CHIP_ID, &id, 1, &address)) {
            bool scl, sda;
            bench_check_pullups(&scl, &sda);
            snprintf(report, size, "BAROMETER,NO_ACK,weak_pulldown_B8=%u,B9=%u\n",
                     (unsigned)scl, (unsigned)sda);
            return;
        }
    }
    if (id != BMP3_CHIP_ID && id != BMP390_CHIP_ID) {
        snprintf(report, size, "BAROMETER,UNKNOWN_ID,0x%02X,0x%02X\n", address, id);
        return;
    }
    struct bmp3_dev dev = {
        .intf = BMP3_I2C_INTF, .intf_ptr = &address,
        .read = bench_i2c_read, .write = bench_i2c_write, .delay_us = bench_wait_us
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
        bench_wait_us(5000, NULL);
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

// LSM6DSO WHO_AM_I is read-only and available after power-up.
void imu_identify(char *report, size_t size) {
    if (!bench_i2c_idle()) {
        snprintf(report, size, "IMU,BUS_LOW\n");
        return;
    }
    const uint8_t addresses[] = {0x6B, 0x6A};
    unsigned last_id = 0, last_address = 0;
    for (unsigned i = 0; i < sizeof addresses; ++i) {
        uint8_t address = addresses[i], id = 0;
        if (bench_i2c_read(0x0F, &id, 1, &address) != 0) continue;
        if (id == 0x6C) {
            snprintf(report, size, "IMU,LSM6DSO,0x%02X,0x6C\n", (unsigned)address);
            return;
        }
        last_id = id;
        last_address = address;
    }
    if (last_address)
        snprintf(report, size, "IMU,UNKNOWN_ID,0x%02X,0x%02X\n", last_address, last_id);
    else
        snprintf(report, size, "IMU,NO_ACK\n");
}
