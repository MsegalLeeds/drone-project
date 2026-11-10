// Run the firmware sensor commands and Bosch compensation against a register map.
#include "bench_i2c.h"
#include "bench_sensors.h"
#include "bmp3.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint8_t registers[256];
static uint8_t device_address;
static int failed_register;
static bool bus_idle, data_ready, forced;
static unsigned reads, writes;
static char report[128];

static void device(uint8_t address) {
    memset(registers, 0, sizeof registers);
    device_address = address;
    failed_register = -1;
    bus_idle = true;
    data_ready = true;
    forced = false;
    reads = writes = 0;
}

bool bench_i2c_idle(void) { return bus_idle; }
void bench_check_pullups(bool *scl, bool *sda) { *scl = true; *sda = false; }
void bench_wait_us(uint32_t delay, void *address) { (void)delay; (void)address; }

int8_t bench_i2c_read(uint8_t reg, uint8_t *out, uint32_t len, void *address) {
    ++reads;
    if (*(uint8_t *)address != device_address || reg == failed_register) return -1;
    assert(reg + len <= sizeof registers);
    memcpy(out, registers + reg, len);
    return 0;
}

int8_t bench_i2c_write(uint8_t reg, const uint8_t *in, uint32_t len, void *address) {
    ++writes;
    assert(*(uint8_t *)address == device_address && len == 1);
    registers[reg] = *in;
    if (reg == BMP3_REG_PWR_CTRL && (*in & 0x33) == 0x13) {
        forced = true;  // One conversion with both temperature and pressure enabled.
        if (data_ready) registers[BMP3_REG_SENS_STATUS] |= 0x60;
    }
    return 0;
}

static void expect_baro(const char *expected) {
    barometer_measure(report, sizeof report);
    if (strcmp(report, expected)) fprintf(stderr, "Expected %sReceived %s", expected, report);
    assert(strcmp(report, expected) == 0);
}

static void expect_imu(const char *expected) {
    imu_identify(report, sizeof report);
    assert(strcmp(report, expected) == 0 && writes == 0);
}

static void bmp_fixture(uint8_t address, uint8_t id) {
    device(address);
    registers[BMP3_REG_CHIP_ID] = id;
    registers[BMP3_REG_SENS_STATUS] = BMP3_CMD_RDY;
    // T2=21504 and raw T=2^20 give 21 C. P5=12500 and raw P=0 give 100000 Pa.
    registers[BMP3_REG_CALIB_DATA + 3] = 0x54;
    registers[BMP3_REG_DATA + 5] = 0x10;
    registers[BMP3_REG_CALIB_DATA + 11] = 0xd4;
    registers[BMP3_REG_CALIB_DATA + 12] = 0x30;
}

int main(void) {
    bmp_fixture(0x76, BMP3_CHIP_ID);
    expect_baro("BAROMETER,BMP388,0x76,1000.00 hPa,21.00 C\n");
    assert(forced);
    bmp_fixture(0x77, BMP390_CHIP_ID);
    expect_baro("BAROMETER,BMP390,0x77,1000.00 hPa,21.00 C\n");
    assert(forced);

    device(0);
    expect_baro("BAROMETER,NO_ACK,weak_pulldown_B8=1,B9=0\n");
    expect_imu("IMU,NO_ACK\n");
    device(0x77);
    registers[BMP3_REG_CHIP_ID] = 0xff;
    expect_baro("BAROMETER,UNKNOWN_ID,0x77,0xFF\n");
    assert(writes == 0);

    bmp_fixture(0x76, BMP3_CHIP_ID);
    data_ready = false;
    expect_baro("BAROMETER,NOT_READY\n");
    assert(forced);
    bmp_fixture(0x76, BMP3_CHIP_ID);
    failed_register = BMP3_REG_DATA;
    barometer_measure(report, sizeof report);
    assert(strncmp(report, "BAROMETER,DRIVER_ERROR,", strlen("BAROMETER,DRIVER_ERROR,")) == 0);

    device(0x6b);
    registers[0x0f] = 0x6c;
    expect_imu("IMU,LSM6DSO,0x6B,0x6C\n");
    device(0x6a);
    registers[0x0f] = 0x6c;
    expect_imu("IMU,LSM6DSO,0x6A,0x6C\n");
    registers[0x0f] = 0x42;
    expect_imu("IMU,UNKNOWN_ID,0x6A,0x42\n");

    device(0);
    bus_idle = false;
    expect_baro("BAROMETER,BUS_LOW\n");
    expect_imu("IMU,BUS_LOW\n");
    assert(reads == 0 && writes == 0);
    puts("Passed barometer compensation, conversion, IMU identification and I2C fault cases.");
    return 0;
}
