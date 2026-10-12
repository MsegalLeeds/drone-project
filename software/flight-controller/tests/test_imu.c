#include "imu.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

#define EPSILON 0.01f

static int nearly_equal(float a, float b)
{
    return fabsf(a - b) < EPSILON;
}

static void test_level_drone(void)
{
    ImuSample sample = imu_simulate(0.0f, 0.0f);

    assert(nearly_equal(sample.gyro_x, 0.0f));
    assert(nearly_equal(sample.accel_y, 0.0f));
    assert(nearly_equal(sample.accel_z, 9.81f));
}

static void test_roll_angle(void)
{
    ImuSample sample = imu_simulate(30.0f, 0.0f);

    assert(nearly_equal(sample.accel_y, 4.905f));
    assert(nearly_equal(sample.accel_z, 8.495f));
}

static void test_roll_rate(void)
{
    ImuSample sample = imu_simulate(0.0f, 45.0f);

    assert(nearly_equal(sample.gyro_x, 45.0f));
}

int main(void)
{
    test_level_drone();
    test_roll_angle();
    test_roll_rate();

    printf("IMU tests passed\n");
    return 0;
}