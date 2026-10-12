#include <assert.h>
#include <stdio.h>
#include <math.h>

#include "simulated_imu.h"
#include "imu.h"

static int nearly_equal(float a, float b)
{
    return fabsf(a - b) < 0.01f;
}

int main(void)
{
    SimulatedDrone drone;

    /* Test 1: initialization */
    simulated_drone_init(&drone, 20.0f);

    assert(drone.roll_angle == 20.0f);
    assert(drone.roll_rate == 0.0f);

    /* Test 2: no control input means no movement initially */
    simulated_drone_init(&drone, 0.0f);

    simulated_drone_step(&drone, 0.0f, 0.1f);

    assert(drone.roll_angle == 0.0f);
    assert(drone.roll_rate == 0.0f);

    /* Test 3: positive control input produces positive rotation */
    simulated_drone_init(&drone, 0.0f);

    simulated_drone_step(&drone, 10.0f, 0.1f);

    assert(drone.roll_rate > 0.0f);
    assert(drone.roll_angle > 0.0f);

    /* Test 4: level drone reads gravity in m/s^2 on Z */
    simulated_drone_init(&drone, 0.0f);

    ImuSample imu = imu_simulate(drone.roll_angle, drone.roll_rate);

    assert(nearly_equal(imu.accel_x, 0.0f));
    assert(nearly_equal(imu.accel_y, 0.0f));
    assert(nearly_equal(imu.accel_z, 9.81f));
    assert(nearly_equal(imu.gyro_x, 0.0f));


    /* Test 5: 90 degree roll rotates gravity onto Y */
    simulated_drone_init(&drone, 90.0f);

    imu = imu_simulate(drone.roll_angle, drone.roll_rate);

    assert(nearly_equal(imu.accel_y, 9.81f));
    assert(nearly_equal(imu.accel_z, 0.0f));

    /* Test 6: negative roll and rate keep their signs at the sensor boundary. */
    simulated_drone_init(&drone, -30.0f);
    imu = imu_simulate(drone.roll_angle, drone.roll_rate);
    assert(nearly_equal(imu.accel_y, -4.905f));
    assert(nearly_equal(imu.accel_z, 8.496f));

    simulated_drone_step(&drone, -10.0f, 0.1f);
    imu = imu_simulate(drone.roll_angle, drone.roll_rate);
    assert(nearly_equal(imu.gyro_x, -1.0f));
    assert(nearly_equal(imu.gyro_y, 0.0f));
    assert(nearly_equal(imu.gyro_z, 0.0f));

    printf("All simulation tests passed.\n");

    return 0;
}
