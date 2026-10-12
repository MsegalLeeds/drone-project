#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "attitude.h"
#include "imu.h"
#include "pid.h"
#include "simulated_imu.h"

static void test_roll_recovery(
    float initial_roll,
    float disturbance_rate,
    float gyro_bias
)
{
    SimulatedDrone drone;
    AttitudeEstimator estimator;
    PID pid;

    const float dt = 0.01f;
    const float rad_to_deg = 57.29577951308232f;

    simulated_drone_init(&drone, initial_roll);

    ImuSample sample =
        imu_simulate(drone.roll_angle, drone.roll_rate);

    attitude_init(
        &estimator,
        atan2f(sample.accel_y, sample.accel_z) * rad_to_deg
    );

    pid_init(&pid, 5.0f, 0.0f, 2.0f);
    pid_set_limits(&pid, -100.0f, 100.0f);

    for (int i = 0; i < 500; i++) {
        /* Apply the shove at t = 1 s, before reading the sensors. */
        if (i == 100) {
            drone.roll_rate += disturbance_rate;
        }

        sample = imu_simulate(drone.roll_angle, drone.roll_rate);

        /* Bias affects the measurement, not the drone's actual motion. */
        sample.gyro_x += gyro_bias;

        float estimated_roll = attitude_update(
            &estimator,
            sample.accel_y,
            sample.accel_z,
            sample.gyro_x,
            dt
        );

        float control = pid_update(
            &pid,
            0.0f,
            estimated_roll,
            dt
        );

        assert(isfinite(estimated_roll));
        assert(isfinite(control));
        assert(control >= -100.0f);
        assert(control <= 100.0f);

        simulated_drone_step(&drone, control, dt);

        assert(isfinite(drone.roll_angle));
        assert(isfinite(drone.roll_rate));

        /* At t = 1 s, apply a sudden change in roll rate, in degrees/s. */
        if (i == 100) {
            drone.roll_rate += disturbance_rate;
        }

        /* Remain within one degree throughout the final second. */
        if (i >= 399) {
            assert(fabsf(drone.roll_angle) <= 1.0f);
        }
    }

    printf(
    "Passed: roll %.1f deg, shove %.1f deg/s, bias %.1f deg/s.\n",
    initial_roll,
    disturbance_rate,
    gyro_bias
);
}

int main(void)
{
    /* Original recovery cases. */
    test_roll_recovery(20.0f, 0.0f, 0.0f);
    test_roll_recovery(-20.0f, 0.0f, 0.0f);
    test_roll_recovery(0.0f, 0.0f, 0.0f);

    /* Sudden rotational disturbances. */
    test_roll_recovery(0.0f, 45.0f, 0.0f);
    test_roll_recovery(0.0f, -45.0f, 0.0f);

    /* Constant gyroscope bias in either direction. */
    test_roll_recovery(0.0f, 0.0f, 1.0f);
    test_roll_recovery(0.0f, 0.0f, -1.0f);

    printf("All closed-loop tests passed.\n");
    return 0;
}