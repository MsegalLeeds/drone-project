#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdbool.h>

#include "attitude.h"
#include "imu.h"
#include "pid.h"
#include "simulated_imu.h"

static void test_roll_recovery(
    float initial_roll,
    float disturbance_rate,
    float gyro_bias,
    bool calibrate

)
{
    SimulatedDrone drone;
    AttitudeEstimator estimator;
    PID pid;

    const float dt = 0.01f;
    const float rad_to_deg = 57.29577951308232f;

    simulated_drone_init(&drone, initial_roll);

    float gyro_offset = 0.0f;

    if (calibrate) {
        enum { CALIBRATION_SAMPLES = 100 };
        ImuSample calibration_samples[CALIBRATION_SAMPLES];

        for (size_t i = 0; i < CALIBRATION_SAMPLES; i++) {
            calibration_samples[i] =
                imu_simulate(drone.roll_angle, drone.roll_rate);

            calibration_samples[i].gyro_x += gyro_bias;
        }

        gyro_offset = imu_calibrate_gyro_x(
            calibration_samples,
            CALIBRATION_SAMPLES
        );
    }

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
        sample.gyro_x -= gyro_offset;

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

        /* Check roll and estimation error throughout the final second. */
        if (i >= 399) {
            assert(fabsf(drone.roll_angle) <= 1.0f);
            assert(fabsf(estimated_roll - drone.roll_angle) <= 1.0f);
        }

        /* At the end, calibrated runs should be close to truly level. */
        if (calibrate && i == 499) {
            assert(fabsf(drone.roll_angle) <= 0.05f);
            assert(fabsf(estimated_roll - drone.roll_angle) <= 0.01f);
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
    /* Original recovery cases without calibration. */
    test_roll_recovery(20.0f, 0.0f, 0.0f, false);
    test_roll_recovery(-20.0f, 0.0f, 0.0f, false);
    test_roll_recovery(0.0f, 0.0f, 0.0f, false);

    /* Disturbance and bias tolerance without calibration. */
    test_roll_recovery(0.0f, 45.0f, 0.0f, false);
    test_roll_recovery(0.0f, -45.0f, 0.0f, false);
    test_roll_recovery(0.0f, 0.0f, 1.0f, false);
    test_roll_recovery(0.0f, 0.0f, -1.0f, false);

    /* Calibration with initial tilt, a shove and sensor bias. */
    test_roll_recovery(20.0f, 45.0f, 1.0f, true);
    test_roll_recovery(-20.0f, -45.0f, -1.0f, true);

    printf("All nine closed-loop scenarios passed.\n");
    return 0;
}