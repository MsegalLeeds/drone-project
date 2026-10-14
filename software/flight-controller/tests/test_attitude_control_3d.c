#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "flight_controller.h"
#include "imu.h"
#include "simulated_imu.h"

#define RAD_TO_DEG 57.29577951308232f

static void test_combined_attitude_and_yaw_rate_control(void)
{
    const float dt = 0.01f;

    SimulatedDrone drone;

    simulated_drone_init_attitude_3d(
        &drone,
        20.0f,
        -15.0f,
        0.0f
    );

    ImuSample sample =
        imu_simulate_attitude_3d(
            drone.roll_angle,
            drone.pitch_angle,
            drone.roll_rate,
            drone.pitch_rate,
            drone.yaw_rate
        );

    float initial_roll =
        atan2f(
            sample.accel_y,
            sample.accel_z
        ) * RAD_TO_DEG;

    float initial_pitch =
        atan2f(
            -sample.accel_x,
            sqrtf(
                sample.accel_y * sample.accel_y +
                sample.accel_z * sample.accel_z
            )
        ) * RAD_TO_DEG;

    FlightController controller;

    flight_controller_init_3d(
        &controller,
        initial_roll,
        initial_pitch,
        0.0f,
        0.0f,
        0.0f
    );

    FlightControllerOutput output = {0};

    for (int i = 0; i < 1200; i++) {

        /*
         * Request +45 deg/s yaw between
         * t = 1 s and t = 7 s.
         */
        float target_yaw_rate =
            (i >= 100 && i < 700)
                ? 45.0f
                : 0.0f;

        sample =
            imu_simulate_attitude_3d(
                drone.roll_angle,
                drone.pitch_angle,
                drone.roll_rate,
                drone.pitch_rate,
                drone.yaw_rate
            );

        output =
            flight_controller_update_3d(
                &controller,
                &sample,
                0.0f,
                0.0f,
                target_yaw_rate,
                0.5f,
                dt
            );

        simulated_drone_step_motors(
            &drone,
            output.motors,
            dt
        );

        assert(isfinite(drone.roll_angle));
        assert(isfinite(drone.pitch_angle));
        assert(isfinite(drone.yaw_rate));

        /*
         * Near the end of the commanded spin,
         * yaw rate should track approximately
         * 45 deg/s.
         */
        if (i >= 600 && i < 700) {
            assert(
                fabsf(
                    drone.yaw_rate - 45.0f
                ) <= 1.0f
            );
        }

        /*
         * Roll and pitch should remain level
         * while yawing.
         */
        if (i >= 500) {
            assert(
                fabsf(drone.roll_angle)
                <= 1.0f
            );

            assert(
                fabsf(drone.pitch_angle)
                <= 1.0f
            );
        }
    }

    /*
     * After the yaw command returns to zero,
     * rotation should stop.
     */
    assert(
        fabsf(drone.yaw_rate)
        <= 0.1f
    );

    assert(
        fabsf(drone.roll_angle)
        <= 0.05f
    );

    assert(
        fabsf(drone.pitch_angle)
        <= 0.05f
    );

    printf(
        "3D attitude/yaw-rate control passed.\n"
    );
}

int main(void)
{
    test_combined_attitude_and_yaw_rate_control();

    return 0;
}