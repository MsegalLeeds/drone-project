#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "flight_controller.h"
#include "imu.h"
#include "simulated_imu.h"

#define RAD_TO_DEG 57.29577951308232f

static void test_recovery(
    float initial_roll,
    float initial_pitch,
    float roll_disturbance,
    float pitch_disturbance
)
{
    SimulatedDrone drone;

    simulated_drone_init_attitude(
        &drone,
        initial_roll,
        initial_pitch
    );

    const float dt = 0.01f;

    ImuSample sample =
        imu_simulate_attitude(
            drone.roll_angle,
            drone.pitch_angle,
            drone.roll_rate,
            drone.pitch_rate
        );

    float initial_roll_estimate =
        atan2f(
            sample.accel_y,
            sample.accel_z
        ) * RAD_TO_DEG;

    float initial_pitch_estimate =
        atan2f(
            -sample.accel_x,
            sqrtf(
                sample.accel_y *
                sample.accel_y +
                sample.accel_z *
                sample.accel_z
            )
        ) * RAD_TO_DEG;

    FlightController controller;

    flight_controller_init(
        &controller,
        initial_roll_estimate,
        initial_pitch_estimate,
        0.0f,
        0.0f
    );

    FlightControllerOutput output = {0};

    for (int i = 0; i < 800; i++) {

        /*
         * Apply an angular-rate disturbance
         * after one second.
         */
        if (i == 100) {
            drone.roll_rate +=
                roll_disturbance;

            drone.pitch_rate +=
                pitch_disturbance;
        }

        sample =
            imu_simulate_attitude(
                drone.roll_angle,
                drone.pitch_angle,
                drone.roll_rate,
                drone.pitch_rate
            );

        output =
            flight_controller_update(
                &controller,
                &sample,
                0.0f,
                0.0f,
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

        assert(isfinite(output.estimated_roll));
        assert(isfinite(output.estimated_pitch));

        assert(isfinite(output.roll_control));
        assert(isfinite(output.pitch_control));

        /*
         * During the final second both axes
         * should be effectively level.
         */
        if (i >= 699) {
            assert(
                fabsf(
                    drone.roll_angle
                ) <= 1.0f
            );

            assert(
                fabsf(
                    drone.pitch_angle
                ) <= 1.0f
            );

            assert(
                fabsf(
                    output.estimated_roll -
                    drone.roll_angle
                ) <= 1.0f
            );

            assert(
                fabsf(
                    output.estimated_pitch -
                    drone.pitch_angle
                ) <= 1.0f
            );
        }
    }

    /*
     * By the end we expect considerably
     * tighter convergence.
     */
    assert(
        fabsf(
            drone.roll_angle
        ) <= 0.05f
    );

    assert(
        fabsf(
            drone.pitch_angle
        ) <= 0.05f
    );

    printf(
        "Passed: roll %.1f, pitch %.1f, "
        "roll shove %.1f, pitch shove %.1f\n",
        initial_roll,
        initial_pitch,
        roll_disturbance,
        pitch_disturbance
    );
}

int main(void)
{
    test_recovery(
        20.0f,
        0.0f,
        0.0f,
        0.0f
    );

    test_recovery(
        0.0f,
        20.0f,
        0.0f,
        0.0f
    );

    test_recovery(
        20.0f,
        -15.0f,
        45.0f,
        -30.0f
    );

    test_recovery(
        -20.0f,
        15.0f,
        -45.0f,
        30.0f
    );

    printf(
        "All 2D attitude-control scenarios passed.\n"
    );

    return 0;
}