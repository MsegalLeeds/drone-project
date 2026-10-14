#include "flight_controller.h"

#define ROLL_KP 5.0f
#define ROLL_KI 0.0f
#define ROLL_KD 2.0f

#define ROLL_OUTPUT_MIN -100.0f
#define ROLL_OUTPUT_MAX 100.0f

/*
 * The simulator currently expects the PID output
 * [-100, 100] to be converted to approximately
 * [-0.5, 0.5] for the mixer.
 *
 * Equivalent to:
 *
 *     control / 200.0f
 */
#define ROLL_TO_MIXER_SCALE (1.0f / 200.0f)

void flight_controller_init(
    FlightController *controller,
    float initial_roll_deg,
    float gyro_x_offset
)
{
    if (controller == 0) {
        return;
    }

    pid_init(
        &controller->roll_pid,
        ROLL_KP,
        ROLL_KI,
        ROLL_KD
    );

    pid_set_limits(
        &controller->roll_pid,
        ROLL_OUTPUT_MIN,
        ROLL_OUTPUT_MAX
    );

    attitude_init(
        &controller->attitude,
        initial_roll_deg
    );

    controller->gyro_x_offset =
        gyro_x_offset;
}

FlightControllerOutput flight_controller_update(
    FlightController *controller,
    const ImuSample *sample,
    float target_roll_deg,
    float throttle,
    float dt
)
{
    FlightControllerOutput output = {
        .estimated_roll = 0.0f,
        .roll_control = 0.0f,
        .motors = {
            .front_left = 0.0f,
            .front_right = 0.0f,
            .rear_right = 0.0f,
            .rear_left = 0.0f
        }
    };

    if (
        controller == 0 ||
        sample == 0 ||
        dt <= 0.0f
    ) {
        return output;
    }

    /*
     * Apply the calibrated gyro bias before
     * feeding the measurement into the estimator.
     */
    float corrected_gyro_x =
        sample->gyro_x -
        controller->gyro_x_offset;

    output.estimated_roll =
        attitude_update(
            &controller->attitude,
            sample->accel_y,
            sample->accel_z,
            corrected_gyro_x,
            dt
        );

    output.roll_control =
        pid_update(
            &controller->roll_pid,
            target_roll_deg,
            output.estimated_roll,
            dt
        );

    float roll_command =
        output.roll_control *
        ROLL_TO_MIXER_SCALE;

    output.motors =
        mixer_quad_x(
            throttle,
            roll_command,
            0.0f,
            0.0f
        );

    return output;
}