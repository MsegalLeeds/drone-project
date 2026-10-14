#include "flight_controller.h"

#define ROLL_KP 5.0f
#define ROLL_KI 0.0f
#define ROLL_KD 2.0f

#define PITCH_KP 5.0f
#define PITCH_KI 0.0f
#define PITCH_KD 2.0f

#define CONTROL_MIN -100.0f
#define CONTROL_MAX 100.0f

#define CONTROL_TO_MIXER_SCALE \
    (1.0f / 200.0f)

void flight_controller_init(
    FlightController *controller,
    float initial_roll_deg,
    float initial_pitch_deg,
    float gyro_x_offset,
    float gyro_y_offset
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
        CONTROL_MIN,
        CONTROL_MAX
    );

    pid_init(
        &controller->pitch_pid,
        PITCH_KP,
        PITCH_KI,
        PITCH_KD
    );

    pid_set_limits(
        &controller->pitch_pid,
        CONTROL_MIN,
        CONTROL_MAX
    );

    attitude_init_2d(
        &controller->attitude,
        initial_roll_deg,
        initial_pitch_deg
    );

    controller->gyro_x_offset =
        gyro_x_offset;

    controller->gyro_y_offset =
        gyro_y_offset;
}

FlightControllerOutput flight_controller_update(
    FlightController *controller,
    const ImuSample *sample,
    float target_roll_deg,
    float target_pitch_deg,
    float throttle,
    float dt
)
{
    FlightControllerOutput output = {
        .estimated_roll = 0.0f,
        .estimated_pitch = 0.0f,

        .roll_control = 0.0f,
        .pitch_control = 0.0f,

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

    float corrected_gyro_x =
        sample->gyro_x -
        controller->gyro_x_offset;

    float corrected_gyro_y =
        sample->gyro_y -
        controller->gyro_y_offset;

    attitude_update_2d(
        &controller->attitude,
        sample->accel_x,
        sample->accel_y,
        sample->accel_z,
        corrected_gyro_x,
        corrected_gyro_y,
        dt
    );

    output.estimated_roll =
        controller->attitude.roll;

    output.estimated_pitch =
        controller->attitude.pitch;

    output.roll_control =
        pid_update(
            &controller->roll_pid,
            target_roll_deg,
            output.estimated_roll,
            dt
        );

    output.pitch_control =
        pid_update(
            &controller->pitch_pid,
            target_pitch_deg,
            output.estimated_pitch,
            dt
        );

    float roll_command =
        output.roll_control *
        CONTROL_TO_MIXER_SCALE;

    float pitch_command =
        output.pitch_control *
        CONTROL_TO_MIXER_SCALE;

    output.motors =
        mixer_quad_x(
            throttle,
            roll_command,
            pitch_command,
            0.0f
        );

    return output;
}