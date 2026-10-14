#include "flight_controller.h"

#define ROLL_KP 5.0f
#define ROLL_KI 0.0f
#define ROLL_KD 2.0f

#define PITCH_KP 5.0f
#define PITCH_KI 0.0f
#define PITCH_KD 2.0f

/*
 * Yaw is currently a RATE controller.
 *
 * We are controlling gyro Z directly rather than
 * absolute yaw angle because there is currently
 * no magnetometer / heading reference.
 */
#define YAW_RATE_KP 2.0f
#define YAW_RATE_KI 3.0f
#define YAW_RATE_KD 0.0f

#define CONTROL_MIN -100.0f
#define CONTROL_MAX 100.0f

#define CONTROL_TO_MIXER_SCALE \
    (1.0f / 200.0f)

void flight_controller_init_3d(
    FlightController *controller,
    float initial_roll_deg,
    float initial_pitch_deg,
    float gyro_x_offset,
    float gyro_y_offset,
    float gyro_z_offset
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

    pid_init(
        &controller->yaw_rate_pid,
        YAW_RATE_KP,
        YAW_RATE_KI,
        YAW_RATE_KD
    );

    pid_set_limits(
        &controller->yaw_rate_pid,
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

    controller->gyro_z_offset =
        gyro_z_offset;
}

void flight_controller_init(
    FlightController *controller,
    float initial_roll_deg,
    float initial_pitch_deg,
    float gyro_x_offset,
    float gyro_y_offset
)
{
    flight_controller_init_3d(
        controller,
        initial_roll_deg,
        initial_pitch_deg,
        gyro_x_offset,
        gyro_y_offset,
        0.0f
    );
}

FlightControllerOutput flight_controller_update_3d(
    FlightController *controller,
    const ImuSample *sample,
    float target_roll_deg,
    float target_pitch_deg,
    float target_yaw_rate_deg_s,
    float throttle,
    float dt
)
{
    FlightControllerOutput output = {
        .estimated_roll = 0.0f,
        .estimated_pitch = 0.0f,
        .measured_yaw_rate = 0.0f,

        .roll_control = 0.0f,
        .pitch_control = 0.0f,
        .yaw_control = 0.0f,

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

    float corrected_gyro_z =
        sample->gyro_z -
        controller->gyro_z_offset;

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

    output.measured_yaw_rate =
        corrected_gyro_z;

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

    output.yaw_control =
        pid_update(
            &controller->yaw_rate_pid,
            target_yaw_rate_deg_s,
            output.measured_yaw_rate,
            dt
        );

    float roll_command =
        output.roll_control *
        CONTROL_TO_MIXER_SCALE;

    float pitch_command =
        output.pitch_control *
        CONTROL_TO_MIXER_SCALE;

    float yaw_command =
        output.yaw_control *
        CONTROL_TO_MIXER_SCALE;

    output.motors =
        mixer_quad_x(
            throttle,
            roll_command,
            pitch_command,
            yaw_command
        );

    return output;
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
    return flight_controller_update_3d(
        controller,
        sample,
        target_roll_deg,
        target_pitch_deg,
        0.0f,
        throttle,
        dt
    );
}