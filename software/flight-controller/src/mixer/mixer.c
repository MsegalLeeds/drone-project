#include "mixer.h"

static float clamp(float value, float min, float max)
{
    if (value < min) {
        return min;
    }

    if (value > max) {
        return max;
    }

    return value;
}

MotorOutputs mixer_quad_x(
    float throttle,
    float roll,
    float pitch,
    float yaw
)
{
    MotorOutputs motors = {
        .front_left =
            throttle + roll - pitch - yaw,

        .front_right =
            throttle - roll - pitch + yaw,

        .rear_right =
            throttle - roll + pitch - yaw,

        .rear_left =
            throttle + roll + pitch + yaw
    };

    motors.front_left =
        clamp(motors.front_left, 0.0f, 1.0f);

    motors.front_right =
        clamp(motors.front_right, 0.0f, 1.0f);

    motors.rear_right =
        clamp(motors.rear_right, 0.0f, 1.0f);

    motors.rear_left =
        clamp(motors.rear_left, 0.0f, 1.0f);

    return motors;
}