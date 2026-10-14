#include <math.h>

#include "attitude.h"

#define RAD_TO_DEG 57.29577951308232f
#define FILTER_ALPHA 0.98f

void attitude_init_2d(
    AttitudeEstimator *estimator,
    float initial_roll,
    float initial_pitch
)
{
    estimator->roll = initial_roll;
    estimator->pitch = initial_pitch;
}

void attitude_init(
    AttitudeEstimator *estimator,
    float initial_roll
)
{
    attitude_init_2d(
        estimator,
        initial_roll,
        0.0f
    );
}

float attitude_update(
    AttitudeEstimator *estimator,
    float accel_y,
    float accel_z,
    float gyro_x,
    float dt
)
{
    float accel_roll =
        atan2f(
            accel_y,
            accel_z
        ) * RAD_TO_DEG;

    float gyro_roll =
        estimator->roll +
        gyro_x * dt;

    estimator->roll =
        FILTER_ALPHA * gyro_roll +
        (1.0f - FILTER_ALPHA) *
        accel_roll;

    return estimator->roll;
}

void attitude_update_2d(
    AttitudeEstimator *estimator,
    float accel_x,
    float accel_y,
    float accel_z,
    float gyro_x,
    float gyro_y,
    float dt
)
{
    /*
     * Roll from gravity projected into Y/Z.
     */
    float accel_roll =
        atan2f(
            accel_y,
            accel_z
        ) * RAD_TO_DEG;

    /*
     * Pitch from gravity projected against
     * the Y/Z magnitude.
     */
    float accel_pitch =
        atan2f(
            -accel_x,
            sqrtf(
                accel_y * accel_y +
                accel_z * accel_z
            )
        ) * RAD_TO_DEG;

    float gyro_roll =
        estimator->roll +
        gyro_x * dt;

    float gyro_pitch =
        estimator->pitch +
        gyro_y * dt;

    estimator->roll =
        FILTER_ALPHA * gyro_roll +
        (1.0f - FILTER_ALPHA) *
        accel_roll;

    estimator->pitch =
        FILTER_ALPHA * gyro_pitch +
        (1.0f - FILTER_ALPHA) *
        accel_pitch;
}