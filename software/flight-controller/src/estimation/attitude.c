#include <math.h>

#include "attitude.h"

#define RAD_TO_DEG 57.29577951308232f

void attitude_init(
    AttitudeEstimator *estimator,
    float initial_roll
)
{
    estimator->roll = initial_roll;
}

float attitude_update(
    AttitudeEstimator *estimator,
    float accel_y,
    float accel_z,
    float gyro_x,
    float dt
)
{
    const float alpha = 0.98f;

    float accel_roll =
        atan2f(accel_y, accel_z) * RAD_TO_DEG;

    float gyro_roll =
        estimator->roll + gyro_x * dt;

    estimator->roll =
        alpha * gyro_roll +
        (1.0f - alpha) * accel_roll;

    return estimator->roll;
}