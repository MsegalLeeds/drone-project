#ifndef ATTITUDE_H
#define ATTITUDE_H

typedef struct {
    float roll;
} AttitudeEstimator;

void attitude_init(
    AttitudeEstimator *estimator,
    float initial_roll
);

float attitude_update(
    AttitudeEstimator *estimator,
    float accel_y,
    float accel_z,
    float gyro_x,
    float dt
);

#endif