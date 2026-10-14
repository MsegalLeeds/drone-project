#ifndef ATTITUDE_H
#define ATTITUDE_H

typedef struct {
    float roll;
    float pitch;
} AttitudeEstimator;

/*
 * Legacy roll-only initialisation.
 *
 * Pitch starts at zero.
 */
void attitude_init(
    AttitudeEstimator *estimator,
    float initial_roll
);

/*
 * Initialise roll and pitch.
 */
void attitude_init_2d(
    AttitudeEstimator *estimator,
    float initial_roll,
    float initial_pitch
);

/*
 * Legacy roll-only complementary filter.
 */
float attitude_update(
    AttitudeEstimator *estimator,
    float accel_y,
    float accel_z,
    float gyro_x,
    float dt
);

/*
 * Roll + pitch complementary filter.
 */
void attitude_update_2d(
    AttitudeEstimator *estimator,
    float accel_x,
    float accel_y,
    float accel_z,
    float gyro_x,
    float gyro_y,
    float dt
);

#endif