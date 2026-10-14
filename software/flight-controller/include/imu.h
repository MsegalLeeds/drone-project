#ifndef IMU_H
#define IMU_H

#include <stddef.h>

typedef struct {
    /* Angular rates in degrees/second. */
    float gyro_x;
    float gyro_y;
    float gyro_z;

    /* Acceleration in m/s^2. */
    float accel_x;
    float accel_y;
    float accel_z;
} ImuSample;

/*
 * Legacy roll-only IMU simulation.
 *
 * Pitch and pitch rate are assumed to be zero.
 */
ImuSample imu_simulate(
    float roll_angle_deg,
    float roll_rate_deg_s
);

/*
 * Simulate roll and pitch.
 *
 * Yaw rate is assumed to be zero.
 */
ImuSample imu_simulate_attitude(
    float roll_angle_deg,
    float pitch_angle_deg,
    float roll_rate_deg_s,
    float pitch_rate_deg_s
);

/*
 * Simulate roll, pitch and yaw rate.
 *
 * Absolute yaw angle is not needed because
 * gravity cannot provide a yaw heading.
 */
ImuSample imu_simulate_attitude_3d(
    float roll_angle_deg,
    float pitch_angle_deg,
    float roll_rate_deg_s,
    float pitch_rate_deg_s,
    float yaw_rate_deg_s
);

float imu_calibrate_gyro_x(
    const ImuSample *samples,
    size_t count
);

float imu_calibrate_gyro_y(
    const ImuSample *samples,
    size_t count
);

float imu_calibrate_gyro_z(
    const ImuSample *samples,
    size_t count
);

#endif