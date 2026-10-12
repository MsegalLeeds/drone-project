#ifndef IMU_H
#define IMU_H

#include <stddef.h>

/* Roll-only ideal IMU: gravity only, with no sensor bias or noise. */
typedef struct {
    /* Angular rates in degrees/second. */
    float gyro_x;
    float gyro_y;
    float gyro_z;

    /* Acceleration in m/s^2; a level drone reads +9.81 on Z. */
    float accel_x;
    float accel_y;
    float accel_z;
} ImuSample;

/* Positive roll rotates gravity toward +Y; gyro_x follows roll rate. */
ImuSample imu_simulate(float roll_angle_deg, float roll_rate_deg_s);

/*
 * Mean gyro-X offset in degrees/second from stationary samples.
 * samples must be non-null and count must be greater than zero.
 */
float imu_calibrate_gyro_x(const ImuSample *samples, size_t count);

#endif
