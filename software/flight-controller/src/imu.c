#include "imu.h"

#include <math.h>

#define GRAVITY 9.81f
#define DEG_TO_RAD (3.14159265358979323846f / 180.0f)

ImuSample imu_simulate(float roll_angle_deg, float roll_rate_deg_s)
{
    float roll_rad = roll_angle_deg * DEG_TO_RAD;

    ImuSample sample = {
        .gyro_x = roll_rate_deg_s,
        .gyro_y = 0.0f,
        .gyro_z = 0.0f,

        .accel_x = 0.0f,
        .accel_y = GRAVITY * sinf(roll_rad),
        .accel_z = GRAVITY * cosf(roll_rad)
    };

    return sample;
}