#include "imu.h"

#include <assert.h>
#include <math.h>

#define GRAVITY 9.81f

#define DEG_TO_RAD \
    (3.14159265358979323846f / 180.0f)

ImuSample imu_simulate_attitude_3d(
    float roll_angle_deg,
    float pitch_angle_deg,
    float roll_rate_deg_s,
    float pitch_rate_deg_s,
    float yaw_rate_deg_s
)
{
    float roll_rad =
        roll_angle_deg * DEG_TO_RAD;

    float pitch_rad =
        pitch_angle_deg * DEG_TO_RAD;

    ImuSample sample = {
        .gyro_x = roll_rate_deg_s,
        .gyro_y = pitch_rate_deg_s,
        .gyro_z = yaw_rate_deg_s,

        .accel_x =
            -GRAVITY * sinf(pitch_rad),

        .accel_y =
            GRAVITY *
            sinf(roll_rad) *
            cosf(pitch_rad),

        .accel_z =
            GRAVITY *
            cosf(roll_rad) *
            cosf(pitch_rad)
    };

    return sample;
}

ImuSample imu_simulate_attitude(
    float roll_angle_deg,
    float pitch_angle_deg,
    float roll_rate_deg_s,
    float pitch_rate_deg_s
)
{
    return imu_simulate_attitude_3d(
        roll_angle_deg,
        pitch_angle_deg,
        roll_rate_deg_s,
        pitch_rate_deg_s,
        0.0f
    );
}

ImuSample imu_simulate(
    float roll_angle_deg,
    float roll_rate_deg_s
)
{
    return imu_simulate_attitude(
        roll_angle_deg,
        0.0f,
        roll_rate_deg_s,
        0.0f
    );
}

float imu_calibrate_gyro_x(
    const ImuSample *samples,
    size_t count
)
{
    assert(samples != NULL);
    assert(count > 0);

    double sum = 0.0;

    for (size_t i = 0; i < count; i++) {
        sum += samples[i].gyro_x;
    }

    return (float)(sum / count);
}

float imu_calibrate_gyro_y(
    const ImuSample *samples,
    size_t count
)
{
    assert(samples != NULL);
    assert(count > 0);

    double sum = 0.0;

    for (size_t i = 0; i < count; i++) {
        sum += samples[i].gyro_y;
    }

    return (float)(sum / count);
}

float imu_calibrate_gyro_z(
    const ImuSample *samples,
    size_t count
)
{
    assert(samples != NULL);
    assert(count > 0);

    double sum = 0.0;

    for (size_t i = 0; i < count; i++) {
        sum += samples[i].gyro_z;
    }

    return (float)(sum / count);
}