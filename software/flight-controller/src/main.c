#include <math.h>
#include <stdio.h>

#include "flight_controller.h"
#include "imu.h"
#include "simulated_imu.h"

#define RAD_TO_DEG 57.29577951308232f

int main(void)
{
    const float dt = 0.01f;

    const float target_roll = 0.0f;
    const float target_pitch = 0.0f;

    const float throttle = 0.5f;

    const float initial_roll = 20.0f;
    const float initial_pitch = -15.0f;

    const float roll_disturbance = 45.0f;
    const float pitch_disturbance = -30.0f;

    const float gyro_x_bias = 1.0f;
    const float gyro_y_bias = -0.75f;

    SimulatedDrone drone;

    simulated_drone_init_attitude(
        &drone,
        initial_roll,
        initial_pitch
    );

    /*
     * Gyro calibration.
     */
    enum {
        CALIBRATION_SAMPLES = 100
    };

    ImuSample calibration_samples[
        CALIBRATION_SAMPLES
    ];

    for (
        size_t i = 0;
        i < CALIBRATION_SAMPLES;
        i++
    ) {
        calibration_samples[i] =
            imu_simulate_attitude(
                drone.roll_angle,
                drone.pitch_angle,
                drone.roll_rate,
                drone.pitch_rate
            );

        calibration_samples[i].gyro_x +=
            gyro_x_bias;

        calibration_samples[i].gyro_y +=
            gyro_y_bias;
    }

    float gyro_x_offset =
        imu_calibrate_gyro_x(
            calibration_samples,
            CALIBRATION_SAMPLES
        );

    float gyro_y_offset =
        imu_calibrate_gyro_y(
            calibration_samples,
            CALIBRATION_SAMPLES
        );

    fprintf(
        stderr,
        "Gyro calibration: "
        "X=%.3f deg/s, Y=%.3f deg/s\n",
        gyro_x_offset,
        gyro_y_offset
    );

    /*
     * Initial attitude from accelerometer.
     */
    ImuSample sample =
        imu_simulate_attitude(
            drone.roll_angle,
            drone.pitch_angle,
            drone.roll_rate,
            drone.pitch_rate
        );

    float initial_roll_estimate =
        atan2f(
            sample.accel_y,
            sample.accel_z
        ) * RAD_TO_DEG;

    float initial_pitch_estimate =
        atan2f(
            -sample.accel_x,
            sqrtf(
                sample.accel_y *
                sample.accel_y +
                sample.accel_z *
                sample.accel_z
            )
        ) * RAD_TO_DEG;

    FlightController controller;

    flight_controller_init(
        &controller,
        initial_roll_estimate,
        initial_pitch_estimate,
        gyro_x_offset,
        gyro_y_offset
    );

    /*
     * Keep the original roll CSV fields
     * so plot_roll.py continues to work.
     */
    printf(
        "time,"
        "true_roll,"
        "estimated_roll,"
        "target_roll,"
        "control,"
        "true_pitch,"
        "estimated_pitch,"
        "target_pitch,"
        "pitch_control\n"
    );

    for (int i = 0; i < 500; i++) {

        if (i == 100) {
            drone.roll_rate +=
                roll_disturbance;

            drone.pitch_rate +=
                pitch_disturbance;
        }

        sample =
            imu_simulate_attitude(
                drone.roll_angle,
                drone.pitch_angle,
                drone.roll_rate,
                drone.pitch_rate
            );

        sample.gyro_x +=
            gyro_x_bias;

        sample.gyro_y +=
            gyro_y_bias;

        FlightControllerOutput output =
            flight_controller_update(
                &controller,
                &sample,
                target_roll,
                target_pitch,
                throttle,
                dt
            );

        simulated_drone_step_motors(
            &drone,
            output.motors,
            dt
        );

        printf(
            "%.2f,"
            "%.6f,"
            "%.6f,"
            "%.6f,"
            "%.6f,"
            "%.6f,"
            "%.6f,"
            "%.6f,"
            "%.6f\n",
            i * dt,

            drone.roll_angle,
            output.estimated_roll,
            target_roll,
            output.roll_control,

            drone.pitch_angle,
            output.estimated_pitch,
            target_pitch,
            output.pitch_control
        );
    }

    return 0;
}