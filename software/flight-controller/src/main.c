#include <math.h>
#include <stdio.h>

#include "flight_controller.h"
#include "imu.h"
#include "simulated_imu.h"

int main(void)
{
    FlightController controller;
    SimulatedDrone drone;

    const float target_roll = 0.0f;
    const float throttle = 0.5f;
    const float dt = 0.01f;

    const float rad_to_deg =
        57.29577951308232f;

    const float initial_roll_deg =
        20.0f;

    const float disturbance_rate =
        45.0f;

    const float gyro_bias =
        1.0f;

    simulated_drone_init(
        &drone,
        initial_roll_deg
    );

    /*
     * Calibrate the gyro while stationary.
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
            imu_simulate(
                drone.roll_angle,
                drone.roll_rate
            );

        calibration_samples[i].gyro_x +=
            gyro_bias;
    }

    float gyro_offset =
        imu_calibrate_gyro_x(
            calibration_samples,
            CALIBRATION_SAMPLES
        );

    fprintf(
        stderr,
        "Gyro-X calibration: %.3f deg/s\n",
        gyro_offset
    );

    /*
     * Determine the initial roll estimate from
     * gravity measured by the accelerometer.
     */
    ImuSample sample =
        imu_simulate(
            drone.roll_angle,
            drone.roll_rate
        );

    float initial_roll =
        atan2f(
            sample.accel_y,
            sample.accel_z
        ) * rad_to_deg;

    flight_controller_init(
        &controller,
        initial_roll,
        gyro_offset
    );

    printf(
        "time,true_roll,estimated_roll,"
        "target_roll,control\n"
    );

    for (int i = 0; i < 500; i++) {

        /*
         * Apply a disturbance after one second.
         */
        if (i == 100) {
            drone.roll_rate +=
                disturbance_rate;
        }

        sample =
            imu_simulate(
                drone.roll_angle,
                drone.roll_rate
            );

        /*
         * Simulate real sensor bias.
         *
         * FlightController removes the calibrated
         * offset internally.
         */
        sample.gyro_x +=
            gyro_bias;

        FlightControllerOutput output =
            flight_controller_update(
                &controller,
                &sample,
                target_roll,
                throttle,
                dt
            );

        simulated_drone_step_motors(
            &drone,
            output.motors,
            dt
        );

        printf(
            "%.2f,%.6f,%.6f,%.6f,%.6f\n",
            i * dt,
            drone.roll_angle,
            output.estimated_roll,
            target_roll,
            output.roll_control
        );
    }

    return 0;
}