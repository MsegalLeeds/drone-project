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
    const float initial_yaw = 0.0f;

    const float roll_disturbance = 45.0f;
    const float pitch_disturbance = -30.0f;

    /*
     * Simulated sensor biases.
     */
    const float gyro_x_bias = 1.0f;
    const float gyro_y_bias = -0.75f;
    const float gyro_z_bias = 0.5f;

    SimulatedDrone drone;

    simulated_drone_init_attitude_3d(
        &drone,
        initial_roll,
        initial_pitch,
        initial_yaw
    );

    /*
     * ---------------------------------------------------------
     * Gyro calibration
     * ---------------------------------------------------------
     *
     * The drone is stationary during calibration.
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
            imu_simulate_attitude_3d(
                drone.roll_angle,
                drone.pitch_angle,
                drone.roll_rate,
                drone.pitch_rate,
                drone.yaw_rate
            );

        calibration_samples[i].gyro_x +=
            gyro_x_bias;

        calibration_samples[i].gyro_y +=
            gyro_y_bias;

        calibration_samples[i].gyro_z +=
            gyro_z_bias;
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

    float gyro_z_offset =
        imu_calibrate_gyro_z(
            calibration_samples,
            CALIBRATION_SAMPLES
        );

    fprintf(
        stderr,
        "Gyro calibration: "
        "X=%.3f deg/s, "
        "Y=%.3f deg/s, "
        "Z=%.3f deg/s\n",
        gyro_x_offset,
        gyro_y_offset,
        gyro_z_offset
    );

    /*
     * ---------------------------------------------------------
     * Initial attitude estimate
     * ---------------------------------------------------------
     *
     * Roll and pitch come from gravity.
     *
     * There is no absolute yaw estimate because the current
     * sensor model has no magnetometer or heading reference.
     */
    ImuSample sample =
        imu_simulate_attitude_3d(
            drone.roll_angle,
            drone.pitch_angle,
            drone.roll_rate,
            drone.pitch_rate,
            drone.yaw_rate
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

    /*
     * ---------------------------------------------------------
     * Flight controller
     * ---------------------------------------------------------
     */
    FlightController controller;

    flight_controller_init_3d(
        &controller,
        initial_roll_estimate,
        initial_pitch_estimate,
        gyro_x_offset,
        gyro_y_offset,
        gyro_z_offset
    );

    /*
     * Keep the original roll/pitch CSV fields intact so the
     * existing plot_roll.py script still works.
     *
     * Yaw-related fields are appended.
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
        "pitch_control,"
        "true_yaw,"
        "true_yaw_rate,"
        "target_yaw_rate,"
        "yaw_control\n"
    );

    /*
     * Eight-second simulation.
     */
    for (int i = 0; i < 800; i++) {

        /*
         * At t = 1 second apply roll/pitch disturbances.
         */
        if (i == 100) {
            drone.roll_rate +=
                roll_disturbance;

            drone.pitch_rate +=
                pitch_disturbance;
        }

        /*
         * Command a +45 deg/s yaw spin from
         * t = 1 s until t = 5 s.
         *
         * Afterwards command zero yaw rate so the
         * controller should stop the rotation.
         */
        float target_yaw_rate =
            (i >= 100 && i < 500)
                ? 45.0f
                : 0.0f;

        /*
         * -----------------------------------------------------
         * Sensor measurement
         * -----------------------------------------------------
         */
        sample =
            imu_simulate_attitude_3d(
                drone.roll_angle,
                drone.pitch_angle,
                drone.roll_rate,
                drone.pitch_rate,
                drone.yaw_rate
            );

        /*
         * Add simulated real-world gyro biases.
         *
         * FlightController removes the calibrated offsets.
         */
        sample.gyro_x +=
            gyro_x_bias;

        sample.gyro_y +=
            gyro_y_bias;

        sample.gyro_z +=
            gyro_z_bias;

        /*
         * -----------------------------------------------------
         * Flight-control loop
         * -----------------------------------------------------
         */
        FlightControllerOutput output =
            flight_controller_update_3d(
                &controller,
                &sample,
                target_roll,
                target_pitch,
                target_yaw_rate,
                throttle,
                dt
            );

        /*
         * Apply motor outputs to simulated drone physics.
         */
        simulated_drone_step_motors(
            &drone,
            output.motors,
            dt
        );

        /*
         * -----------------------------------------------------
         * CSV output
         * -----------------------------------------------------
         */
        printf(
            "%.2f,"
            "%.6f,"
            "%.6f,"
            "%.6f,"
            "%.6f,"
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
            output.pitch_control,

            drone.yaw_angle,
            drone.yaw_rate,
            target_yaw_rate,
            output.yaw_control
        );
    }

    return 0;
}