#include <math.h>
#include <stdio.h>

#include "attitude.h"
#include "imu.h"
#include "pid.h"
#include "simulated_imu.h"

int main(void)
{
    PID roll_pid;
    SimulatedDrone drone;
    AttitudeEstimator estimator;

    const float target_roll = 0.0f;
    const float dt = 0.01f;
    const float rad_to_deg = 57.29577951308232f;
    const float initial_roll_deg = 20.0f;
    const float disturbance_rate = 45.0f;
    const float gyro_bias = 1.0f;

    pid_init(&roll_pid, 5.0f, 0.0f, 2.0f);
    pid_set_limits(&roll_pid, -100.0f, 100.0f);

    simulated_drone_init(&drone, initial_roll_deg);

    /* Initialise the estimate from the accelerometer reading. */
    ImuSample sample =
        imu_simulate(drone.roll_angle, drone.roll_rate);

    float initial_roll =
        atan2f(sample.accel_y, sample.accel_z) * rad_to_deg;

    attitude_init(&estimator, initial_roll);

    printf("time,true_roll,estimated_roll,target_roll,control\n");

    for (int i = 0; i < 500; i++) {
        sample = imu_simulate(drone.roll_angle, drone.roll_rate);

        float estimated_roll = attitude_update(
            &estimator,
            sample.accel_y,
            sample.accel_z,
            sample.gyro_x,
            dt
        );

        /* Rotational shove at one second. */
        if (i == 100) {
            drone.roll_rate += disturbance_rate;
        }

        sample = imu_simulate(drone.roll_angle, drone.roll_rate);
        sample.gyro_x += gyro_bias;

        /* The controller receives the estimate, not the true angle. */
        float control = pid_update(
            &roll_pid,
            target_roll,
            estimated_roll,
            dt
        );

        printf(
            "%.2f,%.6f,%.6f,%.6f,%.6f\n",
            i * dt,
            drone.roll_angle,
            estimated_roll,
            target_roll,
            control
        );


        simulated_drone_step(&drone, control, dt);
    }

    return 0;
}