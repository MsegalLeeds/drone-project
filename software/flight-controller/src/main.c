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

    pid_init(&roll_pid, 5.0f, 0.0f, 2.0f);
    pid_set_limits(&roll_pid, -100.0f, 100.0f);

    simulated_drone_init(&drone, -20.0f);

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

        /* The controller receives the estimate, not the true angle. */
        float control = pid_update(
            &roll_pid,
            target_roll,
            estimated_roll,
            dt
        );

        if (i % 10 == 0) {
            printf(
                "%.2f,%.3f,%.3f,%.3f,%.3f\n",
                i * dt,
                drone.roll_angle,
                estimated_roll,
                target_roll,
                control
            );
        }

        simulated_drone_step(&drone, control, dt);
    }

    return 0;
}