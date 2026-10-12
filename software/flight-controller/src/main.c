#include <stdio.h>

#include "pid.h"
#include "simulated_imu.h"

int main(void)
{
    PID roll_pid;
    SimulatedDrone drone;

    pid_init(
        &roll_pid,
        5.0f,
        0.0f,
        2.0f
    );

    pid_set_limits(
        &roll_pid,
        -100.0f,
        100.0f
    );

    simulated_drone_init(
        &drone,
        20.0f
    );

    const float target_roll = 0.0f;
    const float dt = 0.01f;

    for (int i = 0; i < 500; i++) {

        float control =
            pid_update(
                &roll_pid,
                target_roll,
                drone.roll_angle,
                dt
            );

        simulated_drone_step(
            &drone,
            control,
            dt
        );

        if (i % 10 == 0) {
            printf(
                "time: %.2f  roll: %.2f  control: %.2f\n",
                i * dt,
                drone.roll_angle,
                control
            );
        }
    }

    return 0;
}