#ifndef SIMULATED_IMU_H
#define SIMULATED_IMU_H

typedef struct {
    float roll_angle;
    float roll_rate;
} SimulatedDrone;

void simulated_drone_init(
    SimulatedDrone *drone,
    float initial_roll
);

void simulated_drone_step(
    SimulatedDrone *drone,
    float control_input,
    float dt
);

#endif
