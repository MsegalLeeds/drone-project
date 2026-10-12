#ifndef SIMULATED_IMU_H
#define SIMULATED_IMU_H

#include "mixer.h"

typedef struct {
    float roll_angle;
    float roll_rate;

    /* Positive values are upward. */
    float altitude;
    float vertical_velocity;
} SimulatedDrone;

void simulated_drone_init(
    SimulatedDrone *drone,
    float initial_roll
);

/*
 * Legacy roll-only simulation step.
 */
void simulated_drone_step(
    SimulatedDrone *drone,
    float control_input,
    float dt
);

/*
 * Advance the simulation using the four motor outputs.
 *
 * This currently models:
 * - roll dynamics
 * - vertical thrust
 * - gravity
 */
void simulated_drone_step_motors(
    SimulatedDrone *drone,
    MotorOutputs motors,
    float dt
);

#endif