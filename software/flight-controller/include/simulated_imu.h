#ifndef SIMULATED_IMU_H
#define SIMULATED_IMU_H

#include "mixer.h"

typedef struct {
    float roll_angle;
    float roll_rate;
} SimulatedDrone;

void simulated_drone_init(
    SimulatedDrone *drone,
    float initial_roll
);

/*
 * Legacy direct-control simulation step.
 *
 * Kept temporarily so the existing closed-loop simulation
 * continues to build while the motor-driven model is introduced.
 */
void simulated_drone_step(
    SimulatedDrone *drone,
    float control_input,
    float dt
);

/*
 * Advance the roll simulation using four normalized motor outputs.
 *
 * Motor outputs are expected to be in the range 0.0f - 1.0f.
 */
void simulated_drone_step_motors(
    SimulatedDrone *drone,
    MotorOutputs motors,
    float dt
);

#endif