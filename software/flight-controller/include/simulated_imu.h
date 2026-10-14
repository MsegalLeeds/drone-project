#ifndef SIMULATED_IMU_H
#define SIMULATED_IMU_H

#include "mixer.h"

typedef struct {
    float roll_angle;
    float roll_rate;

    float pitch_angle;
    float pitch_rate;

    /* Positive values are upward. */
    float altitude;
    float vertical_velocity;

    float yaw_angle;
    float yaw_rate;
} SimulatedDrone;

/*
 * Legacy roll-only initialisation.
 *
 * Pitch starts at zero.
 */
void simulated_drone_init(
    SimulatedDrone *drone,
    float initial_roll
);

/*
 * Initialise both roll and pitch.
 */
void simulated_drone_init_attitude(
    SimulatedDrone *drone,
    float initial_roll,
    float initial_pitch
);

/*
 * Initialise roll, pitch and yaw.
 */
void simulated_drone_init_attitude_3d(
    SimulatedDrone *drone,
    float initial_roll,
    float initial_pitch,
    float initial_yaw
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
 * This models:
 * - roll dynamics
 * - pitch dynamics
 * - yaw dynamics
 * - vertical thrust
 * - gravity
 */
void simulated_drone_step_motors(
    SimulatedDrone *drone,
    MotorOutputs motors,
    float dt
);

#endif