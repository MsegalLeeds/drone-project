#include "simulated_imu.h"

void simulated_drone_init(
    SimulatedDrone *drone,
    float initial_roll
)
{
    drone->roll_angle = initial_roll;
    drone->roll_rate = 0.0f;
}

void simulated_drone_step(
    SimulatedDrone *drone,
    float control_input,
    float dt
)
{
    const float damping = 1.5f;

    float angular_acceleration =
        control_input -
        damping * drone->roll_rate;

    drone->roll_rate +=
        angular_acceleration * dt;

    drone->roll_angle +=
        drone->roll_rate * dt;
}
