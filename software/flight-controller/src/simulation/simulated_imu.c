#include "simulated_imu.h"

#define ROLL_DAMPING 1.5f
#define MOTOR_TORQUE_SCALE 50.0f

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
    if (dt <= 0.0f) {
        return;
    }

    float angular_acceleration =
        control_input -
        ROLL_DAMPING * drone->roll_rate;

    drone->roll_rate +=
        angular_acceleration * dt;

    drone->roll_angle +=
        drone->roll_rate * dt;
}

void simulated_drone_step_motors(
    SimulatedDrone *drone,
    MotorOutputs motors,
    float dt
)
{
    if (dt <= 0.0f) {
        return;
    }

    float left_thrust =
        motors.front_left +
        motors.rear_left;

    float right_thrust =
        motors.front_right +
        motors.rear_right;

    /*
     * Positive roll command in mixer_quad_x() increases
     * the left motors and decreases the right motors.
     */
    float roll_torque =
        left_thrust - right_thrust;

    float angular_acceleration =
        roll_torque * MOTOR_TORQUE_SCALE -
        ROLL_DAMPING * drone->roll_rate;

    drone->roll_rate +=
        angular_acceleration * dt;

    drone->roll_angle +=
        drone->roll_rate * dt;
}