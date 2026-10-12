#include "simulated_imu.h"

#define ROLL_DAMPING 1.5f
#define MOTOR_TORQUE_SCALE 50.0f

#define GRAVITY 9.81f
#define DRONE_MASS 1.0f

/*
 * Maximum thrust produced by one motor.
 *
 * With four motors:
 *
 * 4 * 0.5 * 4.905 = 9.81 N
 *
 * Therefore 0.5 throttle represents hover for
 * the current 1 kg simulated drone.
 */
#define MAX_MOTOR_THRUST 4.905f

void simulated_drone_init(
    SimulatedDrone *drone,
    float initial_roll
)
{
    drone->roll_angle = initial_roll;
    drone->roll_rate = 0.0f;

    drone->altitude = 0.0f;
    drone->vertical_velocity = 0.0f;
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

    /*
     * Roll dynamics
     */
    float left_motor_output =
        motors.front_left +
        motors.rear_left;

    float right_motor_output =
        motors.front_right +
        motors.rear_right;

    float roll_torque =
        left_motor_output -
        right_motor_output;

    float angular_acceleration =
        roll_torque * MOTOR_TORQUE_SCALE -
        ROLL_DAMPING * drone->roll_rate;

    drone->roll_rate +=
        angular_acceleration * dt;

    drone->roll_angle +=
        drone->roll_rate * dt;

    /*
     * Vertical dynamics
     */
    float total_motor_output =
        motors.front_left +
        motors.front_right +
        motors.rear_right +
        motors.rear_left;

    float total_thrust =
        total_motor_output * MAX_MOTOR_THRUST;

    float vertical_acceleration =
        (total_thrust / DRONE_MASS) -
        GRAVITY;

    drone->vertical_velocity +=
        vertical_acceleration * dt;

    drone->altitude +=
        drone->vertical_velocity * dt;
}