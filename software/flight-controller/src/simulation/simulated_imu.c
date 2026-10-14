#include "simulated_imu.h"

#define ROLL_DAMPING 1.5f
#define PITCH_DAMPING 1.5f
#define YAW_DAMPING 1.5f

#define MOTOR_TORQUE_SCALE 50.0f
#define YAW_TORQUE_SCALE 50.0f

#define GRAVITY 9.81f
#define DRONE_MASS 1.0f

/*
 * Maximum thrust produced by one motor.
 *
 * Four motors at 0.5:
 *
 * 4 * 0.5 * 4.905 = 9.81 N
 *
 * Therefore 0.5 throttle represents hover for
 * the current 1 kg simulated drone.
 */
#define MAX_MOTOR_THRUST 4.905f

void simulated_drone_init_attitude_3d(
    SimulatedDrone *drone,
    float initial_roll,
    float initial_pitch,
    float initial_yaw
)
{
    drone->roll_angle = initial_roll;
    drone->roll_rate = 0.0f;

    drone->pitch_angle = initial_pitch;
    drone->pitch_rate = 0.0f;

    drone->yaw_angle = initial_yaw;
    drone->yaw_rate = 0.0f;

    drone->altitude = 0.0f;
    drone->vertical_velocity = 0.0f;
}

void simulated_drone_init_attitude(
    SimulatedDrone *drone,
    float initial_roll,
    float initial_pitch
)
{
    simulated_drone_init_attitude_3d(
        drone,
        initial_roll,
        initial_pitch,
        0.0f
    );
}

void simulated_drone_init(
    SimulatedDrone *drone,
    float initial_roll
)
{
    simulated_drone_init_attitude(
        drone,
        initial_roll,
        0.0f
    );
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
     * Roll dynamics.
     *
     * More thrust on the left produces
     * positive roll.
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

    float roll_acceleration =
        roll_torque * MOTOR_TORQUE_SCALE -
        ROLL_DAMPING * drone->roll_rate;

    drone->roll_rate +=
        roll_acceleration * dt;

    drone->roll_angle +=
        drone->roll_rate * dt;

    /*
     * Pitch dynamics.
     *
     * More thrust at the rear produces
     * positive pitch.
     */
    float front_motor_output =
        motors.front_left +
        motors.front_right;

    float rear_motor_output =
        motors.rear_left +
        motors.rear_right;

    float pitch_torque =
        rear_motor_output -
        front_motor_output;

    float pitch_acceleration =
        pitch_torque * MOTOR_TORQUE_SCALE -
        PITCH_DAMPING * drone->pitch_rate;

    drone->pitch_rate +=
        pitch_acceleration * dt;

    drone->pitch_angle +=
        drone->pitch_rate * dt;
    /*
     * Yaw dynamics.
     *
     * Front-right and rear-left rotate CW.
     * Front-left and rear-right rotate CCW.
     *
     * Positive yaw command increases the
     * CW motor pair.
     */
    float clockwise_motor_output =
        motors.front_right +
        motors.rear_left;

    float counterclockwise_motor_output =
        motors.front_left +
        motors.rear_right;

    float yaw_torque =
        clockwise_motor_output -
        counterclockwise_motor_output;

    float yaw_acceleration =
        yaw_torque * YAW_TORQUE_SCALE -
        YAW_DAMPING * drone->yaw_rate;

    drone->yaw_rate +=
        yaw_acceleration * dt;

    drone->yaw_angle +=
        drone->yaw_rate * dt;

    /*
     * Vertical dynamics.
     */
    float total_motor_output =
        motors.front_left +
        motors.front_right +
        motors.rear_right +
        motors.rear_left;

    float total_thrust =
        total_motor_output *
        MAX_MOTOR_THRUST;

    float vertical_acceleration =
        (total_thrust / DRONE_MASS) -
        GRAVITY;

    drone->vertical_velocity +=
        vertical_acceleration * dt;

    drone->altitude +=
        drone->vertical_velocity * dt;
}