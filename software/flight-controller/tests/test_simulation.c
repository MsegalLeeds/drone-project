#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "imu.h"
#include "mixer.h"
#include "simulated_imu.h"

static int nearly_equal(float a, float b)
{
    return fabsf(a - b) < 0.01f;
}

static void test_initialization(void)
{
    SimulatedDrone drone;

    simulated_drone_init(&drone, 20.0f);

    assert(nearly_equal(drone.roll_angle, 20.0f));
    assert(nearly_equal(drone.roll_rate, 0.0f));

    assert(nearly_equal(drone.altitude, 0.0f));
    assert(nearly_equal(drone.vertical_velocity, 0.0f));
}

static void test_no_control_input(void)
{
    SimulatedDrone drone;

    simulated_drone_init(&drone, 0.0f);

    simulated_drone_step(&drone, 0.0f, 0.1f);

    assert(nearly_equal(drone.roll_angle, 0.0f));
    assert(nearly_equal(drone.roll_rate, 0.0f));
}

static void test_positive_control_input(void)
{
    SimulatedDrone drone;

    simulated_drone_init(&drone, 0.0f);

    simulated_drone_step(&drone, 10.0f, 0.1f);

    assert(drone.roll_rate > 0.0f);
    assert(drone.roll_angle > 0.0f);
}

static void test_level_imu(void)
{
    SimulatedDrone drone;

    simulated_drone_init(&drone, 0.0f);

    ImuSample imu =
        imu_simulate(
            drone.roll_angle,
            drone.roll_rate
        );

    assert(nearly_equal(imu.accel_x, 0.0f));
    assert(nearly_equal(imu.accel_y, 0.0f));
    assert(nearly_equal(imu.accel_z, 9.81f));
    assert(nearly_equal(imu.gyro_x, 0.0f));
}

static void test_ninety_degree_roll_imu(void)
{
    SimulatedDrone drone;

    simulated_drone_init(&drone, 90.0f);

    ImuSample imu =
        imu_simulate(
            drone.roll_angle,
            drone.roll_rate
        );

    assert(nearly_equal(imu.accel_y, 9.81f));
    assert(nearly_equal(imu.accel_z, 0.0f));
}

static void test_negative_roll_sensor_signs(void)
{
    SimulatedDrone drone;

    simulated_drone_init(&drone, -30.0f);

    ImuSample imu =
        imu_simulate(
            drone.roll_angle,
            drone.roll_rate
        );

    assert(nearly_equal(imu.accel_y, -4.905f));
    assert(nearly_equal(imu.accel_z, 8.496f));

    simulated_drone_step(
        &drone,
        -10.0f,
        0.1f
    );

    imu =
        imu_simulate(
            drone.roll_angle,
            drone.roll_rate
        );

    assert(nearly_equal(imu.gyro_x, -1.0f));
    assert(nearly_equal(imu.gyro_y, 0.0f));
    assert(nearly_equal(imu.gyro_z, 0.0f));
}

static void test_equal_motor_outputs_produce_no_roll(void)
{
    SimulatedDrone drone;

    simulated_drone_init(&drone, 0.0f);

    MotorOutputs motors = {
        .front_left = 0.5f,
        .front_right = 0.5f,
        .rear_right = 0.5f,
        .rear_left = 0.5f
    };

    simulated_drone_step_motors(
        &drone,
        motors,
        0.1f
    );

    assert(nearly_equal(drone.roll_rate, 0.0f));
    assert(nearly_equal(drone.roll_angle, 0.0f));
}

static void test_stronger_left_motors_produce_positive_roll(void)
{
    SimulatedDrone drone;

    simulated_drone_init(&drone, 0.0f);

    MotorOutputs motors = {
        .front_left = 0.6f,
        .front_right = 0.4f,
        .rear_right = 0.4f,
        .rear_left = 0.6f
    };

    simulated_drone_step_motors(
        &drone,
        motors,
        0.1f
    );

    assert(drone.roll_rate > 0.0f);
    assert(drone.roll_angle > 0.0f);
}

static void test_stronger_right_motors_produce_negative_roll(void)
{
    SimulatedDrone drone;

    simulated_drone_init(&drone, 0.0f);

    MotorOutputs motors = {
        .front_left = 0.4f,
        .front_right = 0.6f,
        .rear_right = 0.6f,
        .rear_left = 0.4f
    };

    simulated_drone_step_motors(
        &drone,
        motors,
        0.1f
    );

    assert(drone.roll_rate < 0.0f);
    assert(drone.roll_angle < 0.0f);
}

static void test_equal_motors_allow_roll_damping(void)
{
    SimulatedDrone drone;

    simulated_drone_init(&drone, 0.0f);

    drone.roll_rate = 10.0f;

    MotorOutputs motors = {
        .front_left = 0.5f,
        .front_right = 0.5f,
        .rear_right = 0.5f,
        .rear_left = 0.5f
    };

    simulated_drone_step_motors(
        &drone,
        motors,
        0.1f
    );

    assert(drone.roll_rate > 0.0f);
    assert(drone.roll_rate < 10.0f);
}

static void test_hover_throttle_has_no_vertical_acceleration(void)
{
    SimulatedDrone drone;

    simulated_drone_init(&drone, 0.0f);

    MotorOutputs motors = {
        .front_left = 0.5f,
        .front_right = 0.5f,
        .rear_right = 0.5f,
        .rear_left = 0.5f
    };

    simulated_drone_step_motors(
        &drone,
        motors,
        0.1f
    );

    assert(nearly_equal(
        drone.vertical_velocity,
        0.0f
    ));

    assert(nearly_equal(
        drone.altitude,
        0.0f
    ));
}

static void test_zero_throttle_accelerates_downward(void)
{
    SimulatedDrone drone;

    simulated_drone_init(&drone, 0.0f);

    MotorOutputs motors = {
        .front_left = 0.0f,
        .front_right = 0.0f,
        .rear_right = 0.0f,
        .rear_left = 0.0f
    };

    simulated_drone_step_motors(
        &drone,
        motors,
        0.1f
    );

    assert(drone.vertical_velocity < 0.0f);
    assert(drone.altitude < 0.0f);
}

static void test_full_throttle_accelerates_upward(void)
{
    SimulatedDrone drone;

    simulated_drone_init(&drone, 0.0f);

    MotorOutputs motors = {
        .front_left = 1.0f,
        .front_right = 1.0f,
        .rear_right = 1.0f,
        .rear_left = 1.0f
    };

    simulated_drone_step_motors(
        &drone,
        motors,
        0.1f
    );

    assert(drone.vertical_velocity > 0.0f);
    assert(drone.altitude > 0.0f);
}

static void test_roll_command_preserves_hover_thrust(void)
{
    SimulatedDrone drone;

    simulated_drone_init(&drone, 0.0f);

    MotorOutputs motors =
        mixer_quad_x(
            0.5f,
            0.1f,
            0.0f,
            0.0f
        );

    simulated_drone_step_motors(
        &drone,
        motors,
        0.1f
    );

    assert(nearly_equal(
        drone.vertical_velocity,
        0.0f
    ));

    assert(nearly_equal(
        drone.altitude,
        0.0f
    ));

    assert(drone.roll_rate > 0.0f);
}

static void test_zero_dt_does_not_change_state(void)
{
    SimulatedDrone drone;

    simulated_drone_init(&drone, 15.0f);

    drone.roll_rate = 5.0f;
    drone.altitude = 10.0f;
    drone.vertical_velocity = 2.0f;

    MotorOutputs motors = {
        .front_left = 1.0f,
        .front_right = 0.0f,
        .rear_right = 0.0f,
        .rear_left = 1.0f
    };

    simulated_drone_step_motors(
        &drone,
        motors,
        0.0f
    );

    assert(nearly_equal(
        drone.roll_angle,
        15.0f
    ));

    assert(nearly_equal(
        drone.roll_rate,
        5.0f
    ));

    assert(nearly_equal(
        drone.altitude,
        10.0f
    ));

    assert(nearly_equal(
        drone.vertical_velocity,
        2.0f
    ));
}

int main(void)
{
    test_initialization();

    test_no_control_input();
    test_positive_control_input();

    test_level_imu();
    test_ninety_degree_roll_imu();
    test_negative_roll_sensor_signs();

    test_equal_motor_outputs_produce_no_roll();
    test_stronger_left_motors_produce_positive_roll();
    test_stronger_right_motors_produce_negative_roll();
    test_equal_motors_allow_roll_damping();

    test_hover_throttle_has_no_vertical_acceleration();
    test_zero_throttle_accelerates_downward();
    test_full_throttle_accelerates_upward();
    test_roll_command_preserves_hover_thrust();

    test_zero_dt_does_not_change_state();

    printf("All simulation tests passed.\n");

    return 0;
}