#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "flight_controller.h"
#include "imu.h"

#define EPSILON 0.0001f

static int nearly_equal(
    float a,
    float b
)
{
    return fabsf(a - b) < EPSILON;
}

static void test_level_hover(void)
{
    FlightController controller;

    flight_controller_init(
        &controller,
        0.0f,
        0.0f,
        0.0f,
        0.0f
    );

    ImuSample sample =
        imu_simulate_attitude(
            0.0f,
            0.0f,
            0.0f,
            0.0f
        );

    FlightControllerOutput output =
        flight_controller_update(
            &controller,
            &sample,
            0.0f,
            0.0f,
            0.5f,
            0.01f
        );

    assert(
        nearly_equal(
            output.estimated_roll,
            0.0f
        )
    );

    assert(
        nearly_equal(
            output.estimated_pitch,
            0.0f
        )
    );

    assert(
        nearly_equal(
            output.roll_control,
            0.0f
        )
    );

    assert(
        nearly_equal(
            output.pitch_control,
            0.0f
        )
    );

    assert(
        nearly_equal(
            output.motors.front_left,
            0.5f
        )
    );

    assert(
        nearly_equal(
            output.motors.front_right,
            0.5f
        )
    );

    assert(
        nearly_equal(
            output.motors.rear_right,
            0.5f
        )
    );

    assert(
        nearly_equal(
            output.motors.rear_left,
            0.5f
        )
    );
}

static void test_positive_roll_generates_correction(void)
{
    FlightController controller;

    flight_controller_init(
        &controller,
        10.0f,
        0.0f,
        0.0f,
        0.0f
    );

    ImuSample sample =
        imu_simulate_attitude(
            10.0f,
            0.0f,
            0.0f,
            0.0f
        );

    FlightControllerOutput output =
        flight_controller_update(
            &controller,
            &sample,
            0.0f,
            0.0f,
            0.5f,
            0.01f
        );

    assert(output.roll_control < 0.0f);

    assert(
        output.motors.front_right >
        output.motors.front_left
    );

    assert(
        output.motors.rear_right >
        output.motors.rear_left
    );
}

static void test_positive_pitch_generates_correction(void)
{
    FlightController controller;

    flight_controller_init(
        &controller,
        0.0f,
        10.0f,
        0.0f,
        0.0f
    );

    ImuSample sample =
        imu_simulate_attitude(
            0.0f,
            10.0f,
            0.0f,
            0.0f
        );

    FlightControllerOutput output =
        flight_controller_update(
            &controller,
            &sample,
            0.0f,
            0.0f,
            0.5f,
            0.01f
        );

    /*
     * Positive pitch must result in a
     * negative corrective command.
     */
    assert(output.pitch_control < 0.0f);

    /*
     * Negative pitch correction raises the
     * front motors relative to the rear.
     */
    assert(
        output.motors.front_left >
        output.motors.rear_left
    );

    assert(
        output.motors.front_right >
        output.motors.rear_right
    );
}

static void test_gyro_offsets_are_removed(void)
{
    FlightController controller;

    flight_controller_init(
        &controller,
        0.0f,
        0.0f,
        1.0f,
        -2.0f
    );

    ImuSample sample =
        imu_simulate_attitude(
            0.0f,
            0.0f,
            0.0f,
            0.0f
        );

    sample.gyro_x += 1.0f;
    sample.gyro_y -= 2.0f;

    FlightControllerOutput output =
        flight_controller_update(
            &controller,
            &sample,
            0.0f,
            0.0f,
            0.5f,
            0.01f
        );

    assert(
        nearly_equal(
            output.estimated_roll,
            0.0f
        )
    );

    assert(
        nearly_equal(
            output.estimated_pitch,
            0.0f
        )
    );

    assert(
        nearly_equal(
            output.roll_control,
            0.0f
        )
    );

    assert(
        nearly_equal(
            output.pitch_control,
            0.0f
        )
    );
}

static void test_positive_yaw_rate_generates_correction(void)
{
    FlightController controller;

    flight_controller_init_3d(
        &controller,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f
    );

    ImuSample sample =
        imu_simulate_attitude_3d(
            0.0f,
            0.0f,
            0.0f,
            0.0f,
            0.0f
        );

    FlightControllerOutput output =
        flight_controller_update_3d(
            &controller,
            &sample,
            0.0f,
            0.0f,
            45.0f,
            0.5f,
            0.01f
        );

    assert(output.yaw_control > 0.0f);

    /*
     * Positive yaw increases the CW motors:
     * front-right and rear-left.
     */
    assert(
        output.motors.front_right >
        output.motors.front_left
    );

    assert(
        output.motors.rear_left >
        output.motors.rear_right
    );
}

static void test_invalid_dt_produces_zero_output(void)
{
    FlightController controller;

    flight_controller_init(
        &controller,
        0.0f,
        0.0f,
        0.0f,
        0.0f
    );

    ImuSample sample =
        imu_simulate_attitude(
            0.0f,
            0.0f,
            0.0f,
            0.0f
        );

    FlightControllerOutput output =
        flight_controller_update(
            &controller,
            &sample,
            0.0f,
            0.0f,
            0.5f,
            0.0f
        );

    assert(
        nearly_equal(
            output.motors.front_left,
            0.0f
        )
    );

    assert(
        nearly_equal(
            output.motors.front_right,
            0.0f
        )
    );
}

int main(void)
{
    test_level_hover();
    test_positive_roll_generates_correction();
    test_positive_pitch_generates_correction();
    test_gyro_offsets_are_removed();
    test_positive_yaw_rate_generates_correction();
    test_invalid_dt_produces_zero_output();

    printf(
        "Flight controller tests passed\n"
    );

    return 0;
}