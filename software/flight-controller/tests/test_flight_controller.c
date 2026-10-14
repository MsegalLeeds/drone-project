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
        0.0f
    );

    ImuSample sample =
        imu_simulate(
            0.0f,
            0.0f
        );

    FlightControllerOutput output =
        flight_controller_update(
            &controller,
            &sample,
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
            output.roll_control,
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

    /*
     * Start with the estimator correctly knowing
     * that the drone is rolled +10 degrees.
     */
    flight_controller_init(
        &controller,
        10.0f,
        0.0f
    );

    ImuSample sample =
        imu_simulate(
            10.0f,
            0.0f
        );

    FlightControllerOutput output =
        flight_controller_update(
            &controller,
            &sample,
            0.0f,
            0.5f,
            0.01f
        );

    /*
     * Current roll is positive while target is zero,
     * so the controller should request negative roll.
     */
    assert(
        output.roll_control < 0.0f
    );

    /*
     * Negative roll correction increases the
     * right-hand motors relative to the left.
     */
    assert(
        output.motors.front_right >
        output.motors.front_left
    );

    assert(
        output.motors.rear_right >
        output.motors.rear_left
    );
}

static void test_gyro_offset_is_removed(void)
{
    FlightController controller;

    /*
     * Simulate a gyro with +1 degree/second bias.
     */
    flight_controller_init(
        &controller,
        0.0f,
        1.0f
    );

    ImuSample sample =
        imu_simulate(
            0.0f,
            0.0f
        );

    sample.gyro_x += 1.0f;

    FlightControllerOutput output =
        flight_controller_update(
            &controller,
            &sample,
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
            output.roll_control,
            0.0f
        )
    );
}

static void test_invalid_dt_produces_zero_output(void)
{
    FlightController controller;

    flight_controller_init(
        &controller,
        0.0f,
        0.0f
    );

    ImuSample sample =
        imu_simulate(
            0.0f,
            0.0f
        );

    FlightControllerOutput output =
        flight_controller_update(
            &controller,
            &sample,
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
    test_gyro_offset_is_removed();
    test_invalid_dt_produces_zero_output();

    printf(
        "Flight controller tests passed\n"
    );

    return 0;
}