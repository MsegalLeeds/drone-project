#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "mixer.h"

#define EPSILON 0.0001f

static int nearly_equal(float a, float b)
{
    return fabsf(a - b) < EPSILON;
}

static void test_hover(void)
{
    MotorOutputs motors =
        mixer_quad_x(0.5f, 0.0f, 0.0f, 0.0f);

    assert(nearly_equal(motors.front_left, 0.5f));
    assert(nearly_equal(motors.front_right, 0.5f));
    assert(nearly_equal(motors.rear_right, 0.5f));
    assert(nearly_equal(motors.rear_left, 0.5f));
}

static void test_positive_roll(void)
{
    MotorOutputs motors =
        mixer_quad_x(0.5f, 0.1f, 0.0f, 0.0f);

    assert(nearly_equal(motors.front_left, 0.6f));
    assert(nearly_equal(motors.rear_left, 0.6f));

    assert(nearly_equal(motors.front_right, 0.4f));
    assert(nearly_equal(motors.rear_right, 0.4f));
}

static void test_positive_pitch(void)
{
    MotorOutputs motors =
        mixer_quad_x(0.5f, 0.0f, 0.1f, 0.0f);

    assert(nearly_equal(motors.front_left, 0.4f));
    assert(nearly_equal(motors.front_right, 0.4f));

    assert(nearly_equal(motors.rear_right, 0.6f));
    assert(nearly_equal(motors.rear_left, 0.6f));
}

static void test_positive_yaw(void)
{
    MotorOutputs motors =
        mixer_quad_x(0.5f, 0.0f, 0.0f, 0.1f);

    /*
     * Positive yaw increases the CW motors:
     * front-right and rear-left.
     */
    assert(nearly_equal(motors.front_left, 0.4f));
    assert(nearly_equal(motors.front_right, 0.6f));
    assert(nearly_equal(motors.rear_right, 0.4f));
    assert(nearly_equal(motors.rear_left, 0.6f));
}

static void test_upper_saturation(void)
{
    MotorOutputs motors =
        mixer_quad_x(0.9f, 0.5f, 0.5f, 0.5f);

    assert(motors.front_left <= 1.0f);
    assert(motors.front_right <= 1.0f);
    assert(motors.rear_right <= 1.0f);
    assert(motors.rear_left <= 1.0f);
}

static void test_lower_saturation(void)
{
    MotorOutputs motors =
        mixer_quad_x(0.1f, -0.5f, 0.5f, 0.5f);

    assert(motors.front_left >= 0.0f);
    assert(motors.front_right >= 0.0f);
    assert(motors.rear_right >= 0.0f);
    assert(motors.rear_left >= 0.0f);
}

int main(void)
{
    test_hover();
    test_positive_roll();
    test_positive_pitch();
    test_positive_yaw();
    test_upper_saturation();
    test_lower_saturation();

    printf("Mixer tests passed\n");

    return 0;
}