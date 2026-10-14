#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "attitude.h"

static int nearly_equal(
    float a,
    float b
)
{
    return fabsf(a - b) < 0.5f;
}

static void test_level(void)
{
    AttitudeEstimator estimator;

    attitude_init_2d(
        &estimator,
        0.0f,
        0.0f
    );

    attitude_update_2d(
        &estimator,
        0.0f,
        0.0f,
        1.0f,
        0.0f,
        0.0f,
        0.01f
    );

    assert(
        nearly_equal(
            estimator.roll,
            0.0f
        )
    );

    assert(
        nearly_equal(
            estimator.pitch,
            0.0f
        )
    );
}

static void test_roll_30_degrees(void)
{
    AttitudeEstimator estimator;

    float angle = 30.0f;
    float radians =
        angle * 0.017453292519943295f;

    attitude_init_2d(
        &estimator,
        30.0f,
        0.0f
    );

    attitude_update_2d(
        &estimator,
        0.0f,
        sinf(radians),
        cosf(radians),
        0.0f,
        0.0f,
        0.01f
    );

    assert(
        nearly_equal(
            estimator.roll,
            30.0f
        )
    );
}

static void test_pitch_30_degrees(void)
{
    AttitudeEstimator estimator;

    float angle = 30.0f;
    float radians =
        angle * 0.017453292519943295f;

    attitude_init_2d(
        &estimator,
        0.0f,
        30.0f
    );

    attitude_update_2d(
        &estimator,
        -sinf(radians),
        0.0f,
        cosf(radians),
        0.0f,
        0.0f,
        0.01f
    );

    assert(
        nearly_equal(
            estimator.pitch,
            30.0f
        )
    );
}

static void test_combined_roll_and_pitch(void)
{
    const float roll_deg = 20.0f;
    const float pitch_deg = -15.0f;

    const float deg_to_rad =
        0.017453292519943295f;

    float roll =
        roll_deg * deg_to_rad;

    float pitch =
        pitch_deg * deg_to_rad;

    float accel_x =
        -sinf(pitch);

    float accel_y =
        sinf(roll) *
        cosf(pitch);

    float accel_z =
        cosf(roll) *
        cosf(pitch);

    AttitudeEstimator estimator;

    attitude_init_2d(
        &estimator,
        roll_deg,
        pitch_deg
    );

    attitude_update_2d(
        &estimator,
        accel_x,
        accel_y,
        accel_z,
        0.0f,
        0.0f,
        0.01f
    );

    assert(
        nearly_equal(
            estimator.roll,
            roll_deg
        )
    );

    assert(
        nearly_equal(
            estimator.pitch,
            pitch_deg
        )
    );
}

int main(void)
{
    test_level();
    test_roll_30_degrees();
    test_pitch_30_degrees();
    test_combined_roll_and_pitch();

    printf(
        "All attitude tests passed.\n"
    );

    return 0;
}