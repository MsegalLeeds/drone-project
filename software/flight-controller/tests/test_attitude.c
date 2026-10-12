#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "attitude.h"

static int nearly_equal(float a, float b)
{
    return fabsf(a - b) < 0.5f;
}

int main(void)
{
    AttitudeEstimator estimator;

    /*
     * Test 1:
     * Level accelerometer should estimate approximately 0 degrees.
     */
    attitude_init(&estimator, 0.0f);

    float roll = attitude_update(
        &estimator,
        0.0f,
        1.0f,
        0.0f,
        0.01f
    );

    assert(nearly_equal(roll, 0.0f));


    /*
     * Test 2:
     * Stationary accelerometer tilted to 30 degrees
     * should pull the estimate toward 30 degrees.
     */
    attitude_init(&estimator, 30.0f);

    const float angle = 30.0f;
    const float radians = angle * 0.017453292519943295f;

    roll = attitude_update(
        &estimator,
        sinf(radians),
        cosf(radians),
        0.0f,
        0.01f
    );

    assert(nearly_equal(roll, 30.0f));

    printf("All attitude tests passed.\n");

    return 0;
}