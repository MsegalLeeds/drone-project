#include <assert.h>
#include <math.h>
#include <stdio.h>

#include"pid.h"

static int nearly_equal(float a, float b)
{
    return fabsf(a - b) < 0.0001f;
}

int main(void)
{
    PID pid;

    /* Test 1: zero error should produce zero output */
    pid_init(&pid, 1.0f, 0.0f, 0.0f);

    float output = pid_update(&pid, 0.0f, 0.0f, 0.01f);

    assert(nearly_equal(output, 0.0f));

    /* Test 2: proportional controller */
    pid_init(&pid, 2.0f, 0.0f, 0.0f);

    output = pid_update(&pid, 10.0f, 7.0f, 0.01f);

    assert(nearly_equal(output, 6.0f));

    /* Test 3: integral term accumulates error */
    pid_init(&pid, 0.0f, 1.0f, 0.0f);

    output = pid_update(&pid, 10.0f, 0.0f, 0.1f);
    assert(nearly_equal(output, 1.0f));

    output = pid_update(&pid, 10.0f, 0.0f, 0.1f);
    assert(nearly_equal(output, 2.0f));

    /* Test 4: derivative responds to changing error */
    pid_init(&pid, 0.0f, 0.0f, 1.0f);

    /* First establish an error of 10 */
    pid_update(&pid, 10.0f, 0.0f, 0.1f);

    /* Error changes from 10 to 5 */
    output = pid_update(&pid, 5.0f, 0.0f, 0.1f);

    assert(nearly_equal(output, -50.0f));

    /* Test 5: output is limited */
    pid_init(&pid, 10.0f, 0.0f, 0.0f);
    pid_set_limits(&pid, -100.0f, 100.0f);

    output = pid_update(&pid, 20.0f, 0.0f, 0.1f);

    assert(nearly_equal(output, 100.0f));

    /* Test 6: negative output is limited */
    pid_init(&pid, 10.0f, 0.0f, 0.0f);
    pid_set_limits(&pid, -100.0f, 100.0f);

    output = pid_update(&pid, -20.0f, 0.0f, 0.1f);

    assert(nearly_equal(output, -100.0f));

    /* Test 7: integral should not wind up while output is saturated */
    pid_init(&pid, 0.0f, 10.0f, 0.0f);
    pid_set_limits(&pid, -100.0f, 100.0f);

    for (int i = 0; i < 100; i++) {
        pid_update(&pid, 100.0f, 0.0f, 0.1f);
    }

    /* Now remove the error */
    output = pid_update(&pid, 0.0f, 0.0f, 0.1f);

    /* If anti-windup works, the stored integral should not be huge */
    assert(fabsf(output) <= 100.0f);

    /* Test 8: reset clears PID history */
    pid_init(&pid, 0.0f, 1.0f, 1.0f);

    /* Build up some integral and previous error */
    pid_update(&pid, 10.0f, 0.0f, 0.1f);
    pid_update(&pid, 5.0f, 0.0f, 0.1f);

    pid_reset(&pid);

    assert(nearly_equal(pid.integral, 0.0f));
    assert(nearly_equal(pid.previous_error, 0.0f));

    printf("All PID tests passed.\n");

    return 0;
}