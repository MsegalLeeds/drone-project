#include "replay.h"
#include <math.h>
#include <stdio.h>

// Shared by the Mac executable and RAM firmware. All motor values are numbers only.
static FlightController controller;
Checkpoint checkpoints[] = {{"level", {0}}, {"right", {0}}, {"returned", {0}}};

void replay_start(void) {
    flight_controller_init_3d(&controller, 0, 0, 0, 0, 0);
}

void replay_sample(unsigned i) {
    float angle = 0, rate = 0;
    if (i >= 20 && i < 40) {
        angle = (i - 19) * 0.5f;
        rate = 50;
    } else if (i >= 40 && i < 80) {
        angle = 10;
    } else if (i >= 80 && i < 100) {
        angle = (99 - i) * 0.5f;
        rate = -50;
    }
    ImuSample sample = imu_simulate_attitude_3d(angle, 0, rate, 0, 0);
    FlightControllerOutput out = flight_controller_update_3d(
        &controller, &sample, 0, 0, 0, 0.5f, PERIOD_US / 1000000.0f);
    if (i == 19) checkpoints[0].output = out;
    if (i == 79) checkpoints[1].output = out;
    if (i == SAMPLE_COUNT - 1) checkpoints[2].output = out;
}

bool replay_correct(void) {
    for (unsigned i = 0; i < 3; ++i) {
        const FlightControllerOutput *o = &checkpoints[i].output;
        float roll = i == 1 ? 10.0f : 0.0f;
        float left = i == 1 ? 0.25f : 0.5f;
        float right = 1.0f - left;
        if (!(fabsf(o->estimated_roll - roll) < 0.05f &&
              fabsf(o->estimated_pitch) < 0.05f &&
              fabsf(o->motors.front_left - left) < 0.002f &&
              fabsf(o->motors.rear_left - left) < 0.002f &&
              fabsf(o->motors.front_right - right) < 0.002f &&
              fabsf(o->motors.rear_right - right) < 0.002f)) return false;
    }
    return true;
}

int format_checkpoint(char *buf, unsigned n, const Checkpoint *p) {
    const FlightControllerOutput *o = &p->output;
    return snprintf(buf, n, "%s,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f\n",
        p->name, (double)o->estimated_roll, (double)o->estimated_pitch,
        (double)o->motors.front_left, (double)o->motors.front_right,
        (double)o->motors.rear_right, (double)o->motors.rear_left);
}
