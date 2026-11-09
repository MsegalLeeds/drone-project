// Bench-only interlock for simulated commands. A fresh low-to-high arm switch
// transition at low throttle is required after startup or a fault.
#include "replay.h"
#include <math.h>

typedef struct { bool armed, saw_switch_low; } BenchGate;
typedef struct {
    const ImuSample *sample;
    unsigned age_us, elapsed_us;
    bool radio_ok, arm_switch;
    float throttle;
} Inputs;

static bool allow_update(BenchGate *g, Inputs in) {
    const ImuSample *s = in.sample;
    bool healthy = in.radio_ok && s && in.age_us <= 2 * PERIOD_US &&
        in.elapsed_us > 0 && in.elapsed_us <= 2 * PERIOD_US &&
        isfinite(in.throttle) && in.throttle >= 0 && in.throttle <= 1 &&
        isfinite(s->gyro_x) && isfinite(s->gyro_y) && isfinite(s->gyro_z) &&
        isfinite(s->accel_x) && isfinite(s->accel_y) && isfinite(s->accel_z);
    if (!healthy || !in.arm_switch) {
        g->armed = false;
        g->saw_switch_low = healthy && !in.arm_switch;
    } else if (g->saw_switch_low) {
        g->armed = in.throttle <= 0.05f;
        g->saw_switch_low = false;
    }
    return g->armed;
}

const char *const fault_names[] = {"high-throttle", "missing-imu", "stale-imu",
    "invalid-reading", "missed-deadline", "radio-loss", "disarm", "restart"};

unsigned check_faults(void) {
    unsigned passed = 0;
    for (unsigned test = 0; test < 8; ++test) {
        ImuSample level = {.accel_z = 9.81f};
        Inputs in = {.sample=&level, .elapsed_us=PERIOD_US, .radio_ok=true};
        BenchGate gate = {0};
        if (allow_update(&gate, in)) continue;
        in.arm_switch = true;
        if (test == 0) {
            in.throttle = 0.7f;
            if (allow_update(&gate, in)) continue;
            in.throttle = 0;
            if (allow_update(&gate, in)) continue;
        } else {
            if (!allow_update(&gate, in)) continue;
            switch (test) {
            case 1: in.sample = NULL; break;
            case 2: in.age_us = 3 * PERIOD_US; break;
            case 3: level.gyro_y = NAN; break;
            case 4: in.elapsed_us = 3 * PERIOD_US; break;
            case 5: in.radio_ok = false; break;
            case 6: in.arm_switch = false; break;
            case 7: gate = (BenchGate){0}; break;
            }
            if (allow_update(&gate, in)) continue;
            level.gyro_y = 0;
            in = (Inputs){.sample=&level, .elapsed_us=PERIOD_US, .radio_ok=true,
                          .arm_switch=true};
            if (test != 6 && allow_update(&gate, in)) continue;
        }
        in.arm_switch = false;
        if (allow_update(&gate, in)) continue;
        in.arm_switch = true;
        if (allow_update(&gate, in)) passed |= 1u << test;
    }
    return passed;
}
