#ifndef PID_H
#define PID_H

typedef struct {
    float kp;
    float ki;
    float kd;

    float integral;
    float previous_error;

    float output_min;
    float output_max;
} PID;

void pid_init(
    PID *pid,
    float kp,
    float ki,
    float kd
);

void pid_reset(PID *pid);

float pid_update(
    PID *pid,
    float setpoint,
    float measurement,
    float dt
);

void pid_set_limits(
    PID *pid,
    float output_min,
    float output_max
);

#endif