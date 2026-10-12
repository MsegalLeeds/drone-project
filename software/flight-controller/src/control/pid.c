#include "pid.h"

void pid_init(
    PID *pid,
    float kp,
    float ki,
    float kd
)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->output_min = -1000000.0f;
    pid->output_max = 1000000.0f;

    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
}

void pid_set_limits(
    PID *pid,
    float output_min,
    float output_max
)
{
    pid->output_min = output_min;
    pid->output_max = output_max;
}

void pid_reset(PID *pid)
{
    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
}

float pid_update(
    PID *pid,
    float setpoint,
    float measurement,
    float dt
)
{
    float error = setpoint - measurement;

    float derivative = 0.0f;

    if (dt > 0.0f) {
        derivative =
            (error - pid->previous_error) / dt;
    }

    float proposed_integral =
        pid->integral + error * dt;

    float output =
        pid->kp * error +
        pid->ki * proposed_integral +
        pid->kd * derivative;

    /*
     * Anti-windup:
     * only accept the new integral if the controller
     * is not trying to push further into saturation.
     */
    if (output <= pid->output_max &&
        output >= pid->output_min) {

        pid->integral = proposed_integral;
        }
    else if (output > pid->output_max &&
             error < 0.0f) {

        pid->integral = proposed_integral;
             }
    else if (output < pid->output_min &&
             error > 0.0f) {

        pid->integral = proposed_integral;
             }

    /*
     * Recalculate the output using the integral
     * value that was actually accepted.
     */
    output =
        pid->kp * error +
        pid->ki * pid->integral +
        pid->kd * derivative;

    /* Clamp output */
    if (output > pid->output_max) {
        output = pid->output_max;
    }

    if (output < pid->output_min) {
        output = pid->output_min;
    }

    pid->previous_error = error;

    return output;
}