#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

#include "attitude.h"
#include "imu.h"
#include "mixer.h"
#include "pid.h"

typedef struct {
    PID roll_pid;
    PID pitch_pid;
    PID yaw_rate_pid;

    AttitudeEstimator attitude;

    float gyro_x_offset;
    float gyro_y_offset;
    float gyro_z_offset;
} FlightController;

typedef struct {
    float estimated_roll;
    float estimated_pitch;

    float measured_yaw_rate;

    float roll_control;
    float pitch_control;
    float yaw_control;

    MotorOutputs motors;
} FlightControllerOutput;

void flight_controller_init(
    FlightController *controller,
    float initial_roll_deg,
    float initial_pitch_deg,
    float gyro_x_offset,
    float gyro_y_offset
);

void flight_controller_init_3d(
    FlightController *controller,
    float initial_roll_deg,
    float initial_pitch_deg,
    float gyro_x_offset,
    float gyro_y_offset,
    float gyro_z_offset
);

FlightControllerOutput flight_controller_update(
    FlightController *controller,
    const ImuSample *sample,
    float target_roll_deg,
    float target_pitch_deg,
    float throttle,
    float dt
);

FlightControllerOutput flight_controller_update_3d(
    FlightController *controller,
    const ImuSample *sample,
    float target_roll_deg,
    float target_pitch_deg,
    float target_yaw_rate_deg_s,
    float throttle,
    float dt
);

#endif