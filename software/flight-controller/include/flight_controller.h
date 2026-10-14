#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

#include "attitude.h"
#include "imu.h"
#include "mixer.h"
#include "pid.h"

typedef struct {
    PID roll_pid;
    AttitudeEstimator attitude;

    float gyro_x_offset;
} FlightController;

typedef struct {
    float estimated_roll;
    float roll_control;

    MotorOutputs motors;
} FlightControllerOutput;

/*
 * Initialise the roll flight controller.
 *
 * initial_roll_deg:
 *     Initial attitude estimate.
 *
 * gyro_x_offset:
 *     Calibrated gyro X bias in degrees/second.
 */
void flight_controller_init(
    FlightController *controller,
    float initial_roll_deg,
    float gyro_x_offset
);

/*
 * Run one iteration of the flight-control loop.
 *
 * Pipeline:
 *
 * IMU
 *  -> attitude estimator
 *  -> roll PID
 *  -> motor mixer
 */
FlightControllerOutput flight_controller_update(
    FlightController *controller,
    const ImuSample *sample,
    float target_roll_deg,
    float throttle,
    float dt
);

#endif