#ifndef MIXER_H
#define MIXER_H

/*
 * Quad-X motor layout:
 *
 *              FRONT
 *
 *        M1             M2
 *   Front Left     Front Right
 *      CCW              CW
 *
 *
 *        M4             M3
 *    Rear Left      Rear Right
 *       CW              CCW
 *
 * Motor outputs are normalized to the range 0.0f - 1.0f.
 */
typedef struct {
    float front_left;
    float front_right;
    float rear_right;
    float rear_left;
} MotorOutputs;

/*
 * Mix throttle and axis corrections into four motor outputs.
 *
 * throttle: 0.0 - 1.0
 * roll:    -1.0 - 1.0
 * pitch:   -1.0 - 1.0
 * yaw:     -1.0 - 1.0
 */
MotorOutputs mixer_quad_x(
    float throttle,
    float roll,
    float pitch,
    float yaw
);

#endif