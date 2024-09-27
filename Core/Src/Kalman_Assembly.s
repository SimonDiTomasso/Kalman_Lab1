/*
 * Kalman_Assembly.s
 *
 *  Created on: Sep 23, 2024
 *      Author: simto
 */
.syntax unified
    .thumb
    .thumb_func
    .global kalman_update

kalman_update:
    ; Input:
    # r0 = pointer to KalmanFilter struct
    # s0 = measurement (float)

    # Load KalmanFilter struct values into floating-point registers
    vldr s1, [r0, #0x00]    //#Load q (process noise covariance) into s1
    vldr s2, [r0, #0x04]    //# Load r (measurement noise covariance) into s2
    vldr s3, [r0, #0x08]   // # Load x (estimated value) into s3
    vldr s4, [r0, #0x0C]    //# Load p (estimation error covariance) into s4

    # p = p + q
    vadd.f32 s4, s4, s1     //# s4 = p + q (update p with process noise covariance)

    # k = p / (p + r)
    vadd.f32 s5, s4, s2    // # s5 = p + r
    vdiv.f32 s6, s4, s5    // # s6 = k = p / (p + r) (calculate Kalman gain)

    # x = x + k * (measurement - x)
    vsub.f32 s7, s0, s3    // # s7 = measurement - x
    vmul.f32 s7, s7, s6    // # s7 = k * (measurement - x)
    vadd.f32 s3, s3, s7     //# s3 = x + k * (measurement - x) (update estimate)

    # p = (1 - k) * p
    vmov.f32 s7, #1.0      // # s7 = 1.0
    vsub.f32 s7, s7, s6    // # s7 = 1 - k
    vmul.f32 s4, s4, s7    // # s4 = (1 - k) * p (update error covariance)

    # Store the updated values back into the KalmanFilter struct
    vstr s3, [r0, #0x08]    //# Store updated x
    vstr s4, [r0, #0x0C]   // # Store updated p

    bx lr                 //   # Return from function

