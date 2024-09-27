/*
 * KalmanFilter_C_CMSIS.c
 *
 *  Created on: Sep 23, 2024
 *      Author: simto
 */

#include "arm_math.h"  // CMSIS header for DSP functions
#include "KalmanFilter_C_CMSIS.h"

// Kalman filter update function
float32_t kalman_update_C_CMSIS(KalmanFilter *filter, float32_t measurement) {
    // Temporary variables for CMSIS-DSP operations
    float32_t srcA[1], srcB[1], dst[1];

    // Update the estimation error covariance: p = p + q
    srcA[0] = filter->p;
    srcB[0] = filter->q;
    arm_add_f32(srcA, srcB, dst, 1);
    filter->p = dst[0];

    // Calculate Kalman gain: k = p / (p + r)
    srcA[0] = filter->p;
    srcB[0] = filter->r;
    arm_add_f32(srcA, srcB, dst, 1);
    filter->k = filter->p / dst[0];  // Direct division since CMSIS lacks arm_div_f32

    // Update the estimate: x = x + k * (measurement - x)
    srcA[0] = measurement;
    srcB[0] = filter->x;
    arm_sub_f32(srcA, srcB, dst, 1);  // dst[0] = measurement - x
    srcA[0] = filter->k;
    arm_mult_f32(srcA, dst, dst, 1);  // dst[0] = k * (measurement - x)
    srcA[0] = filter->x;
    arm_add_f32(srcA, dst, dst, 1);   // dst[0] = x + k * (measurement - x)
    filter->x = dst[0];

    // Update the estimation error covariance: p = (1 - k) * p
    srcA[0] = 1.0f;
    srcB[0] = filter->k;
    arm_sub_f32(srcA, srcB, dst, 1);  // dst[0] = 1 - k
    srcA[0] = filter->p;
    arm_mult_f32(srcA, dst, dst, 1);  // dst[0] = (1 - k) * p
    filter->p = dst[0];

    // Return the updated estimate
    return filter->x;
}

