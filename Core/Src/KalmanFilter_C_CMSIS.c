/*
 * KalmanFilter_C_CMSIS.c
 *
 *  Created on: Sep 23, 2024
 *      Author: simto
 */

#include "arm_math.h"  // CMSIS header for DSP functions

// Kalman filter update function
float32_t kalman_update(KalmanFilter *filter, float32_t measurement) {
    // Update the estimation error covariance: p = p + q
    filter->p += filter->q;

    // Calculate Kalman gain: k = p / (p + r)
    float32_t denominator;
    arm_add_f32(&filter->p, &filter->r, &denominator);  // denominator = p + r
    arm_div_f32(&filter->p, &denominator, &filter->k);  // k = p / denominator

    // Update the estimate: x = x + k * (measurement - x)
    float32_t temp1, temp2;
    arm_sub_f32(&measurement, &filter->x, &temp1);  // temp1 = measurement - x
    arm_mult_f32(&filter->k, &temp1, &temp2);       // temp2 = k * (measurement - x)
    arm_add_f32(&filter->x, &temp2, &filter->x);    // x = x + temp2

    // Update the estimation error covariance: p = (1 - k) * p
    float32_t one_minus_k;
    float32_t one = 1.0f;
    arm_sub_f32(&one, &filter->k, &one_minus_k);  // one_minus_k = 1 - k
    arm_mult_f32(&one_minus_k, &filter->p, &filter->p);  // p = one_minus_k * p

    // Return the updated estimate
    return filter->x;
}

