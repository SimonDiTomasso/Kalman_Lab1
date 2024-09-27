/*
 * KalmanFilter_C_CMSIS.c
 *
 *  Created on: Sep 23, 2024
 *      Author: simto
 */

#include "arm_math.h"  // CMSIS header for DSP functions
#include "KalmanFilter_C_CMSIS.h"

// Kalman filter update function
float32_t kalman_update_C_CMSIS(KalmanFilter_CMSIS *filter, float32_t measurement) {
    float32_t temp;  // Temporary variable for operations

    // Update the estimation error covariance: p = p + q
    arm_add_f32(&(filter->p), &(filter->q), &(filter->p), 1);

    // Calculate Kalman gain: k = p / (p + r)
    arm_add_f32(&(filter->p), &(filter->r), &temp, 1);
    filter->k = filter->p / temp;  // Division since CMSIS lacks arm_div_f32

    // Update the estimate: x = x + k * (measurement - x)
    arm_sub_f32(&measurement, &(filter->x), &temp, 1);  // temp = measurement - x
    arm_mult_f32(&(filter->k), &temp, &temp, 1);        // temp = k * (measurement - x)
    arm_add_f32(&(filter->x), &temp, &(filter->x), 1);  // x = x + temp

    // Update the estimation error covariance: p = (1 - k) * p
    temp = 1.0f - filter->k;
    arm_mult_f32(&(filter->p), &temp, &(filter->p), 1);

    // Return the updated estimate
    return filter->x;
}

