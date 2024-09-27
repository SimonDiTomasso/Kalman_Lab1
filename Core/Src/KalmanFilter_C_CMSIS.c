/*
 * KalmanFilter_C_CMSIS.c
 *
 *  Created on: Sep 23, 2024
 *      Author: simto
 */

#include "arm_math.h"  // CMSIS header for DSP functions

// Structure to hold Kalman filter state
typedef struct {
    float32_t q;  // Process noise covariance
    float32_t r;  // Measurement noise covariance
    float32_t p;  // Estimation error covariance
    float32_t k;  // Kalman gain
    float32_t x;  // Value (the filtered result)
} KalmanFilter;

// Kalman filter initialization function
void kalman_init(KalmanFilter *filter, float32_t q, float32_t r, float32_t p, float32_t k, float32_t x) {
    filter->q = q;            // Set process noise covariance
    filter->r = r;            // Set measurement noise covariance
    filter->p = p;         // Initial estimation error covariance
    filter->k = k;         // Initial Kalman gain
    filter->x = x; // Initial value
}

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

// Example main function
int main() {
    KalmanFilter filter;

    // Initialize the Kalman filter with q = 0.1, r = 5, and initial value = 10
    kalman_init(&filter, 0.1f, 5.0f, 10.0f);

    // Array of measurements (all float)
    float32_t measurements[6] = {0.0f, 1.0f, 2.0f, 3.0f, 4.0f, 5.0f};
    float32_t updated_value;

    // Loop to update the Kalman filter with 6 measurements
    for (int i = 0; i < 6; i++) {
        updated_value = kalman_update(&filter, measurements[i]);
        printf("Updated value after measurement %d: %f\n", i, updated_value);
    }

    return 0;
}

