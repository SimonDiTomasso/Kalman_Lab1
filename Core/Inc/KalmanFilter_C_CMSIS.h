/*
 * KalmanFilter_C_CMSIS.h
 *
 *  Created on: Sep 23, 2024
 *      Author: simto
 */

#ifndef INC_KALMANFILTER_C_CMSIS_H_
#define INC_KALMANFILTER_C_CMSIS_H_



#endif /* INC_KALMANFILTER_C_CMSIS_H_ */

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>  // Standard input/output library for printf
#include "arm_math.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct {
	  float32_t q;
	  float32_t r;
	  float32_t x;
	  float32_t p;
	  float32_t k;
} KalmanFilter_CMSIS;

float kalman_update_C_CMSIS(KalmanFilter_CMSIS *KalmanFilter, float32_t measurement);

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif
