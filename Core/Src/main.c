/* USER CODE BEGIN Header */
/**
  ****************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ****************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ****************************************************************************
  */


/* This main.c is used to call the other c functions to keep it this project cleaner, this used to calculate the time*/




/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define ARM_MATH_CM4
#include "arm_math.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* ----------------------------------------------------------------------
* Defines each of the tests performed
* ------------------------------------------------------------------- */
#define MAX_BLOCKSIZE   32
#define DELTA           (0.000001f)

/* ----------------------------------------------------------------------
* Test input data for Floating point Dot Product example for 32-blockSize
* Generated by the MATLAB randn() function
* ------------------------------------------------------------------- */
/* ----------------------------------------------------------------------
** Test input data of srcA for blockSize 32
** ------------------------------------------------------------------- */
float32_t srcA_buf_f32[MAX_BLOCKSIZE] =
{
-0.4325648115282207,    -1.6655843782380970,    0.1253323064748307,
 0.2876764203585489,    -1.1464713506814637,    1.1909154656429988,
 1.1891642016521031,    -0.0376332765933176,    0.3272923614086541,
 0.1746391428209245,    -0.1867085776814394,    0.7257905482933027,
-0.5883165430141887,     2.1831858181971011,   -0.1363958830865957,
 0.1139313135208096,     1.0667682113591888,    0.0592814605236053,
-0.0956484054836690,    -0.8323494636500225,    0.2944108163926404,
-1.3361818579378040,     0.7143245518189522,    1.6235620644462707,
-0.6917757017022868,     0.8579966728282626,    1.2540014216025324,
-1.5937295764474768,    -1.4409644319010200,    0.5711476236581780,
-0.3998855777153632,     0.6899973754643451
};
/* ----------------------------------------------------------------------
** Test input data of srcB for blockSize 32
** ------------------------------------------------------------------- */
float32_t srcB_buf_f32[MAX_BLOCKSIZE] =
{
 1.7491401329284098,    0.1325982188803279,      0.3252281811989881,
-0.7938091410349637,    0.3149236145048914,     -0.5272704888029532,
 0.9322666565031119,    1.1646643544607362,     -2.0456694357357357,
-0.6443728590041911,    1.7410657940825480,      0.4867684246821860,
 1.0488288293660140,    1.4885752747099299,      1.2705014969484090,
-1.8561241921210170,    2.1343209047321410,  1.4358467535865909,
-0.9173023332875400,   -1.1060770780029008,      0.8105708062681296,
 0.6985430696369063,   -0.4015827425012831,      1.2687512030669628,
-0.7836083053674872,    0.2132664971465569,      0.7878984786088954,
 0.8966819356782295,   -0.1869172943544062,      1.0131816724341454,
 0.2484350696132857,    0.0596083377937976
};

/* Reference dot product output */
float32_t  refDotProdOut = 5.9273644806352142;

/* ----------------------------------------------------------------------
* Declare Global variables
* ------------------------------------------------------------------- */
float32_t multOutput[MAX_BLOCKSIZE];  /* Intermediate output */
float32_t testOutput;  /* Final ouput */

arm_status status;       /* Status of the example */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ITM_Port32(n) (*((volatile unsigned long *) (0xE0000000+4*n)))


float PCSA_test(float* a,uint32_t b){
	uint32_t c;
	c= a[0]+b;
	return c;
}
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */


void kalmanInit(KalmanFilter *KalmanFilter, float q, float r, float x, float p, float k){
	  KalmanFilter->q = q;
	  KalmanFilter->r = r;
	  KalmanFilter->x = x;
	  KalmanFilter->p = p;
	  KalmanFilter->k = k;

}



int main(void){
	
	KalmanFilter kf;

  /* USER CODE BEGIN 1 */
	kalmanInit(&kf, 0.1, 0.1, 0.1, 5.0, 0.0);

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */


	  // call kalman assembly function
	  // r0 = pointer to KalmanFilter struct
	  //s0 = measurement (float)


  	float meas[5] = {0.0, 1.0, 2.0, 3.0, 4.0};




	 for(int i = 0; i < 5; i++) {
		  kalman_update(kf, meas[i]);
	      }
	 	 return 0;










  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */
//   uint32_t i;                      /* Loop counter */
//   float32_t diff;          /* Difference between reference and test outputs */

//   /* Multiplication of two input buffers */
//   arm_mult_f32(srcA_buf_f32, srcB_buf_f32, multOutput, MAX_BLOCKSIZE);
//
//   /* Accumulate the multiplication output values to
//      get the dot product of the two inputs */
//  for(int i =0; i< MAX_BLOCKSIZE; i++)
// {
//           arm_add_f32(&testOutput, &multOutput[i], &testOutput, 1);
// }
//
//  /* absolute value of difference between ref and test */
//  ITM_Port32(31) = 1;
//  //It is better for 1000 execution
//  diff = fabsf(refDotProdOut - testOutput);
//  ITM_Port32(31) = 2;
//




  /* Comparison of dot product value with reference */
//         if(diff > DELTA)
//         {
//                 status = ARM_MATH_TEST_FAILURE;
//         }
//
//         if( status == ARM_MATH_TEST_FAILURE)
//         {
//           while(1);
//         }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

//    float out1=0;
//	float array[1] = {5};
//			  out1=PCSA_test(&array,2);
//
//			  ITM_Port32(31) = 1;
//
//         func(&array, 8);
//
//   	  ITM_Port32(31) = 2;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
