/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "My_Driver.h"
#include "FOC_Math.h"
#include "FOC_Transforms.h"
#include "FOC_Svpwm.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// user debug variables
#define PHASE_SHIFT_120   2.0943951f
#define  STEP_UNIT_20KHZ   0.000314
float  Target_Freq =  10; 
float theta_a;
float theta_b;
float theta_c;

SinCos_t sc_a;
SinCos_t sc_b;
SinCos_t sc_c;


Clark_t ABC_To_AlphaBeta;
Park_t  AlphaBeta_To_DQ;
Ip_t    UdUq_To_UalphaUbeta;
SVPWM_t SVPWM;

float sin_value;
float cos_value;
float theta;
// user debug variables

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
int main(void)
{

  /* USER CODE BEGIN 1 */
	theta = 0;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_SPI3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	DRV8301_Init();
//  MySystemInit();
	HAL_TIM_Base_Start(&htim1);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_3);	
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,2000);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,2000);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,2000);
	
//		HAL_TIM_Base_Start(&htim1);
//	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
//	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
//	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
//	
//	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,2000);
//	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,2000);
//	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,2000);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
		theta += Target_Freq*STEP_UNIT_20KHZ ;
		if(theta >= 6.28f) theta -= 6.28f;
		theta_a = theta;
		AL_fastSinCos(theta_a, &sc_a);
		
		
		theta_b = theta - PHASE_SHIFT_120;
		if(theta_b < 0.0f)
		{
			theta_b += 6.28f;
		}
		AL_fastSinCos(theta_b, &sc_b);
		
		theta_c = theta + PHASE_SHIFT_120;
		if(theta_c > 6.28f)
		{
			theta_c -= 6.28f;
		}
		AL_fastSinCos(theta_c, &sc_c);
		
		ABC_To_AlphaBeta.I_a = sc_a.s;
		ABC_To_AlphaBeta.I_b = sc_b.s;
		ABC_To_AlphaBeta.I_c = sc_c.s;
		AL_Clark(& ABC_To_AlphaBeta);
		
		AlphaBeta_To_DQ.I_alpha = ABC_To_AlphaBeta.I_alpha;
		AlphaBeta_To_DQ.I_beta = ABC_To_AlphaBeta.I_beta;
		AlphaBeta_To_DQ.theta = theta;
		
		AL_Park(&AlphaBeta_To_DQ,sc_a.s,sc_a.c);
		//省略PI控制器
		UdUq_To_UalphaUbeta.d = 0;
		UdUq_To_UalphaUbeta.q = 1;
		UdUq_To_UalphaUbeta.theta = theta;
		
		AL_InvPark(&UdUq_To_UalphaUbeta,sc_a.s,sc_a.c);
		
		SVPWM.T_pwm = 8400;
		SVPWM.U_alpha = UdUq_To_UalphaUbeta.U_alpha;
		SVPWM.U_beta = UdUq_To_UalphaUbeta.U_beta;
		SVPWM.U_dc = 24;
		
		AL_SVPWM_Calculate(&SVPWM);
		for(volatile uint32_t i = 0;i<2000;i++);
		
		
		
		
		
		

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
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
#ifdef USE_FULL_ASSERT
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
