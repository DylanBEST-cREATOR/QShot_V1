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
#include "FOC_Math.h"
#include "FOC_Transforms.h"
#include "FOC_Svpwm.h"
#include "FOC_PID.h"
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
 SinCos_t sc_a;
SinCos_t sc_b;
   SinCos_t sc_c;
  uint16_t theta;
 Current_Sense_t current;
  Clarke_t Clark;
  Park_t   Park;
	Park_t   Ipark;
	Clarke_t  Clark2;
	SVPWM_t   SVPWM_Components;
	
	volatile PID_t my_pid;
volatile q16_t test_ref = 0;    // J-Scope 手动修改的目标给定值
volatile q16_t test_fdb = 0;    // 虚拟电机的反馈值
volatile uint32_t step_timer = 0; // 阶跃信号生成器
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
	current.Ia = 0;
	current.Ib = 0;
	current.Ic = 0;
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

  /* USER CODE BEGIN 2 */


//  MySystemInit();

my_pid.Kp = 32768;      // Kp = 0.5 PU (0.5 * 65536)
my_pid.Ki = 6553;        // Ki = 0.01 PU (0.01 * 65536) - 积分先给小一点
my_pid.Kd = 0;          // Kd = 0
my_pid.Out_Max = 65536; // 限幅在 1.0 PU
my_pid.Out_Min = -65536;// 限幅在 -1.0 PU
my_pid.Integral = 0;
my_pid.Last_Fdb = 0;
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
/* USER CODE END WHILE */

    // 1. 计算三相的 16 位电角度（依靠 uint16_t 自动实现多圈回绕，angle 永远不会越界）
    uint16_t theta_a = theta;
    uint16_t theta_b = theta - 21845; // 减去 120 度
    uint16_t theta_c = theta + 21845; // 加上 120 度
    
    // 2. 转换成三角函数要求的 Q16 标幺输入（0 ~ 131072 代表 0 ~ 2pi）
    // 左移 1 位（相当于乘以 2），将 0~65535 映射到 0~131070
    q16_t angle_a = (q16_t)theta_a << 1;
    q16_t angle_b = (q16_t)theta_b << 1;
    q16_t angle_c = (q16_t)theta_c << 1;

    // 3. 运行定点快速正余弦函数
    FM_fastSinCos(angle_a, &sc_a);
    FM_fastSinCos(angle_b, &sc_b);
    FM_fastSinCos(angle_c, &sc_c);

    // 4. 角度递增（步长 100 左右波形就很平滑了）
    theta += 100;
		

		current.Ia = sc_a.s;
		current.Ib = sc_b.s;
		current.Ic = sc_c.s;
		
		FT_Clarke_Transform(&current, &Clark);
		
		FT_Park_Transform(&Clark,&sc_a,&Park);
		//
		Ipark.d = 0;
		Ipark.q = 32768;
		FT_InvPark_Transform(&Ipark,&sc_a,&Clark2);
		
		
//		step_timer++;
//    if (step_timer < 5000) {
//        my_pid.Ref = 0;                     // 目标值给 0
//    } else if (step_timer < 10000) {
//        my_pid.Ref = 32768;                 // 目标值给 0.5 PU (16384对应12V/24V)
//    } else {
//        step_timer = 0;
//    }

//    // 3. 运行你的定点化 PID 计算
//    FS_PID_Calculate((PID_t *)&my_pid, test_fdb);

    // 4. 模拟虚拟电机（一阶惯性环节）
    // 反馈值 test_fdb 追随 Output，">> 5" 控制滞后时间（数值越大，电机惯性越大，追得越慢）
//    test_fdb += (my_pid.Output - test_fdb) >> 5;
		
		SVPWM_Components.T_pwm  = 3500;
		SVPWM_Components.U_dc   = 65536;
		SVPWM_Components.U_alpha = Clark2.alpha;
		SVPWM_Components.U_beta = Clark2.beta;
		FS_SVPWM_Calculate(&SVPWM_Components);
		

    // 软件延时，防止跑得太快 J-Scope 采样率跟不上
    for(volatile uint32_t i = 0; i < 20000; i++);

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
