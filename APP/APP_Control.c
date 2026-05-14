#include "APP_Control.h"

SystemDataInit_t  SystemData_InitStructure;


MotorState_t volatile AC_Motor_State;

extern  FOC_Core_t FOC_Core_CTS;



void AC_MySystem_Init(void)
{

//	  __disable_irq();
     DRV8301_Init();
	 		 HAL_TIM_Base_Start_IT(&htim3);
    FC_Core_Init(&FOC_Core_CTS,8400);
    // 1. 开启 ADC 偏置校准（此时 PWM 是关闭的，MOE=0）
    AI_ADCOffset_Config();
    
    // 2. 等待校准完成
    while(AC_Motor_State == CALIBRATING);

    // 3. 当状态变为 ROTOR_ALIGN 时，在这里开启 PWM 输出
    if(AC_Motor_State == ROTOR_ALIGN)
    {
        // 开启全桥驱动，仅执行一次
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
        HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
        HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
        HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
        
        // 如果是高级定时器 TIM1，必须使能主输出 (MOE)
        __HAL_TIM_MOE_ENABLE(&htim1); 
    }

    // 4. 等待预定位结束
    while(AC_Motor_State == ROTOR_ALIGN);

    
    // 5. 进入运行模式
    // AC_Motor_State = MOTOR_RUNNING; // 中断里已经切换了
}

