#include "stm32f4xx_hal.h"
#include "APP_Control.h"
extern MotorState_t volatile AC_Motor_State;
extern SystemDataInit_t  SystemData_InitStructure;
Current_Sense_t CurrentSense;

 FOC_Core_t FOC_Core_CTS;
 extern volatile uint16_t AS5600_Zero_Offset;
extern volatile uint16_t Global_AS5600_Raw_Angle;
// extern volatile uint16_t MT6701_Zero_Offset;
//extern volatile uint16_t Global_MT6701_Raw_Angle;


  static int32_t angle;
	FOC_Core_t test_cts;
	uint8_t Debug_Toggle_Var = 0;
	
	PID_t Speed_PID;            // 速度环 PID 实例
q16_t Target_RPM =800<<16;       // 用户设定的目标转速
uint16_t Last_Encoder_Val = 0; 
LPF_t Speed_Filter;
	
VirtualAngle_t VAngle_Gen = {
    .Ctrl_Freq_Hz = 20000, // 20kHz PWM
    .Current_Angle = 0
};


	
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
//	  HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_2);
    if (hadc->Instance != ADC1) return;

    switch (AC_Motor_State)
    {
        case CALIBRATING:
            if (AI_OffsetCalibration_Process(&FOC_Core_CTS.current) == 1)
            {
                AC_Motor_State = ROTOR_ALIGN;
            }
            break;

        case ROTOR_ALIGN:
            {
                static uint32_t Align_Counts = 0;

          
                FOC_Core_CTS.svpwm.U_dc = 65536;
                FOC_Core_CTS.svpwm.T_pwm = 8400;
                FOC_Core_CTS.angle = 0; 
								FOC_Core_CTS.svpwm.U_alpha = 10000; 
								FOC_Core_CTS.svpwm.U_beta = 0;
                
                FS_SVPWM_Calculate(&FOC_Core_CTS.svpwm);
                
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, FOC_Core_CTS.svpwm.Duty_A);
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, FOC_Core_CTS.svpwm.Duty_B);
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, FOC_Core_CTS.svpwm.Duty_C);


                if (++Align_Counts >= 20000) 
                {
                    Align_Counts = 0;
  
                    FOC_Core_CTS.pid_d.Integral = 0;
                    FOC_Core_CTS.pid_q.Integral = 0;
                    AC_Motor_State = MOTOR_RUNNING;
									  FM_VirtualAngle_SetSpeed(&VAngle_Gen, 500, 7);
                }
            }
            break;

      case MOTOR_RUNNING:
			      { 


//                HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_2);
//							HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_SET);
                FOC_Core_CTS.angle = FS_Get_Electrical_Angle(Global_AS5600_Raw_Angle, AS5600_Zero_Offset, 7, 0);
//							  FOC_Core_CTS.angle = FM_VirtualAngle_Update(&VAngle_Gen);


                FOC_Core_CTS.current.adc_raw_2 = LL_ADC_INJ_ReadConversionData32(ADC1, LL_ADC_INJ_RANK_1);
                FOC_Core_CTS.current.adc_raw_3 = LL_ADC_INJ_ReadConversionData32(ADC2, LL_ADC_INJ_RANK_1);
							
           
                FOC_Core_CTS.pid_q.Ref = 32768; //正转
                FOC_Core_CTS.pid_d.Ref = 0;

         
                FC_FOC_Core(&FOC_Core_CTS);

       
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, FOC_Core_CTS.svpwm.Duty_A);
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, FOC_Core_CTS.svpwm.Duty_B);
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, FOC_Core_CTS.svpwm.Duty_C);
//								HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_RESET);


            }
            break;
                            
        default:
            __HAL_TIM_MOE_DISABLE(&htim1);
            break;
    }
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
   if (htim->Instance == TIM3) { // 1ms 定时器
//		    HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_2);
        // --- 1. 获取原始位置 ---
        Global_AS5600_Raw_Angle = _ME_AS5600_Read_Raw_Angle();

        // --- 2. 计算实际转速并滤波 ---
        // 编码器噪声大，不滤波速度环很难调
        q16_t raw_rpm = AS5600_Calculate_RPM(Global_AS5600_Raw_Angle, Last_Encoder_Val);
        q16_t filtered_rpm = FM_LPF_Calculate(&Speed_Filter, raw_rpm);
        
        Last_Encoder_Val = Global_AS5600_Raw_Angle;

        // --- 3. 执行速度控制 (完全解耦) ---
        // 速度环并不修改 FOC 结构体，它只吐出一个电流指令
        q16_t iq_cmd = FC_Speed_Loop_Execute(&Speed_PID, Target_RPM, filtered_rpm);

        // --- 4. 喂给电流环 ---
        if (AC_Motor_State == MOTOR_RUNNING) {
//            FOC_Core_CTS.pid_q.Ref = iq_cmd;
//            FOC_Core_CTS.pid_d.Ref = 0; // 通常 D 轴为 0
        } else {
//            FOC_Core_CTS.pid_q.Ref = 0;
//            Speed_PID.Integral = 0; // 非运行状态清除积分
        }
    }
}







