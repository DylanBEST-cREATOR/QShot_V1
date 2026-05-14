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
 q16_t Target_Speed_RPM =(50<<16);
 PID_t Speed_PID;
  static int32_t angle;
	FOC_Core_t test_cts;
	uint8_t Debug_Toggle_Var = 0;
	
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

          
                FOC_Core_CTS.svpwm.U_dc = 32768;
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



                FOC_Core_CTS.angle = FS_Get_Electrical_Angle(Global_AS5600_Raw_Angle, AS5600_Zero_Offset, 7, 0);
//							  FOC_Core_CTS.angle = FM_VirtualAngle_Update(&VAngle_Gen);


                FOC_Core_CTS.current.adc_raw_2 = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
                FOC_Core_CTS.current.adc_raw_3 = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
            
                FOC_Core_CTS.pid_q.Ref = 3000; //正转
                FOC_Core_CTS.pid_d.Ref = 0;

         
                FC_FOC_Core(&FOC_Core_CTS);

       
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, FOC_Core_CTS.svpwm.Duty_A);
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, FOC_Core_CTS.svpwm.Duty_B);
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, FOC_Core_CTS.svpwm.Duty_C);


            }
            break;
                            
        default:
            __HAL_TIM_MOE_DISABLE(&htim1);
            break;
    }
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3) 
    {
        Global_AS5600_Raw_Angle =   _ME_AS5600_Read_Raw_Angle();
    }
}






