#include "stm32f4xx_hal.h"
#include "APP_Control.h"
#include "host_comm.h"
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
q16_t Target_RPM =32768;       // 用户设定的目标转速
uint16_t Last_Encoder_Val = 0; 
LPF_t Speed_Filter;
volatile int32_t g_motor_speed_rpm = 0;

VirtualAngle_t VAngle_Gen = {
.Ctrl_Freq_Hz = 20000, // 20kHz PWM
.Current_Angle = 0
};
SVPWM_t SVPWM_TE;

extern HostComm_Manager_t g_host_comm;



#define SPEED_BASE_RPM      1800
#define IQ_REF_LIMIT_Q16    32768     // 0.5PU
#define SPEED_REF_LIMIT_RPM 1800

static inline int32_t clamp_i32_local(int32_t x, int32_t min, int32_t max)
{
    if (x > max) return max;
    if (x < min) return min;
    return x;
}

static inline q16_t RPM_To_SpeedPU(int32_t rpm)
{
    return (q16_t)(((int64_t)rpm * 65536) / SPEED_BASE_RPM);
}

static inline int32_t SpeedPU_To_RPM(q16_t speed_pu)
{
    return (int32_t)(((int64_t)speed_pu * SPEED_BASE_RPM) / 65536);
}

	
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
									  AS5600_Zero_Offset = _ME_AS5600_Read_Raw_Angle();
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
								FOC_Core_CTS.current.adc_raw_2 =
								LL_ADC_INJ_ReadConversionData32(ADC1, LL_ADC_INJ_RANK_1);

								FOC_Core_CTS.current.adc_raw_3 =
								LL_ADC_INJ_ReadConversionData32(ADC2, LL_ADC_INJ_RANK_1);

								FS_ADCValueToCurrent(&FOC_Core_CTS.current);
							  
							  FOC_Core_CTS.angle = FS_Get_Electrical_Angle(Global_AS5600_Raw_Angle,AS5600_Zero_Offset,7,0);
							  FOC_Core_CTS.pid_d.Ref = 0;
//							  FOC_Core_CTS.pid_q.Ref = 8000;
							  FC_FOC_Core(&FOC_Core_CTS);
							  
								__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, FOC_Core_CTS.svpwm.Duty_A);
								__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, FOC_Core_CTS.svpwm.Duty_B);
								__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, FOC_Core_CTS.svpwm.Duty_C);
							
			
  
						//    q16_t v_mag = _Q16(0.04f);
						//    q16_t angle_step = OpenLoop_CalcAngleStep(100, MOTOR_POLE_PAIRS);
						//    FOC_Core_CTS.svpwm.T_pwm = SVPWM_TPWM_TICKS;
						//    FC_FOC_OpenLoop_Rotate(&FOC_Core_CTS, v_mag, angle_step);
						//		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, FOC_Core_CTS.svpwm.Duty_A);
						//		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, FOC_Core_CTS.svpwm.Duty_B);
						//		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, FOC_Core_CTS.svpwm.Duty_C);
   
}
break;




   
               


            
       
                            
        default:
            __HAL_TIM_MOE_DISABLE(&htim1);
            break;
    }
		 HostComm_ISR_Snapshot(&g_host_comm);
}


extern volatile int32_t g_motor_speed_rpm;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)
    {
        Global_AS5600_Raw_Angle = _ME_AS5600_Read_Raw_Angle();

        q16_t raw_speed_pu =
            AS5600_Calculate_Speed_PU(Global_AS5600_Raw_Angle,
                                      Last_Encoder_Val);

        q16_t filtered_speed_pu =
            FM_LPF_Calculate(&Speed_Filter, raw_speed_pu);

        Last_Encoder_Val = Global_AS5600_Raw_Angle;

        /*
         * 给网页显示用的实际 rpm。
         */
        g_motor_speed_rpm = SpeedPU_To_RPM(filtered_speed_pu);

        if (AC_Motor_State == MOTOR_RUNNING)
        {
					  if (g_host_comm.ctrl_mode == MODE_CURR || g_host_comm.ctrl_mode == MODE_SPD) {
        __HAL_TIM_MOE_ENABLE(&htim1);   // <-- 加这一句
    }
            if (g_host_comm.ctrl_mode == MODE_CURR)
            {
                int32_t iq_ref = clamp_i32_local(g_host_comm.setpoint,
                                                 -IQ_REF_LIMIT_Q16,
                                                  IQ_REF_LIMIT_Q16);

                FOC_Core_CTS.pid_d.Ref = 0;
                FOC_Core_CTS.pid_q.Ref = iq_ref;

                Speed_PID.Integral = 0;
            }
            else if (g_host_comm.ctrl_mode == MODE_SPD)
            {
                int32_t speed_ref_rpm =
                    clamp_i32_local(g_host_comm.setpoint,
                                    -SPEED_REF_LIMIT_RPM,
                                     SPEED_REF_LIMIT_RPM);

                q16_t target_speed_pu =
                    RPM_To_SpeedPU(speed_ref_rpm);

                q16_t iq_cmd =
                    FC_Speed_Loop_Execute(&Speed_PID,
                                          target_speed_pu,
                                          filtered_speed_pu);

                iq_cmd = clamp_i32_local(iq_cmd,
                                         -SPEED_PID_OUT_MAX,
                                          SPEED_PID_OUT_MAX);

                FOC_Core_CTS.pid_d.Ref = 0;
                FOC_Core_CTS.pid_q.Ref = iq_cmd;
            }
            else
            {
                FOC_Core_CTS.pid_d.Ref = 0;
                FOC_Core_CTS.pid_q.Ref = 0;
                Speed_PID.Integral = 0;
            }
        }
        else
        {
            FOC_Core_CTS.pid_d.Ref = 0;
            FOC_Core_CTS.pid_q.Ref = 0;
            Speed_PID.Integral = 0;
        }
    }
}








