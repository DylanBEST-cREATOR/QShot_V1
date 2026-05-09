#include "FOC_PID.h"

void FP_PID_Calculate(volatile PID_t *PID, volatile float measured_value) {
    PID->Fdb = measured_value;
    PID->Error = PID->Ref - PID->Fdb;
   

    float p_out = PID->Kp * PID->Error;


    PID->Integral +=PID->Ki * PID->Error;
    

    if (PID->Integral > PID->Out_Max) PID->Integral = PID->Out_Max;
    else if (PID->Integral < PID->Out_Min) PID->Integral = PID->Out_Min;
    float d_out = PID->Kd * (PID->Last_Fdb - PID->Fdb); 
    PID->Last_Fdb = PID->Fdb; 

	
    PID->Output = p_out +PID->Integral + d_out;


    if (PID->Output > PID->Out_Max) PID->Output = PID->Out_Max;
    else if (PID->Output < PID->Out_Min) PID->Output = PID->Out_Min;
}
