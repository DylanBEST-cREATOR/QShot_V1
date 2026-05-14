#ifndef __APP_CONTROL_H
#define __APP_CONTROL_H

#include "APP_Interface.h"
#include "FOC_Core.h"
typedef enum {
     CALIBRATING = 0,
	   CALIBRATION_DONE,
     ROTOR_ALIGN,
     MOTOR_RUNNING
} MotorState_t;

typedef struct{
	Current_Offset_t OffsetData_Init;


}SystemDataInit_t;
extern void AC_MySystem_Init(void);


#endif 