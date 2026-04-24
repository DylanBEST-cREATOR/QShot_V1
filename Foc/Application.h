#ifndef __APPLICATION_H
#define __APPLICATION_H
#include "Midware.h"
#include "common.h"


typedef struct{
	 u8 Speed_Error;
	 u8 Position_Error;
	u8 Current_Error;
	u8 Voltage_Error;
	u8 Else_Error;
	u8 System_OK ;
}System_Status_t;

extern void MySystemInit(void);
/* led application */
extern void LEDStatus_Function(System_Status_t *System_Status);
extern System_Status_t System_Status;
extern u16 Response1;

extern void GateDriverInit(void);
#endif
